#include <FS.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/* =====================
   FIRMWARE INFO
   ===================== */
static const char* FW_NAME    = "thingsstring-smart-valve-esp32";
static const char* FW_VERSION = "1.0.0";

/* =====================
   USER / OWNER
   ===================== */
char user_id[40] = "";   // default platform user id

/* =====================
   MQTT
   ===================== */
const char* MQTT_HOST = "c3cfb0615d534d41b7b5ce4a9291d5e8.s1.eu.hivemq.cloud";
const uint16_t MQTT_PORT = 8883;

#define USE_MQTT_AUTH true
const char* MQTT_USER = "admin";
const char* MQTT_PASS_MQTT = "Admin123";

static const char* TOPIC_ROOT = "ts";

/* =====================
   Debug & Console
   ===================== */
bool runtimeDebugEnabled = false;

#define DPRINT(x)      if(runtimeDebugEnabled) Serial.print(x)
#define DPRINTLN(x)    if(runtimeDebugEnabled) Serial.println(x)
#define DPRINTF(...)   if(runtimeDebugEnabled) Serial.printf(__VA_ARGS__)

const char* CONSOLE_PASS = "admin123";
bool consoleAuthenticated = false;
String serialInput = "";

const char* ASCII_ART = R"rawliteral(
  _____ _     _                 ____  _        _             
 |_   _| |__ (_)_ __   __ _ ___/ ___|| |_ _ __(_)_ __   __ _ 
   | | | '_ \| | '_ \ / _` / __\___ \| __| '__| | '_ \ / _` |
   | | | | | | | | | | (_| \__ \___) | |_| |  | | | | | (_| |
   |_| |_| |_|_|_| |_|\__, |___/____/ \__|_|  |_|_| |_|\__, |
                      |___/                            |___/ 
                              www.thingsstring.com
)rawliteral";

/* =====================
   Hardware Configuration
   ===================== */
// Solenoid Relays (Latching Control)
const int RELAY_OPEN_PIN  = 16;
const int RELAY_CLOSE_PIN = 17;

// Pin Definitions
const int CONFIG_BUTTON   = 0;  // GPIO 0 - BOOT button & physical manual toggle
const int STATUS_LED      = 15; // Status Indicator LED
const int BATT_SENSE_PIN  = 4;  // Battery Voltage Sense Pin

#define LED_ACTIVE_LOW false

// Constants
const unsigned long LATCH_PULSE_MS = 500;  // Pulse duration for latching
const unsigned long DEBOUNCE_DELAY  = 50;   // Button debounce delay

// Battery Voltage Configuration
const float VOLTAGE_DIVIDER_RATIO = 2.0; 
const float ADC_REF_VOLTAGE       = 3.3;
const float ADC_RESOLUTION        = 4095.0;

/* =====================
   Status States
   ===================== */
enum StatusState {
  ST_INIT,
  ST_WIFI_CONNECTING,
  ST_AP_MODE,
  ST_MQTT_CONNECTING,
  ST_ONLINE,
  ST_ERROR
};
StatusState currentStatus = ST_INIT;

/* =====================
   Globals
   ===================== */
WiFiClientSecure net;
PubSubClient mqtt(net);
WebServer server(80);
DNSServer dnsServer;

String chipId, deviceId;
String topicConfig, topicTelemetry, topicControl, topicStatus;

bool configSent = false;
bool isOpen = false;

// Relay pulse tracking
unsigned long relayPulseStartTime = 0;
int activeRelayPin = -1;

// Physical Button Debouncing
bool lastButtonState = HIGH;
bool buttonState = HIGH;
unsigned long lastDebounceTime = 0;

// Config / Reset Button hold variables
uint32_t buttonPressTime = 0;
bool buttonPressed = false;
bool portalRequested = false;

// I2C LCD (0x27, 20x4)
LiquidCrystal_I2C lcd(0x27, 20, 4);
bool lcdFound = false;

// Timing
uint32_t tTelemetry = 0;
uint32_t telemetryIntervalMs = 5000;
uint32_t tLed = 0;
bool ledState = false;

/* =====================
   Battery Reading
   ===================== */
float getBatteryVoltage() {
  int rawValue = analogRead(BATT_SENSE_PIN);
  float voltageAtPin = (rawValue / ADC_RESOLUTION) * ADC_REF_VOLTAGE;
  return voltageAtPin * VOLTAGE_DIVIDER_RATIO;
}

/* =====================
   LCD Logic
   ===================== */
void updateLCD() {
  if (!lcdFound) return;
  
  lcd.setCursor(0, 0);
  lcd.print("   Smart Valve Pro  ");
  
  lcd.setCursor(0, 1);
  switch (currentStatus) {
    case ST_ONLINE:          lcd.print("Status: ONLINE      "); break;
    case ST_WIFI_CONNECTING: lcd.print("Status: WIFI...     "); break;
    case ST_AP_MODE:         lcd.print("Status: CONFIG MODE "); break;
    case ST_MQTT_CONNECTING: lcd.print("Status: MQTT...     "); break;
    default:                 lcd.print("Status: OFFLINE     "); break;
  }

  // Row 2: Valve State
  lcd.setCursor(0, 2);
  lcd.printf("Valve: %s            ", isOpen ? "OPEN" : "CLOSED");

  // Row 3: Battery and IP suffix
  lcd.setCursor(0, 3);
  float battV = getBatteryVoltage();
  lcd.printf("V:%4.2fV ", battV);
  
  if (WiFi.status() == WL_CONNECTED) {
    lcd.print("IP:.");
    String ip = WiFi.localIP().toString();
    lcd.print(ip.substring(ip.lastIndexOf('.')));
  } else {
    lcd.print("NO IP ");
  }
}

/* =====================
   Status LED Logic
   ===================== */
void updateLED() {
  uint32_t now = millis();
  static uint32_t lastLoop = 0;
  static float fadeValue = 0;
  static int fadeDir = 1;

  auto writeLED = [](int duty) {
    analogWrite(STATUS_LED, LED_ACTIVE_LOW ? (255 - duty) : duty);
  };

  if (currentStatus == ST_AP_MODE) {
    if (now - lastLoop > 10) {
      lastLoop = now;
      fadeValue += (fadeDir * 2.5); 
      if (fadeValue >= 255) { fadeValue = 255; fadeDir = -1; }
      if (fadeValue <= 0) { fadeValue = 0; fadeDir = 1; }
      writeLED((int)fadeValue);
    }
    return;
  }

  uint32_t interval = 0;
  switch (currentStatus) {
    case ST_WIFI_CONNECTING: interval = 200; break;
    case ST_MQTT_CONNECTING: interval = 500; break;
    case ST_ONLINE:          interval = 2000; break; 
    case ST_ERROR:           interval = 100; break;
    default:                 interval = 0; break;
  }

  if (interval == 0) {
    writeLED(0);
    return;
  }

  if (currentStatus == ST_ONLINE) {
    uint32_t sub = now % interval;
    bool state = (sub < 100) || (sub > 200 && sub < 300);
    writeLED(state ? 255 : 0);
    return;
  }

  if (now - tLed >= interval) {
    tLed = now;
    ledState = !ledState;
    writeLED(ledState ? 255 : 0);
  }
}

/* =====================
   Persistence
   ===================== */
void saveValveState() {
  File file = LittleFS.open("/states.json", "w");
  if (file) {
    StaticJsonDocument<256> doc;
    doc["isOpen"] = isOpen;
    serializeJson(doc, file);
    file.close();
  }
}

void loadValveState() {
  if (LittleFS.exists("/states.json")) {
    File file = LittleFS.open("/states.json", "r");
    if (file) {
      StaticJsonDocument<256> doc;
      DeserializationError err = deserializeJson(doc, file);
      if (!err) {
        isOpen = doc["isOpen"] | false;
      }
      file.close();
    }
  }
}

void saveProvisioningStatus() {
  File file = LittleFS.open("/provisioned.txt", "w");
  if (file) {
    file.print("1");
    file.close();
  }
}

void loadProvisioningStatus() {
  if (LittleFS.exists("/provisioned.txt")) {
    configSent = true;
    DPRINTLN("[SYSTEM] Device already provisioned. Skipping config publish.");
  } else {
    configSent = false;
    DPRINTLN("[SYSTEM] Device not provisioned. Will send config on next connection.");
  }
}

void factoryReset() {
  DPRINTLN("[SYSTEM] Factory Resetting...");
  if (lcdFound) {
    lcd.clear();
    lcd.print(" FACTORY RESET ");
  }
  // Flicker LED for feedback
  for (int i = 0; i < 20; i++) {
    digitalWrite(STATUS_LED, HIGH); delay(50);
    digitalWrite(STATUS_LED, LOW); delay(50);
  }
  if (LittleFS.exists("/config.json")) LittleFS.remove("/config.json");
  if (LittleFS.exists("/states.json")) LittleFS.remove("/states.json");
  if (LittleFS.exists("/provisioned.txt")) LittleFS.remove("/provisioned.txt");
  DPRINTLN("[SYSTEM] Restarting...");
  ESP.restart();
}

bool checkI2C(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

/* =====================
   Relay Control Logic
   ===================== */
void pulseRelay(int pin) {
  // Turn off any currently active pulse first
  if (activeRelayPin != -1) digitalWrite(activeRelayPin, LOW);
  
  digitalWrite(pin, HIGH);
  relayPulseStartTime = millis();
  activeRelayPin = pin;
}

void handleRelayTimeout() {
  if (activeRelayPin != -1 && (millis() - relayPulseStartTime >= LATCH_PULSE_MS)) {
    digitalWrite(activeRelayPin, LOW);
    activeRelayPin = -1;
  }
}

void toggleValve() {
  isOpen = !isOpen;
  
  if (isOpen) {
    Serial.println("Opening Valve...");
    pulseRelay(RELAY_OPEN_PIN);
  } else {
    Serial.println("Closing Valve...");
    pulseRelay(RELAY_CLOSE_PIN);
  }
  
  updateLCD();
}

/* =====================
   MQTT Communication
   ===================== */
bool publishJson(const String& topic, JsonDocument& doc, bool retained=false) {
  static char buf[1024]; 
  size_t n = serializeJson(doc, buf, sizeof(buf));
  if (n == 0) return false;
  bool ok = mqtt.publish(topic.c_str(), (const uint8_t*)buf, n, retained);
  DPRINTF("[PUB] %s | Retained: %d | OK: %s\n", topic.c_str(), retained, ok ? "YES" : "NO");
  return ok;
}

void publishTelemetry() {
  StaticJsonDocument<512> d;
  d["id"] = deviceId;
  d["up"] = millis();
  
  JsonObject a = d.createNestedObject("a");
  JsonObject v = a.createNestedObject("valve");
  v["state"] = isOpen ? "OPEN" : "CLOSED";

  d["vb"] = getBatteryVoltage();
  
  publishJson(topicTelemetry, d, false);
}

void publishConfig() {
  StaticJsonDocument<1024> cfg;
  cfg["user_id"] = user_id;

  JsonObject device = cfg.createNestedObject("device");
  device["device_id"] = deviceId;
  device["user_id"]   = user_id;
  device["chip_id"]   = chipId;
  device["name"]      = "Smart Solenoid Valve";
  device["model"]     = "TS-SMART-VALVE";
  device["fw_name"]   = FW_NAME;
  device["fw"]        = FW_VERSION;

  JsonObject topics = cfg.createNestedObject("topics");
  topics["telemetry"] = topicTelemetry;
  topics["control"]   = topicControl;
  topics["status"]    = topicStatus;
  topics["config"]    = topicConfig;

  JsonObject sensors = cfg.createNestedObject("sensors");
  sensors["battery_v"] = true;

  JsonObject actuators = cfg.createNestedObject("actuators");
  JsonObject v = actuators.createNestedObject("valve");
  v["type"] = "valve";
  v["name"] = "Water Valve";
  JsonObject vDef = v.createNestedObject("default");
  vDef["state"] = "CLOSED";
  vDef["auto"]  = true;

  publishJson(topicConfig, cfg, true);
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  DPRINTF("MQTT RX [%s]\n", topic);
  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, payload, len)) return;

  JsonObject a = doc["actuators"];
  if (!a) return;

  bool changed = false;
  if (a["valve"]) {
    const char* st = a["valve"]["state"];
    bool newState = (st && (!strcmp(st,"OPEN") || !strcmp(st,"ON") || !strcmp(st,"1") || !strcmp(st,"true")));
    if (isOpen != newState) {
      isOpen = newState;
      changed = true;
    }
  }

  if (changed) {
    if (isOpen) {
      Serial.println("Opening Valve via MQTT...");
      pulseRelay(RELAY_OPEN_PIN);
    } else {
      Serial.println("Closing Valve via MQTT...");
      pulseRelay(RELAY_CLOSE_PIN);
    }
    saveValveState();
    publishTelemetry();
    updateLCD();
  }
}

bool mqttConnect() {
  String clientId = "ts_valve_" + deviceId;
  currentStatus = ST_MQTT_CONNECTING;
  updateLCD();
  
  Serial.println("[SYSTEM] Connecting to Cloud...");
  DPRINTF("MQTT Host: %s:%d | ID: %s\n", MQTT_HOST, MQTT_PORT, clientId.c_str());
  
  bool ok = mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS_MQTT,
                        topicStatus.c_str(), 1, true, "offline", true);
  
  if (ok) {
    currentStatus = ST_ONLINE;
    Serial.println("[SYSTEM] MQTT Connected! Cloud Online.");
    mqtt.subscribe(topicControl.c_str());
    DPRINTF("Subscribed to: %s\n", topicControl.c_str());
    
    StaticJsonDocument<128> s;
    s["id"] = deviceId;
    s["status"] = "online";
    char buf[128];
    serializeJson(s, buf);
    mqtt.publish(topicStatus.c_str(), (const uint8_t*)buf, strlen(buf), true);
    
    if (!configSent) {
      Serial.println("[SYSTEM] Sending device configuration for the first time...");
      publishConfig();
      configSent = true;
      saveProvisioningStatus();
    }
    updateLCD();
  } else {
    DPRINTF("MQTT Connection Failed, rc=%d. Will retry...\n", mqtt.state());
  }
  return ok;
}

/* =====================
   WiFi Config Portal
   ===================== */
void handleSave() {
  String uid  = server.arg("user_id");
  String ssid = server.arg("ssid");
  String pass = server.arg("pass");
  
  StaticJsonDocument<512> json;
  json["user_id"] = uid;
  json["ssid"] = ssid;
  json["pass"] = pass;

  File configFile = LittleFS.open("/config.json", "w");
  if (configFile) {
    serializeJson(json, configFile);
    configFile.close();
  }
  
  String successHtml = R"rawliteral(
<!DOCTYPE html>
<html lang='en'>
<head>
    <meta charset='UTF-8'>
    <meta name='viewport' content='width=device-width, initial-scale=1.0'>
    <title>Success | ThingsString</title>
    <style>
        :root { --bg: #ffffff; --text: #000000; --border: #e0e0e0; }
        body { margin: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; background: var(--bg); color: var(--text); display: flex; justify-content: center; align-items: center; min-height: 100vh; text-align: center; }
        .container { width: 90%; max-width: 400px; padding: 40px 20px; }
        .icon { font-size: 48px; margin-bottom: 20px; }
        h2 { font-size: 24px; font-weight: 800; text-transform: uppercase; letter-spacing: -0.5px; margin: 0 0 10px; }
        p { color: #666; font-size: 14px; line-height: 1.6; margin-bottom: 30px; }
        .loader-bar { height: 2px; width: 100%; background: #f0f0f0; position: relative; overflow: hidden; margin-bottom: 30px; }
        .loader-progress { position: absolute; height: 100%; background: #000; width: 0%; animation: load 3s linear forwards; }
        @keyframes load { from { width: 0%; } to { width: 100%; } }
        .btn { display: inline-block; padding: 15px 30px; border: 1px solid #000; color: #000; text-decoration: none; font-size: 12px; font-weight: 700; text-transform: uppercase; letter-spacing: 1px; transition: all 0.2s; }
        .btn:hover { background: #000; color: #fff; }
        @media (prefers-color-scheme: dark) {
            :root { --bg: #000000; --text: #ffffff; }
            .loader-bar { background: #111; }
            .loader-progress { background: #fff; }
            .btn { border-color: #fff; color: #fff; }
            .btn:hover { background: #fff; color: #000; }
        }
    </style>
    <script>
        setTimeout(() => { window.location.href = 'https://www.thingsstring.com/dashboard'; }, 4000);
    </script>
</head>
<body>
    <div class='container'>
        <div class='icon'>✓</div>
        <h2>Config Saved</h2>
        <p>Your device is rebooting and connecting to ThingsString Cloud. You will be redirected to your dashboard shortly.</p>
        <div class='loader-bar'><div class='loader-progress'></div></div>
        <a href='https://www.thingsstring.com/dashboard' class='btn'>Go to Dashboard</a>
    </div>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", successHtml);
  Serial.printf("[SYSTEM] Config Saved. SSID: %s. Restarting...\n", ssid.c_str());
  delay(1000);
  ESP.restart();
}

void handleWifiScan() {
  DPRINTLN("Scanning for WiFi networks...");
  int n = WiFi.scanNetworks();
  StaticJsonDocument<1536> doc;
  JsonArray array = doc.to<JsonArray>();
  for (int i = 0; i < n; ++i) {
    JsonObject obj = array.createNestedObject();
    obj["ssid"] = WiFi.SSID(i);
    obj["rssi"] = WiFi.RSSI(i);
    obj["enc"]  = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "open" : "secure";
  }
  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang='en'>
<head>
    <meta charset='UTF-8'>
    <meta name='viewport' content='width=device-width, initial-scale=1.0'>
    <title>ThingsString | Device Config</title>
    <style>
        :root { --bg: #ffffff; --text: #000000; --accent: #000000; --input-bg: #f5f5f5; --border: #e0e0e0; }
        body { margin: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; background: var(--bg); color: var(--text); display: flex; justify-content: center; align-items: flex-start; min-height: 100vh; padding-top: 40px; }
        .container { width: 90%; max-width: 400px; padding: 20px; box-sizing: border-box; }
        .header { text-align: center; margin-bottom: 40px; }
        .logo { font-size: 24px; font-weight: 800; letter-spacing: -1px; text-transform: uppercase; margin-bottom: 5px; }
        .subtitle { font-size: 12px; color: #666; letter-spacing: 1px; }
        h3 { font-size: 14px; font-weight: 600; margin: 30px 0 15px; text-transform: uppercase; letter-spacing: 0.5px; border-bottom: 1px solid var(--border); padding-bottom: 8px; }
        .wifi-list { margin-bottom: 20px; border: 1px solid var(--border); border-radius: 8px; overflow: hidden; }
        .wifi-item { padding: 15px; border-bottom: 1px solid var(--border); cursor: pointer; display: flex; justify-content: space-between; align-items: center; transition: background 0.2s; }
        .wifi-item:last-child { border-bottom: none; }
        .wifi-item:hover { background: var(--input-bg); }
        .wifi-name { font-weight: 500; font-size: 0.95rem; }
        .wifi-info { display: flex; align-items: center; gap: 8px; font-size: 0.8rem; color: #888; }
        .form-group { margin-bottom: 20px; }
        label { display: block; font-size: 11px; font-weight: 600; color: #888; margin-bottom: 8px; text-transform: uppercase; }
        input { width: 100%; padding: 14px; border-radius: 4px; border: 1px solid var(--border); background: var(--input-bg); color: var(--text); box-sizing: border-box; font-size: 1rem; transition: border-color 0.2s; }
        input:focus { outline: none; border-color: var(--accent); }
        .save-action { width: 100%; padding: 20px; background: transparent; color: var(--text); font-weight: 800; cursor: pointer; transition: all 0.2s; font-size: 14px; text-transform: uppercase; letter-spacing: 2px; border: 1px solid var(--text); border-radius: 0; margin-top: 20px; }
        .save-action:hover { background: var(--text); color: var(--bg); }
        .footer { text-align: center; margin-top: 40px; font-size: 11px; color: #aaa; }
        .footer a { color: #888; text-decoration: none; }
        .signal-bars { display: flex; align-items: flex-end; gap: 2px; height: 12px; }
        .bar { width: 3px; background: #ddd; border-radius: 1px; }
        .bar.active { background: #000; }
        .loader { text-align: center; padding: 30px; font-size: 0.9rem; color: #888; }
        @media (prefers-color-scheme: dark) {
            :root { --bg: #000000; --text: #ffffff; --input-bg: #111111; --border: #222222; }
            .bar.active { background: #fff; }
        }
    </style>
</head>
<body>
    <div class='container'>
        <div class='header'>
            <div class='logo'>ThingsString</div>
            <div class='subtitle'>Smart Solenoid Valve</div>
        </div>
        
        <h3>1. Select Network</h3>
        <div id='wifi-section'>
            <div class='wifi-list' id='list'><div class='loader'>Searching for networks...</div></div>
        </div>
        
        <h3>2. Device Settings</h3>
        <form action='/save' method='POST'>
            <div class='form-group'>
                <label>Wi-Fi SSID</label>
                <input name='ssid' id='ssid' placeholder='Selected network' required>
            </div>
            <div class='form-group'>
                <label>Wi-Fi Password</label>
                <div style='position:relative;'>
                    <input name='pass' id='pass' type='password' placeholder='Leave blank if open'>
                    <span id='togglePass' style='position:absolute; right:15px; top:50%; transform:translateY(-50%); cursor:pointer; font-size:10px; font-weight:800; color:#888; text-transform:uppercase;'>Show</span>
                </div>
            </div>
            <div class='form-group'>
                <label>User Identity (UID)</label>
                <input name='user_id' value='__USER_ID__' placeholder='Your ThingsString UID' required>
            </div>
            <div style='font-size:11px; color:#888; margin: 10px 0 20px; text-align: center;'>Device ID: <span style='color:var(--text); font-weight: 600;'>__DEVICE_ID__</span></div>
            <button type='submit' class='save-action'>Save Settings &rarr;</button>
        </form>

        <div class='footer'>
            &copy; 2024 <a href='https://www.thingsstring.com' target='_blank'>ThingsString IoT</a>
        </div>
    </div>
    <script>
        document.getElementById('togglePass').onclick = function() {
            const p = document.getElementById('pass');
            if(p.type === 'password') {
                p.type = 'text';
                this.innerText = 'Hide';
            } else {
                p.type = 'password';
                this.innerText = 'Show';
            }
        };

        function getBars(rssi) {
            let active = 0;
            if (rssi > -50) active = 4;
            else if (rssi > -60) active = 3;
            else if (rssi > -70) active = 2;
            else if (rssi > -80) active = 1;
            let html = "<div class='signal-bars'>";
            for(let i=1; i<=4; i++) html += `<div class='bar ${i<=active?'active':''} ' style='height:${i*25}%'></div>`;
            return html + "</div>";
        }
        function scan() {
            const list = document.getElementById('list');
            fetch('/scan').then(r => r.json()).then(data => {
                list.innerHTML = "";
                if(data.length == 0) {
                    list.innerHTML = "<div class='loader'>No networks found</div>";
                    return;
                }
                data.sort((a,b) => b.rssi - a.rssi);
                data.forEach(n => {
                    const div = document.createElement('div');
                    div.className = 'wifi-item';
                    div.onclick = () => document.getElementById('ssid').value = n.ssid;
                    div.innerHTML = `<span class='wifi-name'>${n.ssid}</span><div class='wifi-info'><span>${n.enc=='secure'?'🔒':''}</span>${getBars(n.rssi)}</div>`;
                    list.appendChild(div);
                });
            }).catch(e => {
                list.innerHTML = "<div class='loader'>Scan failed. Pull to refresh.</div>";
            });
        }
        window.onload = scan;
    </script>
</body>
</html>
)rawliteral";
  html.replace("__USER_ID__", String(user_id));
  html.replace("__DEVICE_ID__", deviceId);
  server.send(200, "text/html", html);
}

void startConfigPortal() {
  DPRINTLN("----- STARTING CONFIG PORTAL -----");
  currentStatus = ST_AP_MODE;
  updateLCD();
  
  WiFi.mode(WIFI_AP);
  String apName = "TS-VALVE-" + chipId;
  WiFi.softAP(apName.c_str(), "12345678");
  DPRINTF("AP Started: %s | IP: %s\n", apName.c_str(), WiFi.softAPIP().toString().c_str());
  
  dnsServer.start(53, "*", WiFi.softAPIP());
  
  server.on("/", handleRoot);
  server.on("/scan", handleWifiScan);
  server.on("/save", HTTP_POST, handleSave);
  server.begin();

  while (true) {
    dnsServer.processNextRequest();
    server.handleClient();
    updateLED();
    
    // Allow physical button to toggle state during configuration portal
    bool btnReading = digitalRead(CONFIG_BUTTON);
    static bool lastPortalBtnState = HIGH;
    if (btnReading != lastPortalBtnState) {
      lastPortalBtnState = btnReading;
      if (btnReading == LOW) { 
        toggleValve();
        saveValveState();
        delay(150); // Robust debounce delay for portal
      }
    }
    yield();
    delay(1); // Small sleep to save power/prevent heating
  }
}

/* =====================
   Serial Menu
   ===================== */
void showMenu() {
  Serial.println("\n--- THINGSSTRING CONSOLE MENU ---");
  Serial.println("1. Device Status");
  Serial.println("2. Toggle Valve");
  Serial.println("3. Start Config Portal");
  Serial.println("4. Factory Reset");
  Serial.printf("5. Live Debugging: [%s]\n", runtimeDebugEnabled ? "ENABLED" : "DISABLED");
  Serial.println("6. Logout");
  Serial.print("\nSelect Option > ");
}

void handleSerialConsole() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialInput.length() > 0) {
        if (!consoleAuthenticated) {
          if (serialInput == CONSOLE_PASS) {
            consoleAuthenticated = true;
            Serial.println("\n[SUCCESS] Access Granted.");
            showMenu();
          } else {
            Serial.println("\n[ERROR] Incorrect Password.");
            Serial.print("Enter Console Password: ");
          }
        } else {
          // Menu Navigation
          if (serialInput == "1") {
            Serial.println("\n--- DEVICE STATUS ---");
            Serial.printf("Device ID: %s\n", deviceId.c_str());
            Serial.printf("User ID  : %s***\n", String(user_id).substring(0, 4).c_str());
            Serial.printf("WiFi     : %s\n", (WiFi.status() == WL_CONNECTED) ? "CONNECTED" : "DISCONNECTED");
            Serial.printf("MQTT     : %s\n", mqtt.connected() ? "CONNECTED" : "DISCONNECTED");
            Serial.printf("Uptime   : %lu s\n", millis() / 1000);
            Serial.printf("Valve    : %s\n", isOpen ? "OPEN" : "CLOSED");
            Serial.printf("Battery  : %4.2f V\n", getBatteryVoltage());
            showMenu();
          } 
          else if (serialInput == "2") {
            toggleValve();
            saveValveState();
            if (WiFi.status() == WL_CONNECTED && mqtt.connected()) {
              publishTelemetry();
            }
            Serial.printf("\n[ACTION] Valve toggled to %s.\n", isOpen ? "OPEN" : "CLOSED");
            showMenu();
          }
          else if (serialInput == "3") {
            Serial.println("\n[ACTION] Starting Config Portal...");
            portalRequested = true;
          }
          else if (serialInput == "4") {
            Serial.println("\n[WARNING] Perform Factory Reset? (y/n)");
            while(!Serial.available()) yield();
            if (Serial.read() == 'y') factoryReset();
            else showMenu();
          }
          else if (serialInput == "5") {
            runtimeDebugEnabled = !runtimeDebugEnabled;
            Serial.printf("\n[CONFIG] Live Debugging %s.\n", runtimeDebugEnabled ? "Enabled" : "Disabled");
            showMenu();
          }
          else if (serialInput == "6") {
            consoleAuthenticated = false;
            runtimeDebugEnabled = false; // Disable logs on logout
            Serial.println("\n[LOGOUT] Session terminated.");
            Serial.print("\nEnter Console Password: ");
          }
          else {
            Serial.println("\n[ERROR] Invalid choice.");
            showMenu();
          }
        }
        serialInput = "";
      }
    } else {
      serialInput += c;
    }
  }
}

/* =====================
   Setup & Loop
   ===================== */
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(ASCII_ART);
  Serial.printf("[SYSTEM] Firmware   : %s\n", FW_NAME);
  Serial.printf("[SYSTEM] Version    : %s\n", FW_VERSION);
  Serial.print("\nEnter Console Password: ");

  // Device Identity (Calculate Chip ID)
  uint64_t mac = ESP.getEfuseMac();
  char buf_id[13];
  sprintf(buf_id, "%04X%08X", (uint16_t)(mac >> 32), (uint32_t)mac);
  chipId = String(buf_id);
  chipId.toLowerCase();
  deviceId = chipId;

  // Pin Configuration
  pinMode(RELAY_OPEN_PIN, OUTPUT);
  pinMode(RELAY_CLOSE_PIN, OUTPUT);
  pinMode(CONFIG_BUTTON, INPUT_PULLUP);
  pinMode(STATUS_LED, OUTPUT);
  
  // Initialize Relays (OFF)
  digitalWrite(RELAY_OPEN_PIN, LOW);
  digitalWrite(RELAY_CLOSE_PIN, LOW);

  // LCD Initialization
  Wire.begin(); 
  Serial.print("Scanning I2C for LCD at 0x27... ");
  if (checkI2C(0x27)) {
    Serial.println("FOUND");
    lcdFound = true;
    lcd.init(); 
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("   INITIALIZING   ");
    lcd.setCursor(0, 1);
    lcd.print("   SMART VALVE    ");
  } else {
    Serial.println("NOT FOUND. Continuing without LCD.");
  }
  
  // Filesystem and Initial State
  DPRINT("Mounting LittleFS... ");
  if (LittleFS.begin(true)) {
    DPRINTLN("OK");
    loadValveState();
    loadProvisioningStatus();
    
    if (LittleFS.exists("/config.json")) {
      DPRINTLN("Loading config.json");
      File f = LittleFS.open("/config.json", "r");
      StaticJsonDocument<512> json;
      if (!deserializeJson(json, f)) {
        if (json["user_id"]) strncpy(user_id, json["user_id"], sizeof(user_id));
        
        if (json["ssid"] && json["ssid"].as<String>().length() > 0) {
          DPRINTF("Connecting to WiFi: %s\n", json["ssid"].as<const char*>());
          WiFi.mode(WIFI_STA);
          WiFi.begin(json["ssid"].as<const char*>(), json["pass"].as<const char*>());
          currentStatus = ST_WIFI_CONNECTING;
          
          uint32_t startWait = millis();
          while (WiFi.status() != WL_CONNECTED && millis() - startWait < 10000) {
            updateLED();
            delay(10);
            yield();
          }
          if (WiFi.status() == WL_CONNECTED) {
            Serial.print("[SYSTEM] WiFi Connected! IP: ");
            Serial.println(WiFi.localIP());
          }
        }
      }
      f.close();
    }
  }

  Serial.printf("[SYSTEM] Device ID: %s\n", deviceId.c_str());
  Serial.printf("[SYSTEM] User ID  : %s***\n", String(user_id).substring(0, 4).c_str());

  topicTelemetry = String(TOPIC_ROOT) + "/" + deviceId + "/telemetry";
  topicControl   = String(TOPIC_ROOT) + "/" + deviceId + "/control";
  topicStatus    = String(TOPIC_ROOT) + "/" + deviceId + "/status";
  topicConfig    = String(TOPIC_ROOT) + "/" + deviceId + "/config";

  net.setInsecure();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(1024);

  updateLCD();
}

void loop() {
  handleSerialConsole();
  
  // 1. Handle Solenoid Relay Pulses
  handleRelayTimeout();

  // 2. Handle 3-Stage Config / Override Button (GPIO 0)
  bool btnReading = digitalRead(CONFIG_BUTTON);
  bool btnIsPressed = (btnReading == LOW); // BOOT button is active LOW

  if (btnIsPressed) {
    if (!buttonPressed) {
      buttonPressTime = millis();
      buttonPressed = true;
    }
    uint32_t holdTime = millis() - buttonPressTime;
    if (holdTime > 10000) { // Factory Reset feedback (Flicker very fast)
        analogWrite(STATUS_LED, (millis() % 100 < 50) ? 255 : 0);
    } else if (holdTime > 3000) { // Config Mode feedback (Slower blink)
        analogWrite(STATUS_LED, (millis() % 400 < 200) ? 255 : 0);
        if (!portalRequested && holdTime > 4000) {
           DPRINTLN("[BUTTON] Long press detected! Starting portal...");
           portalRequested = true;
        }
    } else { // Immediate feedback for press
        analogWrite(STATUS_LED, 255);
    }
  } else if (buttonPressed) {
    uint32_t duration = millis() - buttonPressTime;
    buttonPressed = false;

    if (duration > 10000 && !portalRequested) {
      DPRINTLN("[BUTTON] Factory Reset triggered!");
      factoryReset();
    } else if (duration > 50 && duration <= 3000) {
      DPRINTLN("[BUTTON] Short press! Toggling Valve...");
      toggleValve();
      saveValveState();
      if (WiFi.status() == WL_CONNECTED && mqtt.connected()) publishTelemetry();
    }
  }

  if (portalRequested) {
    portalRequested = false;
    startConfigPortal();
  }

  // 3. Handle WiFi / MQTT Status & Binds
  updateLED();

  static wl_status_t lastWifiStatus = WL_IDLE_STATUS;
  wl_status_t currentWifiStatus = WiFi.status();
  if (currentWifiStatus != lastWifiStatus) {
    lastWifiStatus = currentWifiStatus;
    Serial.printf("[SYSTEM] WiFi Status Changed: %d\n", currentWifiStatus);
    if (currentWifiStatus == WL_CONNECTED) {
      Serial.print("[SYSTEM] WiFi Connected! IP: ");
      Serial.println(WiFi.localIP());
    } else {
      currentStatus = ST_WIFI_CONNECTING;
    }
  }

  if (currentWifiStatus == WL_CONNECTED) {
    if (!mqtt.connected()) {
      mqttConnect();
    }
    mqtt.loop();
    
    // Periodic Telemetry
    if (millis() - tTelemetry > telemetryIntervalMs) {
      tTelemetry = millis();
      publishTelemetry();
      
      // Print battery warnings locally
      float volt = getBatteryVoltage();
      if (volt < 3.5) {
        Serial.printf("[SYSTEM] WARNING: Low Battery! (%4.2f V)\n", volt);
      }
    }
  }
}
