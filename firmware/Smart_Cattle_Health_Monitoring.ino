#include <Wire.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <MPU6050.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

// WiFi + WebServer
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// Prevent I2C buffer macro redefinition warning with SparkFun header
#ifdef I2C_BUFFER_LENGTH
  #undef I2C_BUFFER_LENGTH
#endif

// SparkFun MAX30105 and Maxim SPO2 algorithm
#include "MAX30105.h"
#include "spo2_algorithm.h"   // add this file to the sketch folder

// ================== WIFI CONFIG ==================
const char* ssid     = "Aditya";
const char* password = "takeit12";

ESP8266WebServer server(80);

// ================= PIN MAPPING =================
#define DHTPIN   D5          // DHT data pin -> D5 (GPIO14)
#define DHTTYPE  DHT11       

#define ONE_WIRE_BUS D6      // DS18B20 -> D6 (GPIO12)

// GPS Pins
#define GPS_RX   D7          // ESP8266 RX  <- GPS TX
#define GPS_TX   D8          // ESP8266 TX  -> GPS RX

// I2C PINS (Wire.begin)
#define I2C_SDA D2
#define I2C_SCL D1

// ================= OBJECTS =================
DHT dht(DHTPIN, DHTTYPE);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
MPU6050 mpu;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX); // RX, TX

MAX30105 sensor;

// -------------- MAX algorithm buffers -------------
#define BUFFER_SIZE 100
uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];

// algorithm outputs (consistent names)
int32_t spo2_raw = 0;
int8_t validSPO2_raw = 0;
int32_t hr_raw = 0;
int8_t validHR_raw = 0;

// Timer for console reports
uint32_t tsLastReport = 0;

// ========== GLOBAL SENSOR STATE FOR WEB ==========
float gAmbientTemp = NAN;
float gHumidity    = NAN;
float gBodyTemp    = NAN;
float gBPM         = 0;
float gSpO2        = 0;
float gActivity    = 0;

double gLatitude   = 0;
double gLongitude  = 0;
int    gSats       = 0;
bool   gGpsValid   = false;

String gStatusDHT   = "INIT";
String gStatusBody  = "INIT";
String gStatusMAX   = "INIT";
String gStatusGPS   = "INIT";

// ===== Health summary for dashboard =====
String gHealthText  = "INIT";   // Human-readable summary
String gHealthLevel = "INIT";   // HEALTHY / WATCH / ALERT / EMERGENCY

// Callback for beat detection (compatibility)
void onBeatDetected() {
  Serial.println("Beat Detected!");
}

// ================= HR + SpO2 STABILITY FILTERS =================
float smoothBPM   = -1.0f;
float smoothSpO2  = -1.0f;

// Last good validated values
float lastGoodBPM  = -1.0f;
float lastGoodSpO2 = -1.0f;

// EMA smoothing constant (0.2–0.35 recommended)
const float ALPHA_HR = 0.30f;

// Validity ranges (tunable)
const float HR_MIN = 40.0f;
const float HR_MAX = 180.0f;
const float SPO2_MIN = 80.0f;
const float SPO2_MAX = 100.0f;

// Finger detection using IR average threshold
uint32_t IR_FINGER_THRESHOLD = 50000UL; // tune for your sensor (increase to avoid false positives)

// Sliding window parameters
const int WINDOW_SHIFT = 25;  // shift per iteration

// Helper: read one sample pair from sensor (blocking until available)
void readSample(uint32_t &red, uint32_t &ir) {
  // Wait for data available in FIFO
  while (!sensor.available()) {
    sensor.check();   // load data from FIFO
    yield();
  }
  red = sensor.getRed();
  ir  = sensor.getIR();
  sensor.nextSample();
}

// Finger presence check using average IR level across buffer
bool fingerPresentByIRAverage() {
  uint64_t sumIR = 0;
  for (int i = 0; i < BUFFER_SIZE; ++i) sumIR += irBuffer[i];
  uint32_t avgIR = (uint32_t)(sumIR / BUFFER_SIZE);
  return (avgIR > IR_FINGER_THRESHOLD);
}

// Process algorithm outputs into smoothed globals
void processAlgorithmOutputs(int32_t hr_val, int8_t validHR, int32_t spo2_val, int8_t validSpO2) {
  // Accept HR only if valid and in sensible range
  if (validHR && hr_val > HR_MIN && hr_val < HR_MAX) {
    lastGoodBPM = hr_val;
  }
  // Accept SpO2 only if valid and in range (fixed variable name here)
  if (validSpO2 && spo2_val >= SPO2_MIN && spo2_val <= SPO2_MAX) {
    lastGoodSpO2 = spo2_val;
  }

  // Smoothing (EMA)
  if (lastGoodBPM > 0) {
    if (smoothBPM < 0) smoothBPM = lastGoodBPM;
    else smoothBPM = ALPHA_HR * lastGoodBPM + (1.0f - ALPHA_HR) * smoothBPM;
  }

  if (lastGoodSpO2 > 0) {
    if (smoothSpO2 < 0) smoothSpO2 = lastGoodSpO2;
    else smoothSpO2 = ALPHA_HR * lastGoodSpO2 + (1.0f - ALPHA_HR) * smoothSpO2;
  }

  // Update public globals (0 indicates no valid reading -> UI shows ----)
  gBPM  = (smoothBPM > 0) ? smoothBPM : 0;
  gSpO2 = (smoothSpO2 > 0) ? smoothSpO2 : 0;
}

// ================== WEB PAGE (HTML) ==================
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>Cattle Health Monitor</title>
  <style>
    body { font-family: Arial, sans-serif; background: #0b1726; color: #f5f5f5; margin:0; padding:0; }
    .container { max-width:900px; margin:20px auto; padding:20px; background:#111827; border-radius:12px; box-shadow:0 0 20px rgba(0,0,0,0.5); }
    h1 { text-align:center; margin-bottom:5px; }
    h2 { margin-top:25px; border-bottom:1px solid #374151; padding-bottom:5px; color:#60a5fa; }
    .updated { text-align:center; font-size:0.85rem; color:#9ca3af; margin-bottom:15px; }
    table { width:100%; border-collapse:collapse; margin-bottom:10px; }
    th, td { padding:8px 10px; border-bottom:1px solid #1f2933; text-align:left; font-size:0.95rem; }
    th { background:#111827; color:#9ca3af; text-transform:uppercase; font-size:0.8rem; letter-spacing:0.05em; }
    .badge { display:inline-block; padding:2px 8px; border-radius:999px; font-size:0.8rem; }
    .ok { background:#065f46; color:#bbf7d0; } .err { background:#7f1d1d; color:#fecaca; } .warn { background:#92400e; color:#ffedd5; }
    .footer { text-align:center; font-size:0.8rem; color:#6b7280; margin-top:10px; }
    .value { font-weight:bold; } ul { margin-top:6px; margin-bottom:6px; padding-left:18px; font-size:0.9rem; color:#d1d5db; } li { margin-bottom:2px; }
  </style>
</head>
<body>
  <div class="container">
    <h1>Smart Cattle Health Monitor</h1>
    <div class="updated">Last updated: <span id="updated">--</span></div>

    <h2>Environment (DHT11)</h2>
    <table>
      <tr><th>Parameter</th><th>Value</th><th>Status</th></tr>
      <tr><td>Ambient Temperature</td><td class="value" id="ambientTemp">--</td><td><span id="statusDHT" class="badge warn">INIT</span></td></tr>
      <tr><td>Humidity</td><td class="value" id="humidity">--</td><td></td></tr>
    </table>

    <h2>Body Temperature (DS18B20)</h2>
    <table>
      <tr><th>Parameter</th><th>Value</th><th>Status</th></tr>
      <tr><td>Body Temperature</td><td class="value" id="bodyTemp">--</td><td><span id="statusBody" class="badge warn">INIT</span></td></tr>
    </table>

    <h2>Heart &amp; Oxygen (MAX30105)</h2>
    <table>
      <tr><th>Parameter</th><th>Value</th><th>Status</th></tr>
      <tr><td>Heart Rate</td><td class="value" id="bpm">--</td><td rowspan="2"><span id="statusMAX" class="badge warn">INIT</span></td></tr>
      <tr><td>SpO₂</td><td class="value" id="spo2">--</td></tr>
    </table>

    <h2>Activity (MPU6050)</h2>
    <table><tr><th>Parameter</th><th>Value</th></tr><tr><td>Activity</td><td class="value" id="activity">--</td></tr></table>

    <h2>Location (GPS)</h2>
    <table><tr><th>Parameter</th><th>Value</th><th>Status</th></tr>
      <tr><td>Latitude</td><td class="value" id="latitude">--</td><td rowspan="3"><span id="statusGPS" class="badge warn">INIT</span></td></tr>
      <tr><td>Longitude</td><td class="value" id="longitude">--</td></tr>
      <tr><td>Satellites</td><td class="value" id="sats">--</td></tr>
    </table>

    <h2>Health Summary</h2>
    <table><tr><th>Status</th><th>Details</th></tr>
      <tr><td><span id="statusHealth" class="badge warn">INIT</span></td><td class="value" id="healthText">--</td></tr>
    </table>

    <div class="footer">Refreshes every 2 seconds • ESP8266 Cattle Health Monitor</div>
  </div>

<script>
function setBadge(el, status) {
  el.classList.remove("ok","err","warn");
  if (status === "HEALTHY" || status === "OK") el.classList.add("ok");
  else if (status === "ALERT" || status === "EMERGENCY" || status === "ERROR") el.classList.add("err");
  else el.classList.add("warn");
  el.textContent = status;
}

function fetchData() {
  fetch('/data')
    .then(response => response.json())
    .then(data => {
      const bpmDisplay = data.bpm === "----" ? "----" : data.bpm + " bpm";
      const spo2Display = data.spo2 === "----" ? "----" : data.spo2 + " %";

      document.getElementById('ambientTemp').textContent = data.ambientTemp + " °C";
      document.getElementById('humidity').textContent    = data.humidity + " %";
      document.getElementById('bodyTemp').textContent    = data.bodyTemp + " °C";
      document.getElementById('bpm').textContent         = bpmDisplay;
      document.getElementById('spo2').textContent        = spo2Display;
      document.getElementById('activity').textContent    = data.activityText + " (" + data.activityG + " g)";
      document.getElementById('latitude').textContent    = data.latitude;
      document.getElementById('longitude').textContent   = data.longitude;
      document.getElementById('sats').textContent        = data.sats;

      document.getElementById('healthText').textContent  = data.healthText;

      setBadge(document.getElementById('statusDHT'),    data.statusDHT);
      setBadge(document.getElementById('statusBody'),   data.statusBody);
      setBadge(document.getElementById('statusMAX'),    data.statusMAX);
      setBadge(document.getElementById('statusGPS'),    data.statusGPS);
      setBadge(document.getElementById('statusHealth'), data.statusHealth);

      document.getElementById('updated').textContent = new Date().toLocaleTimeString();
    })
    .catch(err => {
      console.log(err);
      document.getElementById('updated').textContent = "Error fetching data";
    });
}

setInterval(fetchData, 2000);
fetchData();
</script>

</body>
</html>
)rawliteral";

  server.send(200, "text/html", html);
}

// JSON data endpoint
void handleData() {
  String activityText = (gActivity > 1.2) ? "Active" : "Resting";

  // Prepare safe strings
  String ambStr  = isnan(gAmbientTemp) ? "ERR" : String(gAmbientTemp, 2);
  String humStr  = isnan(gHumidity)    ? "ERR" : String(gHumidity, 2);
  String bodyStr = (gBodyTemp < -40 || gBodyTemp > 80) ? "ERR" : String(gBodyTemp, 2);

  // Show dashed '----' when no valid BPM/SpO2 (finger absent)
  String bpmStr  = (gBPM <= 0) ? "----" : String(gBPM, 1);
  String spoStr  = (gSpO2 <= 0) ? "----" : String(gSpO2, 1);

  String actGStr = String(gActivity, 2);
  String latStr  = gGpsValid ? String(gLatitude, 6)  : "NA";
  String lonStr  = gGpsValid ? String(gLongitude, 6) : "NA";
  String satStr  = gGpsValid ? String(gSats)         : "0";

  String json = "{";
  json += "\"ambientTemp\":\"" + ambStr + "\",";
  json += "\"humidity\":\""    + humStr + "\",";
  json += "\"bodyTemp\":\""    + bodyStr + "\",";
  json += "\"bpm\":\""         + bpmStr + "\",";
  json += "\"spo2\":\""        + spoStr + "\",";
  json += "\"activityText\":\"" + activityText + "\",";
  json += "\"activityG\":\""    + actGStr + "\",";
  json += "\"latitude\":\""     + latStr + "\",";
  json += "\"longitude\":\""    + lonStr + "\",";
  json += "\"sats\":\""         + satStr + "\",";
  json += "\"statusDHT\":\""    + gStatusDHT + "\",";
  json += "\"statusBody\":\""   + gStatusBody + "\",";
  json += "\"statusMAX\":\""    + gStatusMAX + "\",";
  json += "\"statusGPS\":\""    + gStatusGPS + "\",";
  json += "\"healthText\":\""   + gHealthText + "\",";
  json += "\"statusHealth\":\"" + gHealthLevel + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

// ================== SETUP ==================
void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);

  delay(500);

  dht.begin();
  sensors.begin();

  // I2C on D2 (SDA) & D1 (SCL)
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(10);

  // MPU6050
  mpu.initialize();

  // MAX30105 init
  Serial.println(F("Initializing MAX30105..."));
  if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println(F("❌ MAX30105 not found. Check wiring and power."));
    gStatusMAX = "ERROR";
  } else {
    Serial.println(F("✅ MAX30105 detected!"));
    // Configure sensor - tune these values for your hardware
    byte ledBrightness = 60;    // 0–255
    byte sampleAverage = 4;     // 1, 2, 4, 8
    byte ledMode       = 2;     // Red + IR
    byte sampleRate    = 100;   // in Hz (supported rates depend on lib)
    int  pulseWidth    = 411;   // 69, 118, 215, 411
    int  adcRange      = 16384; // 2048, 4096, 8192, 16384

    sensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
    sensor.setPulseAmplitudeRed(ledBrightness);
    sensor.setPulseAmplitudeIR(ledBrightness/2);
    gStatusMAX = "OK";
    delay(100);
  }

  // Fill initial buffer (100 samples) if sensor OK
  if (gStatusMAX != "ERROR") {
    Serial.println(F("Collecting initial MAX samples..."));
    for (int i = 0; i < BUFFER_SIZE; i++) {
      readSample(redBuffer[i], irBuffer[i]);
      delay(5); // small delay to avoid saturating FIFO; adjust if needed
    }
    // Run algorithm once on first window
    maxim_heart_rate_and_oxygen_saturation(
      irBuffer, BUFFER_SIZE,
      redBuffer,
      &spo2_raw, &validSPO2_raw,
      &hr_raw,  &validHR_raw
    );
    // Process initial outputs
    processAlgorithmOutputs(hr_raw, validHR_raw, spo2_raw, validSPO2_raw);
    Serial.println(F("Initial MAX window processed."));
  }

  // ====== WiFi ======
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connect timed out; continuing without WiFi.");
  }

  // ====== Web server routes ======
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
  Serial.println("HTTP server started");

  Serial.println("==============================================");
  Serial.println(" Smart Cattle Health Monitoring (ESP8266)");
  Serial.println(" DHT + DS18B20 + MPU6050 + MAX30105 + GPS + WEB ");
  Serial.println("==============================================");

  // Initial health state
  gHealthText  = "Waiting for stable sensor data...";
  gHealthLevel = "WATCH";
}

// ======== HEALTH LOGIC (same as earlier) ========
void computeHealthStatus(float ambientTemp, float humidity,
                         float bodyTemp, float BPM, float SpO2,
                         float activity) {
  // Default
  gHealthText  = "Analysing...";
  gHealthLevel = "WATCH";

  if (gStatusBody == "ERROR" || gStatusMAX == "ERROR") {
    gHealthText  = "Sensor error – check body temp / MAX30105.";
    gHealthLevel = "ALERT";
    return;
  }

  if (isnan(bodyTemp) || BPM <= 0 || SpO2 <= 0) {
    gHealthText  = "Waiting for stable vitals...";
    gHealthLevel = "WATCH";
    return;
  }

  bool emergency = false;
  bool alert     = false;
  bool watch     = false;

  if ((bodyTemp >= 40.5 && BPM >= 110) ||
      (SpO2 < 85 && BPM > 100) ||
      (bodyTemp <= 37.0 || BPM < 40)) {
    emergency = true;
  }

  if (!emergency) {
    if (bodyTemp >= 39.5 && bodyTemp < 40.5) alert = true;
    if (SpO2 >= 85 && SpO2 < 90) alert = true;
    if (ambientTemp > 32 && humidity > 70 && BPM > 90) alert = true;
  }

  if (!emergency && !alert) {
    if ((bodyTemp > 39.3 && bodyTemp < 39.5) || (BPM > 80 && BPM <= 95)) watch = true;
  }

  if (emergency) {
    gHealthLevel = "EMERGENCY";
    if (bodyTemp >= 40.5 && BPM >= 110) gHealthText = "Emergency: High fever and high heart rate. Immediate attention needed.";
    else if (SpO2 < 85 && BPM > 100) gHealthText = "Emergency: Very low oxygen and high heart rate.";
    else if (bodyTemp <= 37.0 || BPM < 40) gHealthText = "Emergency: Very low temperature or heart rate.";
    else gHealthText = "Emergency condition detected – check animal immediately.";
  } else if (alert) {
    gHealthLevel = "ALERT";
    if (bodyTemp >= 39.5 && BPM >= 39.5 && bodyTemp < 40.5) gHealthText = "Alert: Fever suspected. Monitor closely.";
    else if (SpO2 >= 85 && SpO2 < 90) gHealthText = "Alert: Low oxygen saturation.";
    else if (ambientTemp > 32 && humidity > 70 && BPM > 90) gHealthText = "Alert: Heat stress likely.";
    else gHealthText = "Alert: Vitals outside normal range.";
  } else if (watch) {
    gHealthLevel = "WATCH";
    gHealthText  = "Watch: Slight deviations from normal.";
  } else {
    gHealthLevel = "HEALTHY";
    gHealthText  = "Healthy: Vitals within normal range.";
  }
}

// ================== LOOP ==================
void loop() {
  server.handleClient();

  // Read GPS
  while (gpsSerial.available() > 0) gps.encode(gpsSerial.read());

  // ---------- MAX sliding window update ----------
  if (gStatusMAX != "ERROR") {
    // shift last BUFFER_SIZE - WINDOW_SHIFT samples to start
    for (int i = WINDOW_SHIFT; i < BUFFER_SIZE; i++) {
      redBuffer[i - WINDOW_SHIFT] = redBuffer[i];
      irBuffer[i - WINDOW_SHIFT]  = irBuffer[i];
    }
    // read WINDOW_SHIFT new samples
    for (int i = BUFFER_SIZE - WINDOW_SHIFT; i < BUFFER_SIZE; i++) {
      readSample(redBuffer[i], irBuffer[i]);
    }

    // Run Maxim algorithm on current window
    maxim_heart_rate_and_oxygen_saturation(
      irBuffer, BUFFER_SIZE,
      redBuffer,
      &spo2_raw, &validSPO2_raw,
      &hr_raw,  &validHR_raw
    );

    // Finger presence check (IR average)
    bool fingerNow = fingerPresentByIRAverage();

    // Decide MAX status
    if (gStatusMAX == "ERROR") {
      // keep error
    } else {
      if (!fingerNow) gStatusMAX = "WARN";
      else {
        if (validHR_raw && validSPO2_raw && (hr_raw > HR_MIN && hr_raw < HR_MAX) && (spo2_raw >= SPO2_MIN && spo2_raw <= SPO2_MAX)) {
          gStatusMAX = "OK";
        } else {
          gStatusMAX = "WARN";
        }
      }
    }

    // Process algorithm outputs into smoothed globals if finger present
    if (fingerNow) processAlgorithmOutputs(hr_raw, validHR_raw, spo2_raw, validSPO2_raw);
    else {
      // If no finger, do not update lastGood; set GUI values to 0 so UI shows ----
      gBPM = 0;
      gSpO2 = 0;
    }
  }

  // ---------------- DHT ----------------
  float humidity    = dht.readHumidity();
  float ambientTemp = dht.readTemperature();

  // ---------------- DS18B20 ----------------
  sensors.requestTemperatures();
  float bodyTemp = sensors.getTempCByIndex(0);

  // ---------------- MPU6050 ----------------
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  float activity = sqrt((float)ax * (float)ax + (float)ay * (float)ay + (float)az * (float)az) / 16384.0;

  // ---------------- GPS ----------------
  double latitude  = gps.location.lat();
  double longitude = gps.location.lng();
  bool gpsValid    = gps.location.isValid();
  int   sats       = gps.satellites.value();

  // ======= UPDATE GLOBALS FOR WEB =======
  gAmbientTemp = ambientTemp;
  gHumidity    = humidity;
  gBodyTemp    = bodyTemp;
  // gBPM and gSpO2 already updated by processing
  gActivity    = activity;
  gLatitude    = latitude;
  gLongitude   = longitude;
  gGpsValid    = gpsValid;
  gSats        = sats;

  // Status messages for DHT/Body
  gStatusDHT = (isnan(ambientTemp) || isnan(humidity)) ? "ERROR" : "OK";
  gStatusBody = (bodyTemp < -40 || bodyTemp > 80) ? "ERROR" : "OK";
  gStatusGPS = gpsValid ? "OK" : "WARN";

  // Health classification
  computeHealthStatus(ambientTemp, humidity, bodyTemp, gBPM, gSpO2, activity);

  // ================= PRINT BLOCK (Serial) =================
  if (millis() - tsLastReport > 2000) {
    tsLastReport = millis();

    Serial.println("========== CATTLE HEALTH DATA ==========");
    Serial.print("Health Status: ");
    Serial.print(gHealthLevel);
    Serial.print(" -> ");
    Serial.println(gHealthText);

    // Ambient
    if (isnan(ambientTemp) || isnan(humidity)) {
      Serial.println("Ambient Temp: ERROR (DHT)");
      Serial.println("Humidity: ERROR (DHT)");
    } else {
      Serial.printf("Ambient Temp: %.2f °C\n", ambientTemp);
      Serial.printf("Humidity: %.2f %%\n", humidity);
    }

    // Body Temp
    if (bodyTemp < -40 || bodyTemp > 80) Serial.println("Body Temp: ERROR (DS18B20)");
    else Serial.printf("Body Temp: %.2f °C\n", bodyTemp);

    // MAX - show raw algorithm outputs and smoothed reading or dashes
    Serial.printf("MAX algo raw: HR=%ld (valid=%d)  SpO2=%ld (valid=%d)\n", hr_raw, validHR_raw, spo2_raw, validSPO2_raw);

    if (gBPM <= 0 || gSpO2 <= 0) {
      Serial.println();
      Serial.print(F("HR = ----   |   SpO2 = ----"));
      Serial.println();
    } else {
      Serial.print(F("HR = "));
      Serial.print((int)round(gBPM));
      Serial.print(F("   |   SpO2 = "));
      Serial.print((int)round(gSpO2));
      Serial.println();
    }

    Serial.println(F("-----------------------------------"));

    // MPU6050
    Serial.printf("Activity: %s (%.2f g)\n", activity > 1.2 ? "Active" : "Resting", activity);

    // GPS
    if (gpsValid) {
      Serial.printf("Location: %.6f, %.6f\n", latitude, longitude);
      Serial.printf("Satellites: %d\n", sats);
    } else {
      Serial.println("GPS: Waiting for fix...");
    }

    Serial.println("========================================\n");
  }
}
