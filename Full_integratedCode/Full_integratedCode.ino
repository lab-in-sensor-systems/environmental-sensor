#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// ======== OLED Includes (SSD1306) ========
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ======== GPS Includes ========
#include <Adafruit_GPS.h>
#include <Adafruit_PMTK.h>
#include <NMEA_data.h>

// ======== ToF Includes ========
#include <vl53l4cx_class.h>

// ======== HTTP Client ========
#include <HTTPClient.h>

// ======== BLE Client Includes ========
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>

// ======== WiFi Configuration ========
const char* ssid     = "Adriana";
const char* password = "1234567890";

// ======== Python Server Endpoint ========
const char* serverURL = "http://172.20.10.4:5050/upload";

// ======== Web Server and SSE ========
AsyncWebServer server(80);
AsyncEventSource events("/events");

// ======== BME688 Setup ========
#define SEALEVELPRESSURE_HPA (1007.03)
Adafruit_BME680 bme; // I2C

float temperature = 0;
float humidity    = 0;
float pressure    = 0;
float gas         = 0;
float altitude    = 0;

bool bmeReady  = false;
bool oledReady = false;
bool wifiReady = false;

// ======== GPS Setup ========
Adafruit_GPS gps(&Wire); // PA1010D I2C (0x10)
float gpsLat = 0.0, gpsLon = 0.0, gpsAlt = 0.0;
bool gpsHasFix = false;

// ======== ToF Setup (VL53L4CX) ========
VL53L4CX sensor_vl53l4cx_sat;
TwoWire *TOF_I2C = &Wire;
#define XSHUT_PIN A1

float tofDistance = NAN; // mm
uint8_t tof_status = VL53L4CX_RANGESTATUS_NONE;

// Timing
unsigned long lastTime   = 0;
unsigned long timerDelay = 2000;

// ======== OLED Setup (SSD1306) ========
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDR     0x3D

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ======== BLE Client Setup ========
// PPG server UUIDs
#define PPG_SERVICE_UUID        "d3aa6a66-a623-4c76-80a5-a66004acf0bf"
#define PPG_CHARACTERISTIC_UUID "2df03c7d-8ac5-493e-9d88-ac467aed4ad0"

// ACCEL server UUIDs
#define ACCEL_SERVICE_UUID        "e2f4d9b8-9992-474d-a3b1-db7cea750f93"
#define ACCEL_CHARACTERISTIC_UUID "f446fe54-ab3a-4e86-ab0c-22f42e17ea83"

// PPG client
BLEClient*  ppgClient        = nullptr;
BLERemoteCharacteristic* ppgRemoteCharacteristic = nullptr;
bool ppgConnected = false;

// ACCEL client
BLEClient*  accelClient        = nullptr;
BLERemoteCharacteristic* accelRemoteCharacteristic = nullptr;
bool accelConnected = false;

// HR + SpO2 readings
uint8_t heartRate = 0;
uint8_t hrValid   = 0;
uint8_t spo2      = 0;
uint8_t spo2Valid = 0;

// ACCEL readings (mg and °C)
int16_t accelX = 0;
int16_t accelY = 0;
int16_t accelZ = 0;
int8_t  accelTempC = 0;
bool    accelHasData = false;

// Forward declarations
void updateDisplay();
void sendDataToServer();
bool connectToPPGServer();
bool connectToAccelServer();

// ======== BLE Notification Callbacks ========
// PPG
static void ppgNotifyCallback(
  BLERemoteCharacteristic*,
  uint8_t* pData,
  size_t length,
  bool
) {
  if (length >= 4) {
    heartRate = pData[0];
    hrValid   = pData[1];
    spo2      = pData[2];
    spo2Valid = pData[3];
    Serial.printf("[PPG] HR=%d valid=%d | SpO2=%d valid=%d\n",
                  heartRate, hrValid, spo2, spo2Valid);
  }
}

// ACCEL (expects 7 bytes: xL,xH,yL,yH,zL,zH,temp)
static void accelNotifyCallback(
  BLERemoteCharacteristic*,
  uint8_t* pData,
  size_t length,
  bool
) {
  if (length >= 7) {
    accelX = (int16_t)(pData[0] | (pData[1] << 8));
    accelY = (int16_t)(pData[2] | (pData[3] << 8));
    accelZ = (int16_t)(pData[4] | (pData[5] << 8));
    accelTempC = (int8_t)pData[6];
    accelHasData = true;
    Serial.printf("[ACCEL] x=%dmg y=%dmg z=%dmg T=%dC\n",
                  accelX, accelY, accelZ, accelTempC);
  }
}

// ======== BLE Connect Functions ========

bool connectToPPGServer() {
  Serial.println("Scanning for PPG server...");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  BLEScanResults* foundDevices = pBLEScan->start(5);

  for (int i = 0; i < foundDevices->getCount(); i++) {
    BLEAdvertisedDevice device = foundDevices->getDevice(i);
    if (device.haveServiceUUID() &&
        device.isAdvertisingService(BLEUUID(PPG_SERVICE_UUID))) {
      Serial.println("Found PPG sensor node");
      ppgClient = BLEDevice::createClient();
      if (!ppgClient->connect(&device)) {
        Serial.println("PPG connect failed");
        return false;
      }
      BLERemoteService* pRemoteService =
        ppgClient->getService(BLEUUID(PPG_SERVICE_UUID));
      if (pRemoteService == nullptr) {
        Serial.println("PPG service not found");
        ppgClient->disconnect();
        return false;
      }
      ppgRemoteCharacteristic =
        pRemoteService->getCharacteristic(BLEUUID(PPG_CHARACTERISTIC_UUID));
      if (ppgRemoteCharacteristic == nullptr) {
        Serial.println("PPG characteristic not found");
        ppgClient->disconnect();
        return false;
      }
      if (ppgRemoteCharacteristic->canNotify()) {
        ppgRemoteCharacteristic->registerForNotify(ppgNotifyCallback);
      }
      ppgConnected = true;
      Serial.println("PPG connected and notifications enabled");
      return true;
    }
  }
  Serial.println("PPG server not found");
  return false;
}

bool connectToAccelServer() {
  Serial.println("Scanning for ACCEL server...");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  BLEScanResults* foundDevices = pBLEScan->start(5);

  for (int i = 0; i < foundDevices->getCount(); i++) {
    BLEAdvertisedDevice device = foundDevices->getDevice(i);
    if (device.haveServiceUUID() &&
        device.isAdvertisingService(BLEUUID(ACCEL_SERVICE_UUID))) {
      Serial.println("Found ACCEL sensor node");
      accelClient = BLEDevice::createClient();
      if (!accelClient->connect(&device)) {
        Serial.println("ACCEL connect failed");
        return false;
      }
      BLERemoteService* pRemoteService =
        accelClient->getService(BLEUUID(ACCEL_SERVICE_UUID));
      if (pRemoteService == nullptr) {
        Serial.println("ACCEL service not found");
        accelClient->disconnect();
        return false;
      }
      accelRemoteCharacteristic =
        pRemoteService->getCharacteristic(BLEUUID(ACCEL_CHARACTERISTIC_UUID));
      if (accelRemoteCharacteristic == nullptr) {
        Serial.println("ACCEL characteristic not found");
        accelClient->disconnect();
        return false;
      }
      if (accelRemoteCharacteristic->canNotify()) {
        accelRemoteCharacteristic->registerForNotify(accelNotifyCallback);
      }
      accelConnected = true;
      Serial.println("ACCEL connected and notifications enabled");
      return true;
    }
  }
  Serial.println("ACCEL server not found");
  return false;
}

// ======== Display, BME, GPS, ToF, WiFi Initialization =========
void initDisplay() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    oledReady = false;
    return;
  }
  oledReady = true;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("BME688 Monitor"));
  display.println(F("OLED OK"));
  display.display();
}

void initBME() {
  if (!bme.begin()) { bmeReady = false; return; }
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);
  bmeReady = true;
}

void initGPS() {
  if (!gps.begin(0x10)) return;
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  gps.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  gps.println(PMTK_Q_RELEASE);
}

void initToF() {
  TOF_I2C->begin();
  sensor_vl53l4cx_sat.setI2cDevice(TOF_I2C);
  sensor_vl53l4cx_sat.setXShutPin(XSHUT_PIN);
  VL53L4CX_Error error = sensor_vl53l4cx_sat.InitSensor(VL53L4CX_DEFAULT_DEVICE_ADDRESS);
  if (error) return;
  sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(1000);
  wifiReady = true;
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// ======== Update OLED Display =========
void updateDisplay() {
  if (!oledReady) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("BME688 Monitor"));

  display.printf("T: %.1f C\nH: %.1f %%\nP: %.1f hPa\nGas: %.2f kOhm\nAlt: %.1f m\n",
                 temperature, humidity, pressure, gas, altitude);

  display.printf("HR: %d %s\nSpO2: %d %s\n",
                 heartRate, hrValid?"OK":"NA",
                 spo2,     spo2Valid?"OK":"NA");

  if (accelHasData) {
    display.printf("Ax:%d Ay:%d Az:%d\nT:%dC\n",
                   accelX, accelY, accelZ, accelTempC);
  }

  if (!isnan(tofDistance)) display.printf("ToF: %.0f mm\n", tofDistance);

  display.display();
}

// ======== Read Sensors =========
bool getSensorReadings() {
  if (!bmeReady) return false;
  if (!bme.performReading()) return false;

  temperature = bme.temperature;
  pressure    = bme.pressure / 100.0;
  humidity    = bme.humidity;
  gas         = bme.gas_resistance / 1000.0;
  altitude    = bme.readAltitude(SEALEVELPRESSURE_HPA);

  updateDisplay();
  return true;
}

void getGPSReadings() {
  char c = gps.read();
  (void)c;
  if (gps.newNMEAreceived()) if (!gps.parse(gps.lastNMEA())) return;
  gpsHasFix = gps.fix;
  if (gpsHasFix) {
    gpsLat = gps.latitude;
    gpsLon = gps.longitude;
    gpsAlt = gps.altitude;
  }
}

bool getToFReading() {
  uint8_t NewDataReady = 0;
  VL53L4CX_MultiRangingData_t data;
  VL53L4CX_MultiRangingData_t *pData = &data;

  int status = sensor_vl53l4cx_sat.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
  if (status || !NewDataReady) return false;

  status = sensor_vl53l4cx_sat.VL53L4CX_GetMultiRangingData(pData);
  if (status) return false;

  if (pData->NumberOfObjectsFound > 0 &&
      (pData->RangeData[0].RangeStatus == VL53L4CX_RANGESTATUS_RANGE_VALID ||
       pData->RangeData[0].RangeStatus == VL53L4CX_RANGESTATUS_RANGE_VALID_MERGED_PULSE)) {
    tofDistance = pData->RangeData[0].RangeMilliMeter;
  } else tofDistance = NAN;

  sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
  return true;
}

// ======== HTTP POST =========
void sendDataToServer() {
  if (!wifiReady || WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(serverURL);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  String payload = "temperature=" + String(temperature) +
                   "&humidity=" + String(humidity) +
                   "&pressure=" + String(pressure) +
                   "&gas=" + String(gas) +
                   "&altitude=" + String(altitude) +
                   "&gps_lat=" + String(gpsLat, 6) +
                   "&gps_lon=" + String(gpsLon, 6) +
                   "&gps_alt=" + String(gpsAlt, 2) +
                   "&gps_fix=" + String(gpsHasFix?1:0) +
                   "&tof_distance=" + String(tofDistance) +
                   "&hr=" + String(heartRate) +
                   "&hr_valid=" + String(hrValid) +
                   "&spo2=" + String(spo2) +
                   "&spo2_valid=" + String(spo2Valid) +
                   "&accel_x=" + String(accelX) +
                   "&accel_y=" + String(accelY) +
                   "&accel_z=" + String(accelZ) +
                   "&accel_temp=" + String(accelTempC);

  int httpResponseCode = http.POST(payload);
  Serial.print("HTTP POST Response code: ");
  Serial.println(httpResponseCode);
  http.end();
}

// ======== HTML + SSE =========
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>ESP32 HUB</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; text-align: center; margin: 0; }
    .cards { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); grid-gap: 15px; padding: 20px; }
    .card { background-color: #fff; box-shadow: 2px 2px 12px rgba(0,0,0,0.2); padding: 15px; border-radius: 10px; }
    .reading { font-size: 1.5rem; }
  </style>
</head>
<body>
  <h1>ESP32 HUB Sensors</h1>
  <div class="cards">
    <div class="card"><p>Temperature</p><p class="reading" id="temp">%TEMPERATURE%</p></div>
    <div class="card"><p>Humidity</p><p class="reading" id="hum">%HUMIDITY%</p></div>
    <div class="card"><p>Pressure</p><p class="reading" id="pres">%PRESSURE%</p></div>
    <div class="card"><p>Gas</p><p class="reading" id="gas">%GAS%</p></div>
    <div class="card"><p>Altitude</p><p class="reading" id="alt">%ALTITUDE%</p></div>
    <div class="card"><p>Latitude</p><p class="reading" id="gps_lat">%GPSLAT%</p></div>
    <div class="card"><p>Longitude</p><p class="reading" id="gps_lon">%GPSLON%</p></div>
    <div class="card"><p>GPS Altitude</p><p class="reading" id="gps_alt">%GPSALT%</p></div>
    <div class="card"><p>ToF Distance</p><p class="reading" id="tof_distance">%TOFDISTANCE%</p></div>
    <div class="card"><p>Heart Rate</p><p class="reading" id="hr">%HR%</p></div>
    <div class="card"><p>SpO2</p><p class="reading" id="spo2">%SPO2%</p></div>
    <div class="card"><p>Accel X</p><p class="reading" id="accel_x">%ACCELX%</p></div>
    <div class="card"><p>Accel Y</p><p class="reading" id="accel_y">%ACCELY%</p></div>
    <div class="card"><p>Accel Z</p><p class="reading" id="accel_z">%ACCELZ%</p></div>
    <div class="card"><p>Accel Temp</p><p class="reading" id="accel_temp">%ACCELTEMP%</p></div>
  </div>

<script>
if (!!window.EventSource) {
  var source = new EventSource('/events');

  source.addEventListener('temperature', e => document.getElementById('temp').innerHTML = e.data);
  source.addEventListener('humidity',    e => document.getElementById('hum').innerHTML = e.data);
  source.addEventListener('pressure',    e => document.getElementById('pres').innerHTML = e.data);
  source.addEventListener('gas',         e => document.getElementById('gas').innerHTML = e.data);
  source.addEventListener('altitude',    e => document.getElementById('alt').innerHTML = e.data);
  source.addEventListener('gps_lat',     e => document.getElementById('gps_lat').innerHTML = e.data);
  source.addEventListener('gps_lon',     e => document.getElementById('gps_lon').innerHTML = e.data);
  source.addEventListener('gps_alt',     e => document.getElementById('gps_alt').innerHTML = e.data);
  source.addEventListener('tof_distance',e => document.getElementById('tof_distance').innerHTML = e.data);
  source.addEventListener('hr',          e => document.getElementById('hr').innerHTML = e.data);
  source.addEventListener('spo2',        e => document.getElementById('spo2').innerHTML = e.data);
  source.addEventListener('accel_x',     e => document.getElementById('accel_x').innerHTML = e.data);
  source.addEventListener('accel_y',     e => document.getElementById('accel_y').innerHTML = e.data);
  source.addEventListener('accel_z',     e => document.getElementById('accel_z').innerHTML = e.data);
  source.addEventListener('accel_temp',  e => document.getElementById('accel_temp').innerHTML = e.data);
}
</script>
</body>
</html>)rawliteral";

// ======== HTML Variable Replacement =========
String processor(const String& var) {
  if      (var == "TEMPERATURE") return String(temperature);
  else if (var == "HUMIDITY")    return String(humidity);
  else if (var == "PRESSURE")    return String(pressure);
  else if (var == "GAS")         return String(gas);
  else if (var == "ALTITUDE")    return String(altitude);
  else if (var == "GPSLAT")      return String(gpsLat, 6);
  else if (var == "GPSLON")      return String(gpsLon, 6);
  else if (var == "GPSALT")      return String(gpsAlt, 2);
  else if (var == "TOFDISTANCE") return String(tofDistance);
  else if (var == "HR")          return String(heartRate);
  else if (var == "SPO2")        return String(spo2);
  else if (var == "ACCELX")      return String(accelX);
  else if (var == "ACCELY")      return String(accelY);
  else if (var == "ACCELZ")      return String(accelZ);
  else if (var == "ACCELTEMP")   return String(accelTempC);
  return String();
}

// ======== SETUP =========
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();
  initDisplay();
  initBME();
  initGPS();
  initToF();
  initWiFi();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()) Serial.printf("Client reconnected! Last ID: %u\n", client->lastId());
  });
  server.addHandler(&events);
  server.begin();

  BLEDevice::init("hub");
  connectToPPGServer();
  connectToAccelServer();
}

// ======== LOOP =========
void loop() {
  getGPSReadings();
  getToFReading();

  if ((millis() - lastTime) > timerDelay) {
    if (getSensorReadings()) {
      events.send(String(temperature).c_str(), "temperature", millis());
      events.send(String(humidity).c_str(),    "humidity",    millis());
      events.send(String(pressure).c_str(),    "pressure",    millis());
      events.send(String(gas).c_str(),         "gas",         millis());
      events.send(String(altitude).c_str(),    "altitude",    millis());
      events.send(String(heartRate).c_str(),   "hr",          millis());
      events.send(String(spo2).c_str(),        "spo2",        millis());
      if (accelHasData) {
        events.send(String(accelX).c_str(),     "accel_x",    millis());
        events.send(String(accelY).c_str(),     "accel_y",    millis());
        events.send(String(accelZ).c_str(),     "accel_z",    millis());
        events.send(String(accelTempC).c_str(), "accel_temp", millis());
      }
    }

    if (gpsHasFix) {
      events.send(String(gpsLat, 6).c_str(), "gps_lat", millis());
      events.send(String(gpsLon, 6).c_str(), "gps_lon", millis());
      events.send(String(gpsAlt, 2).c_str(), "gps_alt", millis());
    }

    if (!isnan(tofDistance))
      events.send(String(tofDistance).c_str(), "tof_distance", millis());

    sendDataToServer();
    lastTime = millis();
  }

  // Simple reconnect logic
  if (!ppgConnected)   connectToPPGServer();
  if (!accelConnected) connectToAccelServer();
}
