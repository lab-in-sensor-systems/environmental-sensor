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

// ======== WiFi Configuration ========
const char* ssid     = "Adriana";
const char* password = "1234567890";

// ======== Python Server Endpoint (your laptop) ========
const char* serverURL = "http://172.20.10.4:5050/upload";  // adjust if your PC IP changes

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
#define XSHUT_PIN A1  // adjust to your actual XSHUT pin

float tofDistance = NAN; // mm
uint8_t tof_status = VL53L4CX_RANGESTATUS_NONE;

// Timing
unsigned long lastTime   = 0;
unsigned long timerDelay = 2000;  // every 2 seconds

// ======== OLED Setup (SSD1306) ========
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDR     0x3D       // from your I2C scanner

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Forward declarations
void updateDisplay();
void sendDataToServer();

// ======== OLED Initialization =========
void initDisplay() {
  Serial.println(F("Initialising OLED..."));

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1306 OLED init failed. Check address and wiring."));
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

// ======== BME688 Initialization =========
void initBME() {
  Serial.println(F("Initialising BME688..."));

  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME680/BME688 sensor, check wiring!"));
    bmeReady = false;

    if (oledReady) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println(F("BME688 Monitor"));
      display.println(F("OLED OK"));
      display.println(F("BME FAIL"));
      display.display();
    }
    return;
  }

  // BME configuration
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320°C for 150 ms

  bmeReady = true;
  Serial.println(F("BME688 initialised OK."));

  if (oledReady) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("BME688 Monitor"));
    display.println(F("OLED OK"));
    display.println(F("BME OK"));
    display.display();
  }
}

// ======== GPS Initialization =========
void initGPS() {
  Serial.println(F("Initialising GPS..."));
  if (!gps.begin(0x10)) { // PA1010D fixed I2C address
    Serial.println(F("Could not find a valid GPS!"));
    return;
  }
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // RMC + GGA sentences
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    // 1 Hz updates
  gps.sendCommand(PGCMD_ANTENNA);               // optional antenna info
  delay(1000);
  gps.println(PMTK_Q_RELEASE);
  Serial.println(F("GPS initialised."));
}

// ======== ToF Initialization =========
void initToF() {
  Serial.println(F("Initialising VL53L4CX ToF..."));
  TOF_I2C->begin();

  sensor_vl53l4cx_sat.setI2cDevice(TOF_I2C);
  sensor_vl53l4cx_sat.setXShutPin(XSHUT_PIN);

  VL53L4CX_Error error = sensor_vl53l4cx_sat.InitSensor(VL53L4CX_DEFAULT_DEVICE_ADDRESS);
  if (error != VL53L4CX_ERROR_NONE) {
    Serial.print(F("Error initialising VL53L4CX: "));
    Serial.println(error);
    return;
  }

  sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();
  Serial.println(F("VL53L4CX initialised successfully!"));
}

// ======== WiFi Connection =========
void initWiFi() {
  Serial.println(F("Connecting to WiFi..."));
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());
  wifiReady = true;

  if (oledReady) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("BME688 Monitor"));
    display.println(F("OLED OK"));
    display.println(F("BME OK"));
    display.println(F("WiFi OK:"));
    display.println(WiFi.localIP().toString());
    display.display();
  }
}

// ======== Update OLED with current sensor values =========
void updateDisplay() {
  if (!oledReady) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  if (!bmeReady) {
    display.println(F("BME688 not ready"));
    display.display();
    return;
  }

  display.println(F("BME688 Monitor"));

  display.print(F("T: "));
  display.print(temperature, 1);
  display.println(F(" C"));

  display.print(F("H: "));
  display.print(humidity, 1);
  display.println(F(" %"));

  display.print(F("P: "));
  display.print(pressure, 1);
  display.println(F(" hPa"));

  display.print(F("Gas: "));
  display.print(gas, 2);
  display.println(F(" kOhm"));

  display.print(F("Alt: "));
  display.print(altitude, 1);
  display.println(F(" m"));

  display.print(F("ToF: "));
  if (isnan(tofDistance)) {
    display.println(F("--"));
  } else {
    display.print(tofDistance, 0);
    display.println(F(" mm"));
  }

  display.display();
}

// ======== BME Sensor Reading Function (simple) =========
bool getSensorReadings() {
  if (!bmeReady) {
    Serial.println(F("getSensorReadings() called but BME not ready."));
    return false;
  }

  Serial.println(F("Taking BME688 reading..."));

  if (!bme.performReading()) {
    Serial.println(F("Failed to perform BME688 reading :("));
    return false;
  }

  temperature = bme.temperature;
  pressure    = bme.pressure / 100.0;
  humidity    = bme.humidity;
  gas         = bme.gas_resistance / 1000.0;
  altitude    = bme.readAltitude(SEALEVELPRESSURE_HPA);

  Serial.printf("T = %.2f C, H = %.2f %%, P = %.2f hPa, Gas = %.2f kOhm, Alt = %.2f m\n",
                temperature, humidity, pressure, gas, altitude);

  updateDisplay();  // refresh OLED with new values

  return true;
}

// ======== GPS Reading Function =========
void getGPSReadings() {
  char c = gps.read();
  if (gps.newNMEAreceived()) {
    if (!gps.parse(gps.lastNMEA())) return;
  }

  gpsHasFix = gps.fix;
  if (gpsHasFix) {
    gpsLat = gps.latitude;
    gpsLon = gps.longitude;
    gpsAlt = gps.altitude;
  }
}

// ======== ToF Reading Function =========
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
  } else {
    tofDistance = NAN;
  }

  sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
  return true;
}

// ======== HTTP POST Function (send data to Python server) =========
void sendDataToServer() {
  if (!wifiReady) {
    Serial.println("WiFi not ready, cannot send data.");
    return;
  }

  if (WiFi.status() == WL_CONNECTED) {
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
                     "&gps_fix=" + String(gpsHasFix ? 1 : 0) +
                     "&tof_distance=" + String(tofDistance);

    int httpResponseCode = http.POST(payload);
    Serial.print("HTTP POST Response code: ");
    Serial.println(httpResponseCode);
    http.end();
  } else {
    Serial.println("WiFi not connected, cannot send data.");
  }
}

// ======== Template HTML (BME + GPS + ToF) =========
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>ESP32 BME688 + GPS + ToF WEB SERVER (SSE)</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="stylesheet" href="https://use.fontawesome.com/releases/v5.7.2/css/all.css">
  <link rel="icon" href="data:,">
  <style>
    html {font-family: Arial; display: inline-block; text-align: center;}
    p { font-size: 1.2rem;}
    body {  margin: 0;}
    .topnav { overflow: hidden; background-color: #50B8B4; color: white; font-size: 1rem; }
    .content { padding: 20px; }
    .card { background-color: white; box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5); }
    .cards { max-width: 900px; margin: 0 auto; display: grid; grid-gap: 2rem; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); }
    .reading { font-size: 1.4rem; }
  </style>
</head>
<body>
  <div class="topnav">
    <h1>BME688 + GPS + ToF WEB SERVER (SSE)</h1>
  </div>
  <div class="content">
    <div class="cards">
      <div class="card">
        <p><i class="fas fa-thermometer-half" style="color:#059e8a;"></i> TEMPERATURE</p>
        <p><span class="reading"><span id="temp">%TEMPERATURE%</span> &deg;C</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-tint" style="color:#00add6;"></i> HUMIDITY</p>
        <p><span class="reading"><span id="hum">%HUMIDITY%</span> &percnt;</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-angle-double-down" style="color:#e1e437;"></i> PRESSURE</p>
        <p><span class="reading"><span id="pres">%PRESSURE%</span> hPa</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-burn" style="color:#e37a31;"></i> GAS</p>
        <p><span class="reading"><span id="gas">%GAS%</span> KOhms</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-mountain" style="color:#777777;"></i> ALTITUDE</p>
        <p><span class="reading"><span id="alt">%ALTITUDE%</span> m</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-map-marker-alt"></i> LATITUDE</p>
        <p><span class="reading"><span id="gps_lat">%GPSLAT%</span></span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-map-marker-alt"></i> LONGITUDE</p>
        <p><span class="reading"><span id="gps_lon">%GPSLON%</span></span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-mountain"></i> GPS ALTITUDE</p>
        <p><span class="reading"><span id="gps_alt">%GPSALT%</span> m</span></p>
      </div>
      <div class="card">
        <p><i class="fas fa-ruler" style="color:#ff6600;"></i> TOF DISTANCE</p>
        <p><span class="reading"><span id="tof_distance">%TOFDISTANCE%</span> mm</span></p>
      </div>
    </div>
  </div>
<script>
if (!!window.EventSource) {
 var source = new EventSource('/events');
 
 source.addEventListener('open', function(e) {
  console.log("Events Connected");
 }, false);
 source.addEventListener('error', function(e) {
  if (e.target.readyState != EventSource.OPEN) {
    console.log("Events Disconnected");
  }
 }, false);
 
 source.addEventListener('temperature', function(e) {
  document.getElementById("temp").innerHTML = e.data;
 }, false);
 
 source.addEventListener('humidity', function(e) {
  document.getElementById("hum").innerHTML = e.data;
 }, false);
 
 source.addEventListener('pressure', function(e) {
  document.getElementById("pres").innerHTML = e.data;
 }, false);

 source.addEventListener('gas', function(e) {
  document.getElementById("gas").innerHTML = e.data;
 }, false);

 source.addEventListener('altitude', function(e) {
  document.getElementById("alt").innerHTML = e.data;
 }, false);

 source.addEventListener('gps_lat', function(e) {
  document.getElementById("gps_lat").innerHTML = e.data;
 }, false);

 source.addEventListener('gps_lon', function(e) {
  document.getElementById("gps_lon").innerHTML = e.data;
 }, false);

 source.addEventListener('gps_alt', function(e) {
  document.getElementById("gps_alt").innerHTML = e.data;
 }, false);

 source.addEventListener('tof_distance', function(e) {
  document.getElementById("tof_distance").innerHTML = e.data;
 }, false);
}
</script>
</body>
</html>)rawliteral";

// %VAR% replacement for first page load
String processor(const String& var) {
  if      (var == "TEMPERATURE")   return String(temperature);
  else if (var == "HUMIDITY")      return String(humidity);
  else if (var == "PRESSURE")      return String(pressure);
  else if (var == "GAS")           return String(gas);
  else if (var == "ALTITUDE")      return String(altitude);
  else if (var == "GPSLAT")        return String(gpsLat, 6);
  else if (var == "GPSLON")        return String(gpsLon, 6);
  else if (var == "GPSALT")        return String(gpsAlt, 2);
  else if (var == "TOFDISTANCE")   return String(tofDistance);
  return String();
}

// ======== SETUP =========
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("Starting up..."));

  Wire.begin();         // shared I2C for BME, GPS, ToF
  initDisplay();        // OLED
  initBME();            // BME688
  initGPS();            // GPS
  initToF();            // ToF
  initWiFi();           // WiFi and IP

  // Web Server main page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  // Server-Sent Events handler
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    client->send("hello!", NULL, millis(), 10000);
  });

  server.addHandler(&events);
  server.begin();

  Serial.println(F("Setup complete."));
}

// ======== LOOP =========
void loop() {
  // Keep GPS and ToF updated as often as possible
  getGPSReadings();
  getToFReading();

  if ((millis() - lastTime) > timerDelay) {
    // BME readings
    if (getSensorReadings()) {
      events.send(String(temperature).c_str(), "temperature", millis());
      events.send(String(humidity).c_str(),    "humidity",    millis());
      events.send(String(pressure).c_str(),    "pressure",    millis());
      events.send(String(gas).c_str(),         "gas",         millis());
      events.send(String(altitude).c_str(),    "altitude",    millis());
    }

    // GPS readings (only if fix)
    if (gpsHasFix) {
      Serial.printf("[GPS] Fix:%d Lat:%.6f Lon:%.6f Alt:%.2f\n",
                    gpsHasFix, gpsLat, gpsLon, gpsAlt);
      events.send(String(gpsLat, 6).c_str(), "gps_lat", millis());
      events.send(String(gpsLon, 6).c_str(), "gps_lon", millis());
      events.send(String(gpsAlt, 2).c_str(), "gps_alt", millis());
    } else {
      Serial.println("[GPS] No fix yet. Waiting for satellites...");
    }

    // ToF (if last reading was valid)
    if (!isnan(tofDistance)) {
      Serial.printf("[ToF] Distance: %.2f mm\n", tofDistance);
      events.send(String(tofDistance).c_str(), "tof_distance", millis());
    } else {
      Serial.println("[ToF] No valid measurement.");
    }

    // Send everything to your Python/SQLite server
    sendDataToServer();

    lastTime = millis();
  }
}
