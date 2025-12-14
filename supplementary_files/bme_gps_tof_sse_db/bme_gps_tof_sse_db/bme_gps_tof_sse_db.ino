/*
  BME680, GPS, ToF sensors
  Web Server (SSE)
  HTTP POST Reporter for Python SQLite Logging

  Based on Random Nerd Tutorials and custom HTTP client for DB logging.
*/

//bme libs
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
// gps libs
#include <Adafruit_GPS.h>
#include <Adafruit_PMTK.h>
#include <NMEA_data.h>

// ToF lib
#include <vl53l4cx_class.h>

//wifi and web server libs
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

//http client lib
#include <HTTPClient.h>

// ======== WiFi Configuration ========
const char* ssid = "Adriana";
const char* password = "1234567890";

// ======== Python Server Endpoint ========
const char* serverURL = "http://172.20.10.4:5050/upload"; // <-- Update with your PC/server IP

// ======== Web Server and SSE ========
AsyncWebServer server(80);
AsyncEventSource events("/events");

// Timing for SSE event updates
unsigned long lastTime = 0;
unsigned long timerDelay = 10000;  // Send events every 10 seconds

// ======== BME688/BME680 Setup ========
#define SEALEVELPRESSURE_HPA (1007.03)
Adafruit_BME680 bme; // I2C

float temperature = 0;
float humidity = 0;
float pressure = 0;
float gas      = 0;
float altitude = 0;

// ====== GPS Setup ======
Adafruit_GPS gps(&Wire); // PA1010D on I2C (address 0x10)
float gpsLat = 0.0, gpsLon = 0.0, gpsAlt = 0.0;
bool gpsHasFix = false; // True if GPS reports a valid fix


// ============ ToF Setup ===============
  // Create sensor object
VL53L4CX sensor_vl53l4cx_sat;
TwoWire *TOF_I2C = &Wire;
#define XSHUT_PIN A1 // If you're using an XSHUT pin; set to your actual pin

  // Variable to hold ToF reading
float tofDistance = 0.0; // distance in mm
uint8_t tof_status = VL53L4CX_RANGESTATUS_NONE;


// ======== BME688 Initialization =========
void initBME() {
  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME680/BME688 sensor, check wiring!"));
    while (1);
  }
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

// ====== GPS Initialization ======
void initGPS() {
  if (!gps.begin(0x10)) { // PA1010D's I2C address is fixed at 0x10
    Serial.println(F("Could not find a valid GPS!"));
  }
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Get RMC & GGA sentences
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    // Push out new data every 1s
  gps.sendCommand(PGCMD_ANTENNA);               // Request antenna status (optional)
  delay(1000);                                  // Give GPS time to boot
  gps.println(PMTK_Q_RELEASE);                  // Occasionally log firmware version
}

// ====== ToF Initialization ======
void initToF() {
  Serial.println("Initializing VL53L4CX ToF sensor...");
 TOF_I2C ->begin();

  sensor_vl53l4cx_sat.setI2cDevice(TOF_I2C);
  sensor_vl53l4cx_sat.setXShutPin(XSHUT_PIN);

  VL53L4CX_Error error = sensor_vl53l4cx_sat.InitSensor(VL53L4CX_DEFAULT_DEVICE_ADDRESS);
  if (error != VL53L4CX_ERROR_NONE) {
    Serial.print("Error initializing VL53L4CX: ");
    Serial.println(error);
    while (1) delay(10);
  }

  sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();
  Serial.println("VL53L4CX initialized successfully!");
}



// ======== WiFi Connection =========
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.printf("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.printf("x");
    delay(1000);
  }
  Serial.printf("\nConnected! IP address: ");
  Serial.println(WiFi.localIP());
}

// ======== BME Sensor Reading Function =========
bool getSensorReadings() {
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin BME680 reading :("));
    return false;
  }

  delay(50); // Simulate parallel work
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete BME680 reading :("));
    return false;
  }

  temperature = bme.temperature;
  pressure    = bme.pressure / 100.0;
  humidity    = bme.humidity;
  gas         = bme.gas_resistance / 1000.0;
  altitude    = bme.readAltitude(SEALEVELPRESSURE_HPA);

  return true;
}

// ====== GPS Sensor Reading ======
void getGPSReadings() {
  // Recommended: Call as frequently as possible (not only on interval)
  char c = gps.read(); // Must call regularly to not lose GPS data
  if (gps.newNMEAreceived()) {
    if (!gps.parse(gps.lastNMEA())) return; // If parse fails, skip
  }
  gpsHasFix = gps.fix;
  if (gpsHasFix) {
    gpsLat = gps.latitude;
    gpsLon = gps.longitude;
    gpsAlt = gps.altitude;
  }
}

// ====== ToF Sensor Reading ======
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
    tofDistance = NAN; // no valid measurement
  }

  sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
  return true;
}



// ======== HTTP POST Function =========
void sendDataToServer() {
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

// ====== Template HTML For Web Page (BME and GPS) ======
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
    body { margin: 0;}
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
      <div class="card"><p><i class="fas fa-thermometer-half" style="color:#059e8a;"></i> TEMPERATURE</p><p><span class="reading"><span id="temp">%TEMPERATURE%</span> &deg;C</span></p></div>
      <div class="card"><p><i class="fas fa-tint" style="color:#00add6;"></i> HUMIDITY</p><p><span class="reading"><span id="hum">%HUMIDITY%</span> &percnt;</span></p></div>
      <div class="card"><p><i class="fas fa-angle-double-down" style="color:#e1e437;"></i> PRESSURE</p><p><span class="reading"><span id="pres">%PRESSURE%</span> hPa</span></p></div>
      <div class="card"><p><i class="fas fa-burn" style="color:#e37a31;"></i> GAS</p><p><span class="reading"><span id="gas">%GAS%</span> KOhms</span></p></div>
      <div class="card"><p><i class="fas fa-mountain" style="color:#777777;"></i> ALTITUDE</p><p><span class="reading"><span id="alt">%ALTITUDE%</span> m</span></p></div>
      <div class="card"><p><i class="fas fa-map-marker-alt"></i> LATITUDE</p><p><span class="reading"><span id="gps_lat">%GPSLAT%</span></span></p></div>
      <div class="card"><p><i class="fas fa-map-marker-alt"></i> LONGITUDE</p><p><span class="reading"><span id="gps_lon">%GPSLON%</span></span></p></div>
      <div class="card"><p><i class="fas fa-mountain"></i> GPS ALTITUDE</p><p><span class="reading"><span id="gps_alt">%GPSALT%</span> m</span></p></div>
      <div class="card"><p><i class="fas fa-ruler" style="color:#ff6600;"></i> TOF DISTANCE</p><p><span class="reading"><span id="tof_distance">%TOFDISTANCE%</span> mm</span></p></div>
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

 source.addEventListener('temperature', function(e) { document.getElementById("temp").innerHTML = e.data; }, false);
 source.addEventListener('humidity',    function(e) { document.getElementById("hum").innerHTML = e.data; }, false);
 source.addEventListener('pressure',    function(e) { document.getElementById("pres").innerHTML = e.data; }, false);
 source.addEventListener('gas',         function(e) { document.getElementById("gas").innerHTML = e.data; }, false);
 source.addEventListener('altitude',    function(e) { document.getElementById("alt").innerHTML = e.data; }, false);
 source.addEventListener('gps_lat',     function(e) { document.getElementById("gps_lat").innerHTML = e.data; }, false);
 source.addEventListener('gps_lon',     function(e) { document.getElementById("gps_lon").innerHTML = e.data; }, false);
 source.addEventListener('gps_alt',     function(e) { document.getElementById("gps_alt").innerHTML = e.data; }, false);
 source.addEventListener('tof_distance', function(e) { document.getElementById("tof_distance").innerHTML = e.data; 
}, false);

}
</script>
</body>
</html>
)rawliteral";

// ====== %VAR% replacement for first page load ======
String processor(const String& var) {
  if (var == "TEMPERATURE") return String(temperature);
  else if (var == "HUMIDITY") return String(humidity);
  else if (var == "PRESSURE") return String(pressure);
  else if (var == "GAS") return String(gas);
  else if (var == "ALTITUDE") return String(altitude);
  else if (var == "GPSLAT") return String(gpsLat, 6);
  else if (var == "GPSLON") return String(gpsLon, 6);
  else if (var == "GPSALT") return String(gpsAlt, 2);
  else if (var == "TOFDISTANCE") return String(tofDistance);

  return String();
}

// ====== SETUP Function ======
void setup() {
  Serial.begin(9600); 
  initWiFi();
  initBME();
  initGPS();
  initToF();

  // Web Server main page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", index_html, processor);
  });

  // SSE event handler
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);
  server.begin();
}

// ====== LOOP Function ======
void loop() {
  // Call GPS and ToF reading as frequently as possible (every loop!)
  getGPSReadings();
  getToFReading();

  // Update sensors/Send Events every timerDelay milliseconds
  if ((millis() - lastTime) > timerDelay) {
    if (getSensorReadings()) {
      Serial.printf("[BME] Temp=%.2fºC Hum=%.2f%% Press=%.2f hPa Gas=%.2f KOhms Alt=%.2f m\n",
        temperature, humidity, pressure, gas, altitude);

      // Push BME readings via SSE
      events.send(String(temperature).c_str(), "temperature", millis());
      events.send(String(humidity).c_str(), "humidity", millis());
      events.send(String(pressure).c_str(), "pressure", millis());
      events.send(String(gas).c_str(), "gas", millis());
      events.send(String(altitude).c_str(), "altitude", millis());

    }

    // Push GPS readings if we have a valid fix
    if (gpsHasFix) {
      Serial.printf("[GPS] Fix:%d Lat:%.6f Lon:%.6f Alt:%.2f\n", gpsHasFix, gpsLat, gpsLon, gpsAlt);
      events.send(String(gpsLat, 6).c_str(), "gps_lat", millis());
      events.send(String(gpsLon, 6).c_str(), "gps_lon", millis());
      events.send(String(gpsAlt, 2).c_str(), "gps_alt", millis());
    } else {
      Serial.println("[GPS] No fix yet. Waiting for satellites...");
    }

    // ToF reading
    if (getToFReading()) {
      Serial.printf("[ToF] Distance: %.2f mm\n", tofDistance);
      events.send(String(tofDistance).c_str(), "tof_distance", millis());
    }

    // Send all data to server for database logging
    sendDataToServer();

    lastTime = millis();
  }
}

