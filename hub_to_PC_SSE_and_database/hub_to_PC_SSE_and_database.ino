/*
  Combined ESP32 + BME680 + Adafruit Mini GPS PA1010D (I2C) Web Server
  Live sensor readings via Server Sent Events (SSE) to a web dashboard.
  Code references:
    - https://randomnerdtutorials.com/esp32-bme680-sensor-arduino/
    - https://github.com/adafruit/Adafruit_GPS/blob/master/examples/GPS_I2C_Parsing/GPS_I2C_Parsing.ino
    - https://randomnerdtutorials.com/esp32-web-server-sent-events-sse/
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_GPS.h>
#include <Adafruit_PMTK.h>
#include <NMEA_data.h>

// ====== WiFi Configuration ======
const char* ssid = "REPLACE_WITH_YOUR_SSID";
const char* password = "REPLACE_WITH_YOUR_PASSWORD";

// ====== Web Server and SSE ======
AsyncWebServer server(80);
AsyncEventSource events("/events");

// ====== BME688 Setup ======
#define SEALEVELPRESSURE_HPA (1007.03)
Adafruit_BME680 bme; // Uses default I2C address 

// Sensor values
float temperature = 0;
float humidity = 0;
float pressure = 0;
float gas      = 0;
float altitude = 0;

// ====== GPS Setup ======
Adafruit_GPS gps(&Wire); // PA1010D on I2C (address 0x10)
float gpsLat = 0.0, gpsLon = 0.0, gpsAlt = 0.0;
bool gpsHasFix = false; // True if GPS reports a valid fix

// Timing for SSE event updates
unsigned long lastTime = 0;
unsigned long timerDelay = 10000;  // 10 seconds between SSE updates

// ====== BME688 Initialization ======
void initBME() {
  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME680/BME688 sensor, check wiring!"));
    while (1);
  }
  // Pick oversampling and filter parameters for highest accuracy
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320°C for 150ms (factory recommended)
}

// ====== GPS Initialization ======
void initGPS() {
  if (!gps.begin(0x10)) { // PA1010D's I2C address is fixed at 0x10
    Serial.println(F("Could not find a valid GPS, check wiring!"));
    while (1);
  }
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Get RMC & GGA sentences
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    // Push out new data every 1s
  gps.sendCommand(PGCMD_ANTENNA);               // Request antenna status (optional)
  delay(1000);                                  // Give GPS time to boot
  gps.println(PMTK_Q_RELEASE);                  // Occasionally log firmware version
}

// ====== WiFi Connection ======
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.print("\nConnected! IP address: ");
  Serial.println(WiFi.localIP());
}

// ====== BME688 Sensor Reading ======
bool getSensorReadings() {
  // Start async reading
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin BME688 reading :("));
    return false;
  }
  delay(50); // Simulate "parallel" work; if needed, increase for other tasks
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete BME688 reading :("));
    return false;
  }

  // Update global variables with the sensor result
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

// ====== Template HTML For Web Page (adds GPS cards and BME cards) ======
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>ESP32 BME688 + GPS WEB SERVER (SSE)</title>
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
    <h1>BME688 + GPS WEB SERVER (SSE)</h1>
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
  return String();
}

// ====== SETUP Function ======
void setup() {
  Serial.begin(115200);
  initWiFi();
  initBME();
  initGPS();

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
  // Call GPS reading as frequently as possible (every loop!)
  getGPSReadings();

  // Update BME/Send Events every timerDelay milliseconds
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

    lastTime = millis();
  }
}
