/*
Code edited from https://randomnerdtutorials.com/esp32-bme680-sensor-arduino/
and https://randomnerdtutorials.com/esp32-web-server-sent-events-sse/ 
*/

#include <Wire.h>  
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// ======== WiFi Configuration ========
const char* ssid = "Wenjun's iPhone";
const char* password = "Wenjun2004";

// ======== Web Server and SSE ========
AsyncWebServer server(80);
AsyncEventSource events("/events");

// ======== BME688 Setup ========
#define SEALEVELPRESSURE_HPA (1007.03)
Adafruit_BME680 bme; // I2C

float temperature = 0;
float humidity = 0;
float pressure = 0;
float gas      = 0;
float altitude = 0;

// Timing
unsigned long lastTime = 0;
unsigned long timerDelay = 10000;  // Send events every 10 seconds

bool _wifiHadIP = false;

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

// ======== WiFi Connection =========
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

// ======== Sensor Reading Function =========
bool getSensorReadings() {
  // Start async reading
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin BME688 reading :("));
    return false;
  }

  delay(50); // Simulate parallel work, optional
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete BME688 reading :("));
    return false;
  }

  temperature = bme.temperature;
  pressure    = bme.pressure / 100.0;
  humidity    = bme.humidity;
  gas         = bme.gas_resistance / 1000.0;
  altitude    = bme.readAltitude(SEALEVELPRESSURE_HPA);

  return true;
}

// ======== Template HTML (from your example) =========
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>ESP32 BME688 WEB SERVER (SSE)</title>
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
    .cards { max-width: 800px; margin: 0 auto; display: grid; grid-gap: 2rem; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); }
    .reading { font-size: 1.4rem; }
  </style>
</head>
<body>
  <div class="topnav">
    <h1>BME688 WEB SERVER (SSE)</h1>
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
}
</script>
</body>
</html>)rawliteral";

// %VAR% replacement for first page load
String processor(const String& var) {
  if(var == "TEMPERATURE") return String(temperature);
  else if(var == "HUMIDITY") return String(humidity);
  else if(var == "PRESSURE") return String(pressure);
  else if(var == "GAS") return String(gas);
  else if(var == "ALTITUDE") return String(altitude);
  return String();
}

void wifiStatusTask() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!_wifiHadIP) {
      _wifiHadIP = true;
      if (Serial) { Serial.print("[ESP-32] Connected. IP: "); Serial.println(WiFi.localIP()); }
    }
    return;
  }

// ======== SETUP =========
void setup() {
  Serial.begin(115200);
  initWiFi();
  initBME();

  // Web Server main page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", index_html, processor);
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
}

// ======== LOOP =========
void loop() {
  if ((millis() - lastTime) > timerDelay) {
    if(getSensorReadings()) {
      Serial.printf("Temperature = %.2f ºC\n", temperature);
      Serial.printf("Humidity = %.2f %%\n", humidity);
      Serial.printf("Pressure = %.2f hPa\n", pressure);
      Serial.printf("Gas = %.2f KOhms\n", gas);
      Serial.printf("Altitude = %.2f m\n\n", altitude);

      // Send live updates via SSE
      events.send(String(temperature).c_str(), "temperature", millis());
      events.send(String(humidity).c_str(), "humidity", millis());
      events.send(String(pressure).c_str(), "pressure", millis());
      events.send(String(gas).c_str(), "gas", millis());
      events.send(String(altitude).c_str(), "altitude", millis());
    }
    lastTime = millis();
  }
}
