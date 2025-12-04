#include <Arduino.h>
#include <PulseSensorPlayground.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// =========================
// BLE UUIDs
// =========================
#define SERVICE_UUID        "d3aa6a66-a623-4c76-80a5-a66004acf0bf"
#define CHARACTERISTIC_UUID "2df03c7d-8ac5-493e-9d88-ac467aed4ad0"
#define bleServerName       "ppg"

// =========================
// PulseSensor Pins (ESP32-S3 Feather)
// =========================
const int PULSE_INPUT = A0;   // GPIO 17
const int PULSE_BLINK = 13;   // Built-in LED (works)
const int PULSE_FADE  = 5;    // PWM-capable pin (OK)
const int THRESHOLD   = 685;

// PulseSensor object
PulseSensorPlayground pulseSensor;

// BLE objects
BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristic = nullptr;

bool deviceConnected = false;

// =========================
// BLE Callbacks
// =========================
class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    pServer->startAdvertising();
  }
};

// =========================
// Setup
// =========================
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Set LED pins
  pinMode(PULSE_BLINK, OUTPUT);
  pinMode(PULSE_FADE, OUTPUT);

  // -------------------------
  // PulseSensor Setup
  // -------------------------
  analogReadResolution(10);

  pulseSensor.analogInput(PULSE_INPUT);
  pulseSensor.blinkOnPulse(PULSE_BLINK);
  pulseSensor.fadeOnPulse(PULSE_FADE);
  pulseSensor.setSerial(Serial);
  pulseSensor.setThreshold(THRESHOLD);

  if (!pulseSensor.begin()) {
    Serial.println("PulseSensor initialization FAILED.");
    while (1) {
      digitalWrite(PULSE_BLINK, LOW);
      delay(50);
      digitalWrite(PULSE_BLINK, HIGH);
      delay(50);
    }
  }
  Serial.println("PulseSensor initialized!");

  // -------------------------
  // BLE Setup
  // -------------------------
  BLEDevice::init(bleServerName);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *service = pServer->createService(SERVICE_UUID);

  pCharacteristic = service->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("BLE Advertising started…");
}

// =========================
// Loop
// =========================
void loop() {

  if (pulseSensor.sawStartOfBeat()) {
    int bpm = pulseSensor.getBeatsPerMinute();

    Serial.print("BPM: ");
    Serial.println(bpm);

    if (deviceConnected) {
      String bpmString = String(bpm);
      pCharacteristic->setValue(bpmString.c_str());
      pCharacteristic->notify();
    }
  }

  delay(20);
}
