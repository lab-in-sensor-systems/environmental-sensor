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
// PulseSensor Pins
// =========================
const int PULSE_INPUT = A0;
const int PULSE_BLINK = 13;
const int PULSE_FADE  = 5;
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
    pServer->startAdvertising();  // keep advertising
  }
};

// =========================
// Setup
// =========================
void setup() {
  Serial.begin(115200);
  delay(1000);

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

  // Start service
  service->start();

  // Start advertising
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

  // Check for beat
  if (pulseSensor.sawStartOfBeat()) {
    int bpm = pulseSensor.getBeatsPerMinute();

    Serial.print("BPM: ");
    Serial.println(bpm);

    // Send notification over BLE
    if (deviceConnected) {
      String bpmString = String(bpm);
      pCharacteristic->setValue(bpmString.c_str());
      pCharacteristic->notify();
    }
  }

  delay(20);
}
