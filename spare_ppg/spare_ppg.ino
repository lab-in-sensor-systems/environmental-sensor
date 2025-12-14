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
const int PULSE_BLINK = 13;   // Built-in LED
const int PULSE_FADE  = 5;    // PWM-capable pin
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
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("[BLE] Central connected");
  }
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    Serial.println("[BLE] Central disconnected, restarting advertising");
    pServer->startAdvertising();
  }
};

// =========================
// Setup
// =========================
void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println();
  Serial.println("===== SPARE PPG BOOT =====");

  // Set LED pins
  pinMode(PULSE_BLINK, OUTPUT);
  pinMode(PULSE_FADE, OUTPUT);

  // -------------------------
  // PulseSensor Setup
  // -------------------------
  Serial.println("[INIT] PulseSensor...");
  analogReadResolution(10);

  pulseSensor.analogInput(PULSE_INPUT);
  pulseSensor.blinkOnPulse(PULSE_BLINK);
  pulseSensor.fadeOnPulse(PULSE_FADE);
  pulseSensor.setSerial(Serial);
  pulseSensor.setThreshold(THRESHOLD);

  if (!pulseSensor.begin()) {
    Serial.println("[INIT] PulseSensor initialization FAILED. Check wiring.");
    while (1) {
      digitalWrite(PULSE_BLINK, LOW);
      delay(50);
      digitalWrite(PULSE_BLINK, HIGH);
      delay(50);
    }
  }
  Serial.println("[INIT] PulseSensor initialized OK");

  // -------------------------
  // BLE Setup
  // -------------------------
  Serial.println("[INIT] BLE server setup...");
  BLEDevice::init(bleServerName);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *service = pServer->createService(SERVICE_UUID);

  pCharacteristic = service->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  // Optional: set an initial value so you can read it from a scanner
  pCharacteristic->setValue("0");

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("[BLE] Advertising started, waiting for central...");
}

// =========================
// Loop
// =========================
void loop() {
  // Check for heartbeat
  if (pulseSensor.sawStartOfBeat()) {
    int bpm = pulseSensor.getBeatsPerMinute();

    Serial.println("♥  Heartbeat detected!");
    Serial.print("[PPG] Local BPM: ");
    Serial.println(bpm);

    if (deviceConnected) {
      Serial.print("[PPG] Sending BPM over BLE: ");
      Serial.println(bpm);

      String bpmString = String(bpm);
      pCharacteristic->setValue(bpmString.c_str());
      pCharacteristic->notify();
    } else {
      Serial.println("[PPG] No BLE central connected, not sending BPM");
    }
  }

  delay(20);
}
