#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

// Buffers for Maxim algorithm
uint32_t irBuffer[100];
uint32_t redBuffer[100];
int32_t bufferLength = 100;
int32_t spo2 = 0, heartRate = 0;
int8_t validSPO2 = 0, validHeartRate = 0;

// BLE UUIDs
#define SERVICE_UUID        "d3aa6a66-a623-4c76-80a5-a66004acf0bf"
#define CHARACTERISTIC_UUID "2df03c7d-8ac5-493e-9d88-ac467aed4ad0"

BLECharacteristic *pCharacteristic;

unsigned long lastNotifyTime = 0;
const unsigned long notifyInterval = 1000; // ms

void setup() {
  Serial.begin(115200);

  // I2C + MAX30102
  Wire.begin(); // adjust SDA/SCL if needed

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX3010x Sensor not found!");
    while (1);
  }

  // MAX30102 recommended settings
  byte ledBrightness = 60;
  byte sampleAverage = 4;
  byte ledMode = 2;       // Red + IR
  byte sampleRate = 100;
  int pulseWidth = 411;
  int adcRange = 4096;

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

  // BLE setup
  BLEDevice::init("ESP32_SPO2_SERVER");
  BLEServer *server = BLEDevice::createServer();
  BLEService *service = server->createService(SERVICE_UUID);

  pCharacteristic = service->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );

  pCharacteristic->addDescriptor(new BLE2902());
  service->start();
  server->getAdvertising()->start();

  Serial.println("BLE Server Started. Waiting for client...");

  // ---- Prime initial 100 samples ----
  for (int i = 0; i < bufferLength; i++) {
    while (!particleSensor.available()) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  // Initial HR + SpO2 computation
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, bufferLength,
    redBuffer,
    &spo2, &validSPO2,
    &heartRate, &validHeartRate
  );
}

void loop() {
  // ---- Sliding window: every 25 new samples ----
  for (int i = 25; i < 100; i++) {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25] = irBuffer[i];
  }

  for (int i = 75; i < 100; i++) {
    while (!particleSensor.available()) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  // Compute HR + SpO2
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, bufferLength,
    redBuffer,
    &spo2, &validSPO2,
    &heartRate, &validHeartRate
  );

  // Print to Serial for debugging
  Serial.printf("HR=%ld valid=%d | SpO2=%ld valid=%d\n",
        heartRate, validHeartRate, spo2, validSPO2);

  // ---- BLE Notification (once per second) ----
  unsigned long now = millis();
  if (now - lastNotifyTime >= notifyInterval) {
    lastNotifyTime = now;

    uint8_t packet[4];
    packet[0] = (uint8_t)heartRate;
    packet[1] = validHeartRate;
    packet[2] = (uint8_t)spo2;
    packet[3] = validSPO2;

    pCharacteristic->setValue(packet, 4);
    pCharacteristic->notify();
  }
}
