#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "SparkFun_LIS2DH12.h"   // Library edited so isConnected() only checks ACK
SPARKFUN_LIS2DH12 accel;

// BLE UUIDs (same hub as your PPG code)
#define SERVICE_UUID        "e2f4d9b8-9992-474d-a3b1-db7cea750f93"
#define CHARACTERISTIC_UUID "f446fe54-ab3a-4e86-ab0c-22f42e17ea83"

// BLE server name
#define bleServerName "accel"

BLECharacteristic *pCharacteristic;

unsigned long lastNotifyTime = 0;
const unsigned long notifyInterval = 1000; // ms, adjust as needed

void setup() {
  Serial.begin(115200);
  Serial.println("LIS2DE12 + BLE example");

  // I2C + accelerometer
  Wire.begin(); // adjust SDA/SCL pins if using ESP32-C3 with non-default pins

  if (!accel.begin()) {
    Serial.println("Accelerometer not detected. Check wiring. Freezing...");
    while (1);
  }

  // Optional: set range and data rate if needed
  // accel.setScale(LIS2DH12_2G);      // default is ±2 g
  // accel.setOutputDataRate(LIS2DH12_ODR_100Hz);

  // BLE setup
  BLEDevice::init(bleServerName);
  BLEServer *server = BLEDevice::createServer();
  BLEService *service = server->createService(SERVICE_UUID);

  pCharacteristic = service->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );

  pCharacteristic->addDescriptor(new BLE2902());
  service->start();
  server->getAdvertising()->start();

  Serial.println("BLE Accelerometer Server Started. Waiting for client...");
}

void loop() {
  // Only read when new data is available from LIS2DE12
  if (accel.available()) {
    float accelX = accel.getX(); // mg
    float accelY = accel.getY();
    float accelZ = accel.getZ();
    float tempC = accel.getTemperature();

    // Serial debug
    Serial.print("Acc [mg]: ");
    Serial.print(accelX, 1);
    Serial.print(" x, ");
    Serial.print(accelY, 1);
    Serial.print(" y, ");
    Serial.print(accelZ, 1);
    Serial.print(" z, ");
    Serial.print(tempC, 1);
    Serial.println(" C");

    // ---- BLE Notification (rate-limited) ----
    unsigned long now = millis();
    if (now - lastNotifyTime >= notifyInterval) {
      lastNotifyTime = now;

      // Pack data into bytes.
      // Here: 2 bytes per axis (int16 = mg), 1 byte temperature (offset by +40 to avoid negative).
      int16_t x_mg = (int16_t)accelX;
      int16_t y_mg = (int16_t)accelY;
      int16_t z_mg = (int16_t)accelZ;
      int8_t temp_q = (int8_t)tempC;  // crude 1 °C resolution

      uint8_t packet[7];
      packet[0] = (uint8_t)(x_mg & 0xFF);
      packet[1] = (uint8_t)((x_mg >> 8) & 0xFF);
      packet[2] = (uint8_t)(y_mg & 0xFF);
      packet[3] = (uint8_t)((y_mg >> 8) & 0xFF);
      packet[4] = (uint8_t)(z_mg & 0xFF);
      packet[5] = (uint8_t)((z_mg >> 8) & 0xFF);
      packet[6] = (uint8_t)temp_q;

      pCharacteristic->setValue(packet, sizeof(packet));
      pCharacteristic->notify();
    }
  }
}
