/* This code works with MAX30102 
 * It's displays the Average BPM on the screen
 * It's a modified version of  the HeartRate library example
 * Refer to www.surtrtech.com for more details  or SurtrTech YouTube channel
 */
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
unsigned long lastBeat = 0;   // use unsigned long with millis()
float beatsPerMinute = 0.0f;
int beatAvg = 0;

void setup() {
  Serial.begin(115200);
  // ESP32-C3 default I2C pins are GPIO5 (SDA) and GPIO6 (SCL) on many boards; adjust if your board differs.
  Wire.begin();               // or Wire.begin(SDA_PIN, SCL_PIN, 400000);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX3010x not found. Check wiring/power/I2C address.");
    while (1) { delay(1000); }
  }

  particleSensor.setup();               // library defaults
  particleSensor.setPulseAmplitudeRed(0x0A); // low red LED as “running” indicator
  // Optional: setPulseAmplitudeIR/Green, sample rate, pulse width, etc.
}

void loop() {
  long irValue = particleSensor.getIR();

  if (irValue > 7000) {
    // Show the current rolling average (will be 0 until a few beats are collected)
    Serial.printf("IR=%ld, AvgBPM=%d\r\n", irValue, beatAvg);

    if (checkForBeat(irValue)) {
      unsigned long now = millis();
      unsigned long delta = now - lastBeat;
      lastBeat = now;

      if (delta > 0) {
        beatsPerMinute = 60.0f / (delta / 1000.0f);
        if (beatsPerMinute > 20.0f && beatsPerMinute < 255.0f) {
          rates[rateSpot++] = (byte)beatsPerMinute;
          rateSpot %= RATE_SIZE;

          int sum = 0;
          for (byte x = 0; x < RATE_SIZE; x++) sum += rates[x];
          beatAvg = sum / RATE_SIZE;
        }
      }

      Serial.printf("Beat! BPM=%.1f, Avg=%d\r\n", beatsPerMinute, beatAvg);
    }
  } else {
    beatAvg = 0;
    Serial.println("Place finger on PPG");
    delay(200); // small pause to avoid spamming the port
  }
}
