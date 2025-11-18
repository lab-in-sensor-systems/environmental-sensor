#include <Wire.h>
#include "MAX30105.h"            // SparkFun MAX3010x library
#include "spo2_algorithm.h"      // Maxim's SpO2/HR algorithm

MAX30105 particleSensor;

// Choose buffer width per SparkFun/Maxim examples (100 samples is common)
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  uint16_t irBuffer[100];
  uint16_t redBuffer[100];
#else
  uint32_t irBuffer[100];
  uint32_t redBuffer[100];
#endif

int32_t bufferLength = 100;       // Samples in buffer
int32_t spo2 = 0;                 // SpO2 result
int8_t validSPO2 = 0;             // SpO2 validity flag
int32_t heartRate = 0;            // HR result
int8_t validHeartRate = 0;        // HR validity flag

void setup() {
  Serial.begin(115200);
  // ESP32-C3 default I2C pins are often SDA=5, SCL=6; adjust to your wiring if different.
  Wire.begin(); // or Wire.begin(SDA_PIN, SCL_PIN, 400000)

  // Initialize MAX30102
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX3010x not found. Check wiring/power/I2C.");
    while (1) { delay(1000); }
  }

  // Configure recommended settings for SpO2: Red+IR, suitable SR, PW, ADC range
  byte ledBrightness = 60;   // 0..255 (50mA at 255). Tune for your module.
  byte sampleAverage = 4;    // 1, 2, 4, 8, 16, 32
  byte ledMode = 2;          // 2 = Red + IR (required for SpO2)
  byte sampleRate = 100;     // 50..3200 sps; 100sps commonly used
  int pulseWidth = 411;      // 69, 118, 215, 411 us (longer = higher resolution)
  int adcRange = 4096;       // 2048, 4096, 8192, 16384 nA
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

  Serial.println("Attach finger on sensor and keep still...");
}

void loop() {
  // Prime initial 100 samples (about 1–2 s depending on SR)
  for (int i = 0; i < bufferLength; i++) {
    while (particleSensor.available() == false) {
      particleSensor.check();
    }
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();

    // Optional: print raw for debugging
    // Serial.printf("red=%lu, ir=%lu\r\n", (unsigned long)redBuffer[i], (unsigned long)irBuffer[i]);
  }

  // Compute initial HR and SpO2
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength,
                                         redBuffer, &spo2, &validSPO2,
                                         &heartRate, &validHeartRate);

  // Continuous update: every 25 new samples, recompute on sliding window
  while (true) {
    // Shift 75 samples up, discard oldest 25
    for (int i = 25; i < 100; i++) {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25]  = irBuffer[i];
    }

    // Acquire 25 new samples
    for (int i = 75; i < 100; i++) {
      while (particleSensor.available() == false) {
        particleSensor.check();
      }
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i]  = particleSensor.getIR();
      particleSensor.nextSample();
    }

    // Recompute HR and SpO2
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength,
                                           redBuffer, &spo2, &validSPO2,
                                           &heartRate, &validHeartRate);

    // Print labeled, newline-terminated output
    Serial.printf("IR=%lu, RED=%lu, HR=%ld, HRvalid=%d, SpO2=%ld, SpO2valid=%d\r\n",
                  (unsigned long)irBuffer[99],
                  (unsigned long)redBuffer[99],
                  (long)heartRate, (int)validHeartRate,
                  (long)spo2, (int)validSPO2);
  }
}
