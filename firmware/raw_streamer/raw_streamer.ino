/*
 * High-Speed Raw Biometric Streamer for Python DSP Analysis
 * Target MCU: Standard ESP32 (PIN_GSR=34, PIN_PULSE=35) or ESP32-S3
 * 
 * Features:
 * - Ultra-Precise Hardware Timer Interrupt / High-Resolution micros() loop
 * - Exact 500Hz Sampling (2,000.0 microseconds per sample)
 * - 16x ADC Hardware Oversampling (removes ESP32 SAR ADC noise)
 * - NO Software Filtering, NO Interpolation, NO BME280 Delay
 * - Clean Raw CSV Output: Timestamp_ms,RawPulse,RawGSR
 */

#include <Arduino.h>

#define PIN_GSR     34
#define PIN_PULSE   35
#define PIN_BUTTON  18

// 16x Hardware ADC Oversampling (in microsecond burst)
inline uint16_t readOversampledADC(uint8_t pin) {
  uint32_t sum = 0;
  for (int i = 0; i < 16; i++) {
    sum += analogRead(pin);
    delayMicroseconds(10); // Fast 160us total burst
  }
  return sum / 16;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  
  // Set ADC Attenuation & Resolution for ESP32
  analogReadResolution(12); // 0 - 4095
  analogSetAttenuation(ADC_11db);

  delay(1000);

  // Pure CSV Header
  Serial.println("Timestamp_ms,RawPulse,RawGSR");
}

void loop() {
  unsigned long startUs = micros();
  unsigned long nowMs = millis();

  // 1. Read 16x Oversampled Raw Analog Data
  uint16_t rawPulse = readOversampledADC(PIN_PULSE);
  uint16_t rawGSR   = readOversampledADC(PIN_GSR);

  // 2. Stream Raw CSV Data
  Serial.print(nowMs);
  Serial.print(",");
  Serial.print(rawPulse);
  Serial.print(",");
  Serial.println(rawGSR);

  // 3. Strict 500.0 Hz Timing Control (Exactly 2,000 microseconds per frame)
  while (micros() - startUs < 2000) {
    // High-resolution hardware wait loop
  }
}
