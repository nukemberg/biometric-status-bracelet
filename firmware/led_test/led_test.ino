/*
 * Raw GPIO10 (MAX30102 INT) sampling test -- no ISR, no DSP task, just polling
 * digitalRead() as fast as possible to see if the line ever goes LOW at all.
 * Isolates "is the physical line toggling" from "is the ISR/interrupt path working".
 *
 * Compile/upload with CDCOnBoot=cdc.
 */

#include <Wire.h>
#include <max30102.h>

#define PIN_SDA  8
#define PIN_SCL  9
#define PIN_INT  10

Max30102 ppg;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  bool ok = ppg.begin(Wire);
  Serial.println(ok ? F("MAX30102 ready") : F("MAX30102 NOT FOUND"));

  pinMode(PIN_INT, INPUT);

  Serial.println(F("Sampling GPIO10 raw state for 5s..."));
  uint32_t lowCount = 0, highCount = 0, transitions = 0;
  int lastState = digitalRead(PIN_INT);
  uint32_t start = millis();
  uint32_t irBatch[8];

  while (millis() - start < 5000) {
    int state = digitalRead(PIN_INT);
    if (state == LOW) lowCount++; else highCount++;
    if (state != lastState) transitions++;
    lastState = state;

    // Drain the FIFO like the real firmware does, so PPG_RDY actually gets
    // asserted/cleared during this window instead of sitting idle.
    ppg.read(irBatch, nullptr, 8);
  }

  Serial.print(F("low="));
  Serial.print(lowCount);
  Serial.print(F(" high="));
  Serial.print(highCount);
  Serial.print(F(" transitions="));
  Serial.println(transitions);
}

void loop() {
  delay(1000);
}
