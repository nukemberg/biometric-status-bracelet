/*
 * MAX30102 bring-up probe.
 *
 * Diagnostic only -- not part of the product. Exists because the first deploy showed
 * the part responding to its PART_ID read while OVF_COUNTER sat pegged at 0x1F on
 * every FIFO poll and no samples ever came out. That combination has several possible
 * causes and guessing between them is how you end up "fixing" the wrong one:
 *
 *   1. Config writes are not landing (part is sampling at some default rate).
 *   2. Config writes land but the sample rate is far higher than intended.
 *   3. The repeated-start read is malformed and the pointer bytes are garbage that
 *      happens to read as 0x1F.
 *
 * So this reads the config registers BACK and dumps the raw pointer bytes. Whatever
 * the answer is, it is visible here rather than inferred.
 */

#include <Arduino.h>
#include <Wire.h>

#define PIN_SDA  21
#define PIN_SCL  22
#define ADDR     0x57

void wr(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  Wire.write(val);
  uint8_t rc = Wire.endTransmission();
  if (rc) { Serial.print(F("  write 0x")); Serial.print(reg, HEX);
            Serial.print(F(" FAILED rc=")); Serial.println(rc); }
}

// Deliberately uses a STOP between address and read, not a repeated start, so this
// probe does not share the suspect path with the driver.
uint8_t rd(uint8_t reg) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  Wire.endTransmission(true);
  if (Wire.requestFrom((int)ADDR, 1) != 1) return 0xEE;
  return Wire.read();
}

void dumpCfg(const char *when) {
  Serial.print(F("--- config ")); Serial.println(when);
  Serial.print(F("  PART_ID   0xFF = 0x")); Serial.println(rd(0xFF), HEX);
  Serial.print(F("  REV_ID    0xFE = 0x")); Serial.println(rd(0xFE), HEX);
  Serial.print(F("  INT_EN1   0x02 = 0x")); Serial.println(rd(0x02), HEX);
  Serial.print(F("  FIFO_CFG  0x08 = 0x")); Serial.println(rd(0x08), HEX);
  Serial.print(F("  MODE_CFG  0x09 = 0x")); Serial.println(rd(0x09), HEX);
  Serial.print(F("  SPO2_CFG  0x0A = 0x")); Serial.println(rd(0x0A), HEX);
  Serial.print(F("  LED1_PA   0x0C = 0x")); Serial.println(rd(0x0C), HEX);
  Serial.print(F("  LED2_PA   0x0D = 0x")); Serial.println(rd(0x0D), HEX);
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println(F("\n=== MAX30102 probe ==="));

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  Serial.println(F("--- bus scan"));
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  device at 0x")); Serial.println(a, HEX);
    }
  }

  dumpCfg("at power-on (before any write)");

  Serial.println(F("--- reset"));
  wr(0x09, 0x40);
  for (int i = 0; i < 50 && (rd(0x09) & 0x40); i++) delay(2);
  Serial.print(F("  MODE_CFG after reset = 0x")); Serial.println(rd(0x09), HEX);

  dumpCfg("after reset, before config");

  Serial.println(F("--- writing config"));
  wr(0x08, (0b010 << 5) | (1 << 4) | 0x00);   // SMP_AVE=4, rollover on
  wr(0x09, 0x03);                              // SpO2 mode
  wr(0x0A, (0b01 << 5) | (0b001 << 2) | 0b11); // 8192nA, 100Hz, 411us
  wr(0x0C, 0x32);
  wr(0x0D, 0x32);
  wr(0x02, 1 << 6);

  dumpCfg("after config");

  // Clear FIFO
  wr(0x04, 0); wr(0x05, 0); wr(0x06, 0);

  Serial.println(F("\n--- FIFO pointers, raw, every 200 ms"));
  Serial.println(F("    (at 25 Hz expect avail to sit at 4-6 between polls)"));
}

uint32_t lastMs = 0;
uint32_t totalSamples = 0;
uint32_t started = 0;

void loop() {
  if (millis() - lastMs < 200) return;
  lastMs = millis();
  if (!started) started = millis();

  // Individual reads, STOP between address and data.
  uint8_t wrp = rd(0x04), ovf = rd(0x05), rdp = rd(0x06);
  uint8_t avail = (uint8_t)((wrp - rdp) & 0x1F);

  // Burst read of the same three registers with a REPEATED START -- exactly what the
  // driver does. If these disagree with the three above, the driver's read path is
  // the bug; if they agree, OVF_COUNTER itself is what the driver misinterprets.
  uint8_t b3[3] = {0xEE, 0xEE, 0xEE};
  Wire.beginTransmission(ADDR);
  Wire.write(0x04);
  if (Wire.endTransmission(false) == 0 && Wire.requestFrom((int)ADDR, 3) == 3) {
    for (int i = 0; i < 3; i++) b3[i] = Wire.read();
  }

  Serial.print(F("  WR=")); Serial.print(wrp);
  Serial.print(F(" OVF=")); Serial.print(ovf);
  Serial.print(F(" RD=")); Serial.print(rdp);
  Serial.print(F(" avail=")); Serial.print(avail);
  Serial.print(F(" | burst[")); Serial.print(b3[0]);
  Serial.print(','); Serial.print(b3[1]);
  Serial.print(','); Serial.print(b3[2]); Serial.print(']');

  // Drain whatever is there and show the first IR value, so we can tell "no samples"
  // from "samples that are all zero".
  if (avail) {
    uint8_t n = avail > 8 ? 8 : avail;
    Wire.beginTransmission(ADDR);
    Wire.write(0x07);
    Wire.endTransmission(true);
    uint8_t got = Wire.requestFrom((int)ADDR, (int)n * 6);
    if (got == n * 6) {
      uint32_t red = 0, ir = 0;
      for (uint8_t i = 0; i < n; i++) {
        uint8_t b[6];
        for (int j = 0; j < 6; j++) b[j] = Wire.read();
        red = ((uint32_t)b[0] << 16 | (uint32_t)b[1] << 8 | b[2]) & 0x03FFFF;
        ir  = ((uint32_t)b[3] << 16 | (uint32_t)b[4] << 8 | b[5]) & 0x03FFFF;
      }
      totalSamples += n;
      Serial.print(F("  RED=")); Serial.print(red);
      Serial.print(F(" IR=")); Serial.print(ir);
    } else {
      Serial.print(F("  FIFO read short: ")); Serial.print(got);
      while (Wire.available()) Wire.read();
    }
    wr(0x05, 0);   // clear overflow counter only
  }

  float secs = (millis() - started) / 1000.0f;
  Serial.print(F("  | rate="));
  Serial.print(secs > 0 ? totalSamples / secs : 0.0f, 2);
  Serial.println(F(" Hz"));
}
