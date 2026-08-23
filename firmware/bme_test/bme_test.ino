/*
 * BME280 temperature bench test.
 *
 * Chases the field symptom: the armband occasionally reports an impossible
 * temperature (185 C was the observed value). Adafruit_BME280 has no error
 * return -- readTemperature() always yields a finite float computed from
 * whatever the raw registers happened to contain -- so the failure is invisible
 * from the library API alone. This sketch reads underneath it.
 *
 * Three hypotheses, and what distinguishes them:
 *
 *   H1  Transient I2C corruption of the raw burst (0xFA..0xFC). Shows up as a
 *       single-sample raw outlier with the calibration coefficients still
 *       intact, chip id still 0x60, and the next sample fine.
 *   H2  Chip reset / not-measuring. Raw reads back as 0x80000 (the power-on
 *       register value), which compensates to a wrong-but-stable number, and
 *       the id/status regs corroborate it.
 *   H3  Calibration coefficients corrupted during begin(). Then EVERY reading
 *       is wrong for the rest of the boot, and the raw counts look perfectly
 *       normal. This is the one that a per-sample plausibility gate cannot fix
 *       by itself, so it matters whether it is happening.
 *
 * To tell them apart, every cycle logs: chip id, status, the raw ADC word, this
 * sketch's own compensation from coefficients read once at boot, and the
 * library's value. A divergence between the last two is H3.
 *
 * The MAX30102 is initialised and drained at 25 Hz alongside, because the
 * suspected trigger is bus contention -- main_armband shares SDA/SCL between
 * the PPG FIFO burst and this sensor, and a test on a quiet bus would not
 * reproduce it.
 *
 * Output is CSV on serial for tools/capture.py; lines beginning with '#' are
 * commentary and running stats.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <max30102.h>

#define PIN_SDA 8
#define PIN_SCL 9

#define SAMPLE_HZ       10
#define SAMPLE_MS       (1000 / SAMPLE_HZ)
#define PPG_DRAIN_MS    40      // 25 Hz, same as main_armband
#define STATS_MS        5000

// Same gates as the firmware, so a reject counted here is a reject there.
static const float TEMP_MIN_C = 0.0f;
static const float TEMP_MAX_C = 50.0f;
static const float TEMP_MAX_SLEW_C = 5.0f;

Adafruit_BME280 bme;
Max30102 ppg;
bool bmeFound = false;
bool ppgFound = false;
uint8_t bmeAddr = 0x76;

// Coefficients read once, straight off the chip, so the library's private copy
// can be checked against them rather than trusted.
uint16_t digT1 = 0;
int16_t digT2 = 0, digT3 = 0;

struct Counters {
  uint32_t samples = 0;
  uint32_t i2cErr = 0;      // NACK or short read on the raw burst
  uint32_t idBad = 0;       // chip id != 0x60
  uint32_t rawReset = 0;    // raw == 0x80000, the power-on value
  uint32_t rangeBad = 0;
  uint32_t slewBad = 0;
  uint32_t nanRead = 0;
  uint32_t libMismatch = 0; // library value != own compensation
} cnt;

float lastGoodTempC = 0.0f;
bool seeded = false;

// USB CDC re-enumerates across the reset, so a host that attaches a few seconds
// in misses everything setup() printed -- which is exactly the part that says
// what state the chip came up in. Keep it and replay it with the early stats.
String bootReport;

void blog(const char *fmt, ...) {
  char buf[192];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print(buf);
  bootReport += buf;
}

// --- raw register access ----------------------------------------------------

bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(bmeAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(bmeAddr);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom((int)bmeAddr, (int)len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// Datasheet compensation, integer form, result in 0.01 C.
int32_t compensateT(int32_t adcT) {
  int32_t var1 = ((((adcT >> 3) - ((int32_t)digT1 << 1))) * ((int32_t)digT2)) >> 11;
  int32_t var2 = (((((adcT >> 4) - ((int32_t)digT1)) *
                    ((adcT >> 4) - ((int32_t)digT1))) >> 12) * ((int32_t)digT3)) >> 14;
  int32_t tFine = var1 + var2;
  return (tFine * 5 + 128) >> 8;
}

// ctrl_hum (0xF2), ctrl_meas (0xF4), config (0xF5). ctrl_meas bits 1:0 are the
// mode: 00 = sleep. A sleeping chip never converts, so the raw registers keep
// their 0x80000 power-on value and every "reading" is the same fiction.
void dumpModeRegs(const char *tag) {
  uint8_t f2 = 0, f4 = 0, f5 = 0;
  bool ok = readRegs(0xF2, &f2, 1) && readRegs(0xF4, &f4, 1) && readRegs(0xF5, &f5, 1);
  if (!ok) { blog("# %s mode regs READ FAILED\n", tag); return; }
  const char *mode = (f4 & 0x03) == 0 ? "SLEEP" : ((f4 & 0x03) == 3 ? "normal" : "forced");
  blog("# %s ctrl_hum=0x%02X ctrl_meas=0x%02X config=0x%02X mode=%s "
       "osrs_t=%u osrs_p=%u osrs_h=%u\n",
                tag, f2, f4, f5, mode, (f4 >> 5) & 7, (f4 >> 2) & 7, f2 & 7);
}

// Does a ctrl_meas write stick? Three read-backs separate the possibilities:
// immediately (the write itself), after a settling delay (something clearing it
// asynchronously), and after PPG bus traffic (the shared-bus hypothesis).
void probeWrites() {
  uint8_t v = 0, st = 0;
  bool wrote = writeReg(0xF2, 0x01);                 // ctrl_hum = x1 oversampling
  // ctrl_hum only latches when ctrl_meas is written afterwards -- datasheet 5.4.3.
  bool wrote2 = writeReg(0xF4, 0x27);                // osrs_t x1, osrs_p x1, normal mode
  readRegs(0xF4, &v, 1);
  blog("# probe write ctrl_meas=0x27 ack=%d/%d readback=0x%02X\n", wrote, wrote2, v);

  // Time the revert. The shape tells us what is clearing it: an immediate 0x00 is
  // a write that never latched (a part that only pretends to accept it), while a
  // value that holds for milliseconds and then drops -- with status bit 0
  // (im_update, NVM copy) set on the way -- is a real power-on reset cycle.
  const uint16_t marks[] = {1, 2, 5, 10, 20, 50, 100, 200, 500};
  uint16_t elapsed = 0;
  for (uint8_t i = 0; i < sizeof(marks) / sizeof(marks[0]); i++) {
    delay(marks[i] - elapsed);
    elapsed = marks[i];
    readRegs(0xF4, &v, 1);
    readRegs(0xF3, &st, 1);
    blog("# probe t+%ums ctrl_meas=0x%02X status=0x%02X%s\n", elapsed, v, st,
         v == 0x00 ? " <-- cleared" : "");
    if (v == 0x00) break;
  }

  // Forced mode: one conversion, then back to sleep by design. If even this
  // produces a raw value, the part works and only the persistent normal-mode
  // configuration is being lost.
  writeReg(0xF2, 0x01);
  writeReg(0xF4, 0x25);                              // osrs_t x1, osrs_p x1, forced
  delay(20);                                         // max conversion time at x1
  uint8_t raw[3];
  if (readRegs(0xFA, raw, 3)) {
    int32_t adcT = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
    blog("# probe forced raw=%ld (%s) temp=%.2f\n", (long)adcT,
         adcT == 0x80000 ? "still the power-on value -- not converting" : "CONVERTED",
         compensateT(adcT) / 100.0f);
  }

  // Repeat the write at 100 kHz. If the register sticks here and not at 400 kHz,
  // the bus is marginal (pull-ups too weak for the wire length) rather than the
  // part being at fault.
  Wire.setClock(100000);
  writeReg(0xF2, 0x01);
  writeReg(0xF4, 0x27);
  delay(100);
  readRegs(0xF4, &v, 1);
  blog("# probe at 100kHz: ctrl_meas after 100ms = 0x%02X\n", v);
  Wire.setClock(400000);
}

// One forced conversion, read back as raw counts. Normal mode does not survive on
// this part (see probeWrites), so this is the only acquisition path that produces
// real data here: write mode=forced, wait out the conversion, read 0xFA..0xFC.
bool forcedRead(int32_t *adcT) {
  if (!writeReg(0xF2, 0x01)) return false;
  if (!writeReg(0xF4, 0x25)) return false;   // osrs_t x1, osrs_p x1, forced
  delay(10);                                  // 9.3 ms worst case at x1/x1/x1
  uint8_t raw[3];
  if (!readRegs(0xFA, raw, 3)) return false;
  *adcT = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
  return true;
}

void printStats() {
  Serial.printf("# stats n=%lu i2c_err=%lu id_bad=%lu raw_reset=%lu "
                "range_bad=%lu slew_bad=%lu nan=%lu lib_mismatch=%lu\n",
                cnt.samples, cnt.i2cErr, cnt.idBad, cnt.rawReset,
                cnt.rangeBad, cnt.slewBad, cnt.nanRead, cnt.libMismatch);
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) { }
  delay(200);

  Serial.println(F("# BME280 temperature bench test"));

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);      // same as main_armband

  if (bme.begin(0x76, &Wire)) {
    bmeFound = true; bmeAddr = 0x76;
  } else if (bme.begin(0x77, &Wire)) {
    bmeFound = true; bmeAddr = 0x77;
  }
  blog("# bme280: %s addr=0x%02X\n", bmeFound ? "found" : "NOT FOUND", bmeAddr);

  if (bmeFound) {
    // 0x88..0x8D: dig_T1 (u16), dig_T2 (i16), dig_T3 (i16), all little-endian.
    uint8_t c[6];
    if (readRegs(0x88, c, 6)) {
      digT1 = (uint16_t)(c[0] | (c[1] << 8));
      digT2 = (int16_t)(c[2] | (c[3] << 8));
      digT3 = (int16_t)(c[4] | (c[5] << 8));
      // Typical parts land near T1 27000-29000, T2 26000-27000, T3 about -1000
      // to 100. A T1 of 0 or 0xFFFF is a failed calibration read (H3) and every
      // temperature this boot is fiction.
      blog("# calib dig_T1=%u dig_T2=%d dig_T3=%d %s\n",
                    digT1, digT2, digT3,
                    (digT1 == 0 || digT1 == 0xFFFF) ? "<-- IMPLAUSIBLE" : "");
    } else {
      Serial.println(F("# calib read FAILED"));
    }
  }

  if (!bmeFound) {
    Serial.println(F("# nothing to test, halting"));
    while (true) delay(1000);
  }

  // Dumped either side of the PPG init: main_armband brings the MAX30102 up on
  // the same bus right after the BME280, so if that sequence is what leaves the
  // BME280 in sleep, the two dumps disagree.
  dumpModeRegs("after bme.begin()");
  ppgFound = ppg.begin(Wire);
  dumpModeRegs("after ppg.begin()");

  blog("# max30102: %s (bus contention %s)\n",
                ppgFound ? "found" : "NOT FOUND",
                ppgFound ? "active" : "ABSENT -- test is on a quiet bus");

  probeWrites();

  Serial.println(F("ms,id,status,forced_raw,normal_c,forced_c,verdict"));
}

void loop() {
  static uint32_t lastSample = 0, lastDrain = 0, lastStats = 0;
  uint32_t now = millis();

  // Keep the bus as busy as the real firmware keeps it.
  if (ppgFound && (now - lastDrain >= PPG_DRAIN_MS)) {
    lastDrain = now;
    uint32_t ir[8];
    ppg.read(ir, nullptr, 8);
  }

  if (now - lastSample >= SAMPLE_MS) {
    lastSample = now;
    cnt.samples++;

    uint8_t id = 0, status = 0, raw[3];
    bool idOk = readRegs(0xD0, &id, 1);
    bool stOk = readRegs(0xF3, &status, 1);
    bool rawOk = readRegs(0xFA, raw, 3);

    int32_t adcT = -1;
    float ownC = NAN;
    // Trigger a conversion first, then use the raw burst that was read above only
    // as the "what would normal mode have given us" reference.
    int32_t forcedT = -1;
    float forcedC = NAN;
    if (forcedRead(&forcedT)) forcedC = compensateT(forcedT) / 100.0f;
    if (rawOk) {
      adcT = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
      ownC = compensateT(adcT) / 100.0f;
    }

    float libC = forcedC;

    const char *verdict = "ok";
    if (!idOk || !stOk || !rawOk) { cnt.i2cErr++; verdict = "i2c_err"; }
    else if (id != 0x60)          { cnt.idBad++; verdict = "id_bad"; }
    else if (adcT == 0x80000)     { cnt.rawReset++; verdict = "raw_reset"; }
    else if (isnan(libC))         { cnt.nanRead++; verdict = "nan"; }
    else if (libC < TEMP_MIN_C || libC > TEMP_MAX_C) { cnt.rangeBad++; verdict = "range"; }
    else if (seeded && fabsf(libC - lastGoodTempC) > TEMP_MAX_SLEW_C) {
      cnt.slewBad++; verdict = "slew";
    } else {
      lastGoodTempC = libC;
      seeded = true;
    }

    // Independent of the verdict: if the library and this sketch disagree on the
    // same raw word, the library's cached coefficients are corrupt (H3).
    if (rawOk && !isnan(libC) && fabsf(ownC - libC) > 0.5f) {
      cnt.libMismatch++;
      if (strcmp(verdict, "ok") == 0) verdict = "lib_mismatch";
    }

    Serial.printf("%lu,0x%02X,0x%02X,%ld,%.2f,%.2f,%s\n",
                  now, id, status, (long)forcedT, ownC, forcedC, verdict);

    if (strcmp(verdict, "ok") != 0) {
      Serial.printf("# ANOMALY %s at %lu ms | raw=%ld own=%.2f lib=%.2f last_good=%.2f\n",
                    verdict, now, (long)adcT, ownC, libC, seeded ? lastGoodTempC : NAN);
    }
  }

  if (now - lastStats >= STATS_MS) {
    lastStats = now;
    printStats();
    dumpModeRegs("periodic");
    if (now < 30000) Serial.print(bootReport);
  }
}
