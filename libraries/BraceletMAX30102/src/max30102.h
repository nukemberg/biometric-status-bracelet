/*
 * BraceletMAX30102 - minimal register-level driver for the MAX30102 PPG sensor.
 *
 * Scope is deliberately narrow: configure the part, drain its FIFO, hand out IR
 * counts. There is no beat detection, no SpO2, no filtering. Rate, confidence and
 * beat phase come from the resonator bank in BraceletDSP, which is validated offline
 * against recorded captures -- a second, unvalidated estimator inside the driver
 * would be a source of numbers nobody checks.
 *
 * Why not SparkFun's MAX3010x library: it pulls in its own heart-rate and SpO2
 * estimators, buffers samples in a way that hides the FIFO pointers, and adds a
 * dependency to a project that otherwise vendors everything in libraries/. What we
 * need is about 120 lines.
 *
 * Kept out of BraceletDSP on purpose. dsp.h must stay free of Arduino types so
 * tools/dsp_v2_parity.sh can compile the real trackers on a host; this file needs
 * <Wire.h> and therefore can never live there.
 *
 * ---------------------------------------------------------------------------
 * Sample rate
 * ---------------------------------------------------------------------------
 * Configured for 100 Hz internal sampling with 4x on-chip averaging, so the FIFO
 * fills at 25 Hz -- exactly DSP_HZ. The PPG path therefore bypasses the 500 Hz ADC
 * and the 20-sample boxcar entirely; those still exist, but only for GSR.
 *
 * 411 us pulse width (18-bit ADC) with both LEDs active caps the internal rate at
 * 100 Hz per the datasheet's pulse-width/sample-rate table, which is why 100/4 is
 * used rather than 400/16. Resolution is worth more here than headroom: DESIGN.md
 * section 2.1 shows the whole v1 problem was amplitude.
 *
 * CAVEAT, and it is a real one: 25 Hz is the *nominal* FIFO rate. The MAX30102 runs
 * off its own internal oscillator, and the datasheet does not specify its tolerance.
 * The resonator bin frequencies in dsp.h are derived from DSP_HZ = 25, so if the part
 * actually delivers 25.6 Hz every reported BPM is 2.4 % low, uniformly and silently.
 * measuredHz() below exists to make that observable rather than assumed -- see
 * DESIGN.md section 4.2 for how to check it and what to do about it.
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

#define MAX30102_I2C_ADDR   0x57

// Registers used here. The part has more; these are the ones this driver touches.
#define MAX30102_REG_INT_STATUS_1  0x00
#define MAX30102_REG_INT_STATUS_2  0x01
#define MAX30102_REG_INT_ENABLE_1  0x02
#define MAX30102_REG_INT_ENABLE_2  0x03
#define MAX30102_REG_FIFO_WR_PTR   0x04
#define MAX30102_REG_OVF_COUNTER   0x05
#define MAX30102_REG_FIFO_RD_PTR   0x06
#define MAX30102_REG_FIFO_DATA     0x07
#define MAX30102_REG_FIFO_CONFIG   0x08
#define MAX30102_REG_MODE_CONFIG   0x09
#define MAX30102_REG_SPO2_CONFIG   0x0A
#define MAX30102_REG_LED1_PA       0x0C   // RED
#define MAX30102_REG_LED2_PA       0x0D   // IR
#define MAX30102_REG_PART_ID       0xFF

#define MAX30102_PART_ID           0x15

// FIFO is 32 samples deep. In SpO2 mode each sample is 6 bytes (RED then IR, 3 bytes
// each), and the ESP32 Wire buffer is 128 bytes, so a single requestFrom() can carry
// at most 21. Cap at 16: a power of two, comfortably inside the buffer, and eight
// times more than the 1-2 samples a 25 Hz drain actually expects.
#define MAX30102_DRAIN_MAX         16

// LED drive current, 0.2 mA per LSB. 0x19 = 5 mA.
//
// Tuned on skin 2026-09-02: at 0x32 (10 mA) IR DC sat 157k-159k against a 262143
// full scale, above the 20k-150k target in DESIGN.md 4.2 and close enough to the
// ADC ceiling to risk clipping the cardiac AC. Halved to 0x19; re-measure IR DC
// on the next capture and retune if it's not comfortably inside 20k-150k -- this
// depends on skin tone, hair and strap tension so it isn't a universal constant.
#define MAX30102_LED_CURRENT       0x19

struct Max30102 {
  TwoWire *wire = nullptr;
  uint8_t addr = MAX30102_I2C_ADDR;
  bool ok = false;

  // Last raw value of OVF_COUNTER (0x05). Telemetry ONLY -- deliberately not used to
  // make any decision, and named to make that obvious.
  //
  // The datasheet presents this as a latched count of samples lost to a full FIFO. On
  // the part fitted here (PART_ID 0x15, REV_ID 0x03) it is not: measured on-device, it
  // reads 5 during completely healthy operation -- WR advancing 5, RD following 5,
  // avail steady at 5, IR data clean and continuous at 24.9 Hz -- and it reads 5 again
  // on the next poll after being explicitly written to 0. It appears to track samples
  // produced since the last clear rather than samples dropped.
  //
  // An earlier version of this driver treated any non-zero value as "continuity lost,
  // resync" and so threw away every sample the sensor ever produced, while reporting a
  // plausible-looking overflow count that made it look like a scheduling problem. Do
  // not reintroduce that. Sample loss is detected from measuredHz() instead (below).
  uint8_t ovfRegister = 0;

  // Samples delivered since the last measuredHz() call, and when that window opened.
  // See the oscillator caveat at the top of the file.
  uint32_t rateCount = 0;
  uint32_t rateStartMs = 0;

  bool present() const { return ok; }

  bool begin(TwoWire &bus, uint8_t address = MAX30102_I2C_ADDR) {
    wire = &bus;
    addr = address;
    ok = false;

    if (readReg(MAX30102_REG_PART_ID) != MAX30102_PART_ID) return false;

    // Full reset, so a warm restart of the MCU does not inherit whatever mode the
    // part was left in. The bit self-clears when the reset completes.
    writeReg(MAX30102_REG_MODE_CONFIG, 0x40);
    for (int i = 0; i < 50; i++) {
      if ((readReg(MAX30102_REG_MODE_CONFIG) & 0x40) == 0) break;
      delay(2);
    }
    if (readReg(MAX30102_REG_MODE_CONFIG) & 0x40) return false;

    // FIFO: 4x averaging (0b010 << 5), rollover enabled, A_FULL left at 0.
    //
    // Rollover matters and is the same choice the signals ring buffer in
    // main_armband.ino makes: when the FIFO fills, drop the OLDEST sample and keep
    // running. With rollover off the part stops writing instead, so a late drain
    // would return data that is arbitrarily stale while still looking continuous.
    writeReg(MAX30102_REG_FIFO_CONFIG, (0b010 << 5) | (1 << 4) | 0x00);

    // SpO2 mode (RED + IR). Heart-rate mode would run the RED LED alone; we want the
    // IR channel, which penetrates deeper and is the conventional choice for
    // reflective wrist PPG. RED is powered because the mode requires it -- its
    // samples are read off the FIFO and discarded.
    writeReg(MAX30102_REG_MODE_CONFIG, 0x03);

    // SPO2_ADC_RGE 0b01 (8192 nA full scale), SPO2_SR 0b001 (100 Hz),
    // LED_PW 0b11 (411 us, 18-bit).
    writeReg(MAX30102_REG_SPO2_CONFIG, (0b01 << 5) | (0b001 << 2) | 0b11);

    writeReg(MAX30102_REG_LED1_PA, MAX30102_LED_CURRENT);
    writeReg(MAX30102_REG_LED2_PA, MAX30102_LED_CURRENT);

    // PPG_RDY interrupt enabled so the INT pin carries a usable signal for a future
    // FIFO-driven path. Nothing reads it today -- the DSP task polls the pointers at
    // its 25 Hz decimation boundary, which costs one 3-byte I2C read per 40 ms.
    writeReg(MAX30102_REG_INT_ENABLE_1, 1 << 6);
    writeReg(MAX30102_REG_INT_ENABLE_2, 0x00);

    clearFifo();
    ovfRegister = 0;
    rateCount = 0;
    rateStartMs = millis();
    ok = true;
    return true;
  }

  void clearFifo() {
    writeReg(MAX30102_REG_FIFO_WR_PTR, 0);
    writeReg(MAX30102_REG_OVF_COUNTER, 0);
    writeReg(MAX30102_REG_FIFO_RD_PTR, 0);
  }

  // Drains up to maxSamples entries into ir[] (and red[], if non-null) and returns how
  // many were read. Returns 0 when the FIFO is empty, which at a 25 Hz drain against a
  // 25 Hz producer happens routinely -- it is not an error.
  uint8_t read(uint32_t *ir, uint32_t *red, uint8_t maxSamples) {
    if (!ok || maxSamples == 0) return 0;
    if (maxSamples > MAX30102_DRAIN_MAX) maxSamples = MAX30102_DRAIN_MAX;

    // PPG_RDY clears only by reading Interrupt Status 1 (0x00) -- reading FIFO_DATA
    // clears A_FULL, not PPG_RDY. Without this the INT pin latches low on the first
    // sample and never releases; confirmed on-device (GPIO10 stuck low, 0 transitions
    // over 10s, even while this function was draining the FIFO every cycle).
    readReg(MAX30102_REG_INT_STATUS_1);

    // One burst read of WR_PTR / OVF_COUNTER / RD_PTR. Verified on-device against
    // three separate single-register reads: the burst and the individual reads agree
    // exactly, so the repeated-start path here is sound.
    uint8_t ptr[3];
    if (!readRegs(MAX30102_REG_FIFO_WR_PTR, ptr, 3)) return 0;
    uint8_t wr = ptr[0] & 0x1F;
    ovfRegister = ptr[1] & 0x1F;   // telemetry only -- see the field comment
    uint8_t rd = ptr[2] & 0x1F;

    // The write and read pointers are the reliable pair. They track cleanly: at a
    // 25 Hz drain against a 25 Hz producer, avail sits at 1.
    uint8_t avail = (uint8_t)((wr - rd) & 0x1F);
    if (avail == 0) return 0;
    if (avail > maxSamples) avail = maxSamples;

    uint8_t buf[MAX30102_DRAIN_MAX * 6];
    if (!readRegs(MAX30102_REG_FIFO_DATA, buf, (size_t)avail * 6)) return 0;

    for (uint8_t i = 0; i < avail; i++) {
      const uint8_t *s = buf + (size_t)i * 6;
      // 18-bit right-aligned in 3 bytes; the top 6 bits are undefined and must be
      // masked off, not merely assumed zero.
      uint32_t r = ((uint32_t)s[0] << 16 | (uint32_t)s[1] << 8 | s[2]) & 0x0003FFFF;
      uint32_t j = ((uint32_t)s[3] << 16 | (uint32_t)s[4] << 8 | s[5]) & 0x0003FFFF;
      if (red) red[i] = r;
      ir[i] = j;
    }
    rateCount += avail;
    return avail;
  }

  // Effective FIFO sample rate over the window since the last call, in Hz, and then
  // opens a new window. Returns 0 if the window is too short to mean anything.
  //
  // This carries two jobs, because OVF_COUNTER cannot do the second one here:
  //
  //  1. The oscillator check from the top of the file. It should read 25.0; a steady
  //     reading elsewhere scales every reported BPM by the same factor.
  //  2. Sample-loss detection. The FIFO is 32 deep and `avail` is computed as a 5-bit
  //     difference, so a completely full FIFO is indistinguishable from an empty one
  //     (both give 0) -- the classic wrap ambiguity that OVF_COUNTER exists to
  //     resolve, and which this part's OVF does not actually resolve. Losing samples
  //     needs a stall of 32/25 = 1.3 s, and that shows up here as a rate dip. It is a
  //     coarser detector than a working overflow counter, and it is the honest one.
  //
  // Measured on-device: 24.9-25.6 Hz across a 10 s window.
  float measuredHz(uint32_t nowMs) {
    uint32_t span = nowMs - rateStartMs;
    if (span < 1000) return 0.0f;
    float hz = 1000.0f * (float)rateCount / (float)span;
    rateCount = 0;
    rateStartMs = nowMs;
    return hz;
  }

  // --- register access ------------------------------------------------------
  void writeReg(uint8_t reg, uint8_t value) {
    wire->beginTransmission(addr);
    wire->write(reg);
    wire->write(value);
    wire->endTransmission();
  }

  uint8_t readReg(uint8_t reg) {
    uint8_t v = 0;
    readRegs(reg, &v, 1);
    return v;
  }

  bool readRegs(uint8_t reg, uint8_t *out, size_t n) {
    wire->beginTransmission(addr);
    wire->write(reg);
    if (wire->endTransmission(false) != 0) return false;   // repeated start
    if (wire->requestFrom((int)addr, (int)n) != (int)n) return false;
    for (size_t i = 0; i < n; i++) out[i] = (uint8_t)wire->read();
    return true;
  }
};
