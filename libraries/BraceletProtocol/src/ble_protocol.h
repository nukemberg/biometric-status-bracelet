/*
 * BraceletProtocol - BLE wire format for the biometric status bracelet.
 *
 * Shared by three independent implementations: the firmware (C++), the CLI
 * (Python/bleak) and the web app (JavaScript). A layout disagreement between them
 * does not raise an error -- it silently decodes to plausible-looking wrong numbers,
 * which is the worst failure mode in this project. Two things guard against it:
 *
 *   1. Every packet carries BLE_PROTOCOL_VERSION as its first byte, so a mismatch
 *      fails loudly instead of quietly.
 *   2. tools/ble_packet_test.cpp asserts this exact byte layout against golden
 *      fixtures, so reordering or resizing any field breaks the build.
 *
 * Fields are written at explicit offsets, little-endian, rather than by memcpy of a
 * packed struct: struct layout depends on compiler, architecture and packing pragmas,
 * and none of those are the same across an Xtensa firmware, a host test and a browser.
 *
 * Depends only on <stdint.h> and <stddef.h>. No Arduino, no NimBLE. See docs/DESIGN.md
 * section 6.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

// Bump on ANY change to a layout below. Consumers must reject packets they do not
// recognise rather than guess.
#define BLE_PROTOCOL_VERSION 1

// ---------------------------------------------------------------------------
// Service and characteristic UUIDs
// ---------------------------------------------------------------------------
#define BLE_SVC_BRACELET   "a1b20001-5e8f-4d7a-9c31-0f2e6d4b8a70"
#define BLE_CHR_VITALS     "a1b20002-5e8f-4d7a-9c31-0f2e6d4b8a70"
#define BLE_CHR_SIGNALS    "a1b20003-5e8f-4d7a-9c31-0f2e6d4b8a70"
#define BLE_CHR_SPECTRUM   "a1b20004-5e8f-4d7a-9c31-0f2e6d4b8a70"
#define BLE_CHR_CONTROL    "a1b20005-5e8f-4d7a-9c31-0f2e6d4b8a70"
#define BLE_CHR_CONFIG     "a1b20006-5e8f-4d7a-9c31-0f2e6d4b8a70"
#define BLE_CHR_INFO       "a1b20007-5e8f-4d7a-9c31-0f2e6d4b8a70"

// ---------------------------------------------------------------------------
// Packet sizes
// ---------------------------------------------------------------------------
#define BLE_SIGNALS_BATCH  5     // 25 Hz samples per notification -> 5 Hz packets
#define BLE_SPECTRUM_BINS  48    // must match N_BINS in BraceletDSP

#define BLE_VITALS_LEN     18
#define BLE_SIGNALS_LEN    (6 + BLE_SIGNALS_BATCH * 8)   // 46
#define BLE_SPECTRUM_LEN   (8 + BLE_SPECTRUM_BINS)       // 56
#define BLE_CONTROL_MAXLEN 4
#define BLE_CONFIG_WRITE_LEN 5
// Read returns every tunable as CFG_PARAM_COUNT records of [paramId u8][f32], in
// ascending paramId order. Same record shape as the write, so a client shares one
// decoder between the two directions. Fits a single 247-byte ATT MTU with room to
// spare, so a read is never fragmented into packets a decoder would reject.
#define BLE_CONFIG_READ_LEN   (CFG_PARAM_COUNT * BLE_CONFIG_WRITE_LEN)   // 50

// Vitals flag bits
#define VF_WORN            0x01
#define VF_PULSE_TRUSTED   0x02
#define VF_STROBE          0x04
#define VF_MODE_MASK       0x18   // 2 bits, shifted 3
#define VF_MODE_SHIFT      3

// Control commands
enum BleCommand : uint8_t {
  CMD_SET_MODE       = 0x01,   // arg: mode 0..2
  CMD_SET_BRIGHTNESS = 0x02,   // arg: 0..255
  CMD_RECALIBRATE_GSR= 0x03,   // no arg
  CMD_SET_STREAMS    = 0x04,   // arg: bitmask below
  CMD_RESET_BANK     = 0x05,   // no arg
  CMD_RESET_CONFIG   = 0x06,   // no arg -- restore compiled defaults
};

// Stream enable bitmask for CMD_SET_STREAMS. Vitals is always on.
#define STREAM_SIGNALS     0x01
#define STREAM_SPECTRUM    0x02

// Config parameter ids. Append only -- these are wire values.
enum BleConfigParam : uint8_t {
  CFG_HUE_BPM_LO   = 0x01,
  CFG_HUE_BPM_HI   = 0x02,
  CFG_HUE_AT_LO    = 0x03,
  CFG_HUE_AT_HI    = 0x04,
  CFG_PI_TRUST_MIN = 0x05,
  CFG_GSR_WORN_MIN = 0x06,
  CFG_GSR_WORN_MAX = 0x07,
  CFG_CONF_GATE    = 0x08,
  CFG_CONF_REF     = 0x09,
  CFG_SLEW_BPM_S   = 0x0A,
  CFG_BRIGHTNESS   = 0x0B,
  CFG_PARAM_COUNT  = 0x0B,
};

// ---------------------------------------------------------------------------
// Little-endian primitives
// ---------------------------------------------------------------------------
static inline void bleputU8(uint8_t *b, size_t o, uint8_t v) { b[o] = v; }
static inline void bleputU16(uint8_t *b, size_t o, uint16_t v) {
  b[o] = (uint8_t)(v & 0xFF);
  b[o + 1] = (uint8_t)(v >> 8);
}
static inline void bleputI16(uint8_t *b, size_t o, int16_t v) {
  bleputU16(b, o, (uint16_t)v);
}
static inline void bleputU32(uint8_t *b, size_t o, uint32_t v) {
  b[o] = (uint8_t)(v & 0xFF);
  b[o + 1] = (uint8_t)((v >> 8) & 0xFF);
  b[o + 2] = (uint8_t)((v >> 16) & 0xFF);
  b[o + 3] = (uint8_t)((v >> 24) & 0xFF);
}
static inline uint8_t blegetU8(const uint8_t *b, size_t o) { return b[o]; }
static inline uint16_t blegetU16(const uint8_t *b, size_t o) {
  return (uint16_t)(b[o] | ((uint16_t)b[o + 1] << 8));
}
static inline int16_t blegetI16(const uint8_t *b, size_t o) {
  return (int16_t)blegetU16(b, o);
}
static inline uint32_t blegetU32(const uint8_t *b, size_t o) {
  return (uint32_t)b[o] | ((uint32_t)b[o + 1] << 8) |
         ((uint32_t)b[o + 2] << 16) | ((uint32_t)b[o + 3] << 24);
}

// Clamp before narrowing, so an out-of-range value saturates visibly instead of
// wrapping into a plausible wrong number.
static inline uint16_t bleScaleU16(float v, float scale) {
  float s = v * scale;
  if (s < 0.0f) return 0;
  if (s > 65535.0f) return 65535;
  return (uint16_t)(s + 0.5f);
}
static inline int16_t bleScaleI16(float v, float scale) {
  float s = v * scale;
  if (s < -32768.0f) return -32768;
  if (s > 32767.0f) return 32767;
  return (int16_t)(s >= 0.0f ? s + 0.5f : s - 0.5f);
}
static inline uint8_t bleUnitToU8(float v) {
  if (v <= 0.0f) return 0;
  if (v >= 1.0f) return 255;
  return (uint8_t)(v * 255.0f + 0.5f);
}

// ---------------------------------------------------------------------------
// Vitals -- 18 bytes, notified at 4 Hz
//
//  0  u8   protocol version
//  1  u8   flags (worn, pulseTrusted, strobe, mode<<3)
//  2  u16  bpm x10
//  4  u8   confidence 0..255
//  5  u8   arousal 0..255
//  6  u16  perfusion percent x100
//  8  u16  gsr raw
// 10  u16  pulse raw
// 12  i16  temperature C x100
// 14  u16  gsr tonic
// 16  u8   brightness
// 17  u8   reserved
// ---------------------------------------------------------------------------
struct BleVitals {
  float bpm = 0.0f;
  float confidence = 0.0f;
  float arousal = 0.0f;
  float perfusion = 0.0f;
  float tempC = 0.0f;
  float gsrTonic = 0.0f;
  uint16_t gsrRaw = 0;
  uint16_t pulseRaw = 0;
  uint8_t brightness = 0;
  uint8_t mode = 0;
  bool worn = false;
  bool pulseTrusted = false;
  bool strobe = false;
};

static inline size_t blePackVitals(uint8_t *b, const BleVitals &v) {
  uint8_t flags = 0;
  if (v.worn) flags |= VF_WORN;
  if (v.pulseTrusted) flags |= VF_PULSE_TRUSTED;
  if (v.strobe) flags |= VF_STROBE;
  flags |= (uint8_t)((v.mode << VF_MODE_SHIFT) & VF_MODE_MASK);

  bleputU8(b, 0, BLE_PROTOCOL_VERSION);
  bleputU8(b, 1, flags);
  bleputU16(b, 2, bleScaleU16(v.bpm, 10.0f));
  bleputU8(b, 4, bleUnitToU8(v.confidence));
  bleputU8(b, 5, bleUnitToU8(v.arousal));
  bleputU16(b, 6, bleScaleU16(v.perfusion, 100.0f));
  bleputU16(b, 8, v.gsrRaw);
  bleputU16(b, 10, v.pulseRaw);
  bleputI16(b, 12, bleScaleI16(v.tempC, 100.0f));
  bleputU16(b, 14, bleScaleU16(v.gsrTonic, 1.0f));
  bleputU8(b, 16, v.brightness);
  bleputU8(b, 17, 0);
  return BLE_VITALS_LEN;
}

static inline bool bleUnpackVitals(const uint8_t *b, size_t len, BleVitals &v) {
  if (len < BLE_VITALS_LEN) return false;
  if (blegetU8(b, 0) != BLE_PROTOCOL_VERSION) return false;
  uint8_t flags = blegetU8(b, 1);
  v.worn = (flags & VF_WORN) != 0;
  v.pulseTrusted = (flags & VF_PULSE_TRUSTED) != 0;
  v.strobe = (flags & VF_STROBE) != 0;
  v.mode = (uint8_t)((flags & VF_MODE_MASK) >> VF_MODE_SHIFT);
  v.bpm = blegetU16(b, 2) / 10.0f;
  v.confidence = blegetU8(b, 4) / 255.0f;
  v.arousal = blegetU8(b, 5) / 255.0f;
  v.perfusion = blegetU16(b, 6) / 100.0f;
  v.gsrRaw = blegetU16(b, 8);
  v.pulseRaw = blegetU16(b, 10);
  v.tempC = blegetI16(b, 12) / 100.0f;
  v.gsrTonic = (float)blegetU16(b, 14);
  v.brightness = blegetU8(b, 16);
  return true;
}

// ---------------------------------------------------------------------------
// Signals -- 46 bytes, 5 batched 25 Hz samples notified at 5 Hz
//
//  0  u8   protocol version
//  1  u8   sample count
//  2  u32  timestamp_ms of the first sample
//  6  then count * 8 bytes:
//         +0 i16 pulse filtered x10
//         +2 i16 gsr phasic x10
//         +4 u16 pulse raw
//         +6 u16 gsr raw
// ---------------------------------------------------------------------------
struct BleSignalSample {
  float pulseFiltered = 0.0f;
  float gsrPhasic = 0.0f;
  uint16_t pulseRaw = 0;
  uint16_t gsrRaw = 0;
};

static inline size_t blePackSignals(uint8_t *b, uint32_t firstTimestampMs,
                                    const BleSignalSample *s, uint8_t count) {
  if (count > BLE_SIGNALS_BATCH) count = BLE_SIGNALS_BATCH;
  bleputU8(b, 0, BLE_PROTOCOL_VERSION);
  bleputU8(b, 1, count);
  bleputU32(b, 2, firstTimestampMs);
  for (uint8_t i = 0; i < count; i++) {
    size_t o = 6 + (size_t)i * 8;
    bleputI16(b, o + 0, bleScaleI16(s[i].pulseFiltered, 10.0f));
    bleputI16(b, o + 2, bleScaleI16(s[i].gsrPhasic, 10.0f));
    bleputU16(b, o + 4, s[i].pulseRaw);
    bleputU16(b, o + 6, s[i].gsrRaw);
  }
  return 6 + (size_t)count * 8;
}

static inline bool bleUnpackSignals(const uint8_t *b, size_t len,
                                    uint32_t &firstTimestampMs,
                                    BleSignalSample *s, uint8_t &count) {
  if (len < 6) return false;
  if (blegetU8(b, 0) != BLE_PROTOCOL_VERSION) return false;
  count = blegetU8(b, 1);
  if (count > BLE_SIGNALS_BATCH) return false;
  if (len < 6 + (size_t)count * 8) return false;
  firstTimestampMs = blegetU32(b, 2);
  for (uint8_t i = 0; i < count; i++) {
    size_t o = 6 + (size_t)i * 8;
    s[i].pulseFiltered = blegetI16(b, o + 0) / 10.0f;
    s[i].gsrPhasic = blegetI16(b, o + 2) / 10.0f;
    s[i].pulseRaw = blegetU16(b, o + 4);
    s[i].gsrRaw = blegetU16(b, o + 6);
  }
  return true;
}

// ---------------------------------------------------------------------------
// Spectrum -- 56 bytes, 48 resonator bins notified at 1 Hz
//
//  0  u8   protocol version
//  1  u8   bin count
//  2  u8   peak bin index
//  3  u8   reserved
//  4  u16  bpm of bin 0, x10
//  6  u16  bpm of the last bin, x10
//  8  u8[] per-bin power, normalised so the peak is 255
//
// Power is normalised to the peak rather than sent as an absolute: the useful
// question is which bins compete with the winner (harmonic capture, DESIGN.md 2.3),
// not the absolute magnitude, which spans orders of magnitude with contact quality.
// ---------------------------------------------------------------------------
static inline size_t blePackSpectrum(uint8_t *b, const float *binPower,
                                     uint8_t bins, uint8_t peakBin,
                                     float bpmLo, float bpmHi) {
  if (bins > BLE_SPECTRUM_BINS) bins = BLE_SPECTRUM_BINS;
  bleputU8(b, 0, BLE_PROTOCOL_VERSION);
  bleputU8(b, 1, bins);
  bleputU8(b, 2, peakBin);
  bleputU8(b, 3, 0);
  bleputU16(b, 4, bleScaleU16(bpmLo, 10.0f));
  bleputU16(b, 6, bleScaleU16(bpmHi, 10.0f));

  float peak = 0.0f;
  for (uint8_t i = 0; i < bins; i++) {
    if (binPower[i] > peak) peak = binPower[i];
  }
  for (uint8_t i = 0; i < bins; i++) {
    uint8_t v = 0;
    if (peak > 0.0f) {
      float r = binPower[i] / peak;
      v = bleUnitToU8(r);
    }
    bleputU8(b, 8 + i, v);
  }
  return 8 + (size_t)bins;
}

static inline bool bleUnpackSpectrum(const uint8_t *b, size_t len, uint8_t *outBins,
                                     uint8_t &bins, uint8_t &peakBin,
                                     float &bpmLo, float &bpmHi) {
  if (len < 8) return false;
  if (blegetU8(b, 0) != BLE_PROTOCOL_VERSION) return false;
  bins = blegetU8(b, 1);
  if (bins > BLE_SPECTRUM_BINS) return false;
  if (len < 8 + (size_t)bins) return false;
  peakBin = blegetU8(b, 2);
  bpmLo = blegetU16(b, 4) / 10.0f;
  bpmHi = blegetU16(b, 6) / 10.0f;
  for (uint8_t i = 0; i < bins; i++) outBins[i] = blegetU8(b, 8 + i);
  return true;
}

// ---------------------------------------------------------------------------
// Config write -- 5 bytes: [version][paramId][f32 value]
// Float is sent as raw IEEE-754 little-endian bits.
// ---------------------------------------------------------------------------
static inline uint32_t bleFloatBits(float f) {
  union { float f; uint32_t u; } c;
  c.f = f;
  return c.u;
}
static inline float bleBitsFloat(uint32_t u) {
  union { float f; uint32_t u; } c;
  c.u = u;
  return c.f;
}

static inline size_t blePackConfigWrite(uint8_t *b, uint8_t paramId, float value) {
  bleputU8(b, 0, paramId);
  bleputU32(b, 1, bleFloatBits(value));
  return BLE_CONFIG_WRITE_LEN;
}

static inline bool bleUnpackConfigWrite(const uint8_t *b, size_t len,
                                        uint8_t &paramId, float &value) {
  if (len < BLE_CONFIG_WRITE_LEN) return false;
  paramId = blegetU8(b, 0);
  if (paramId == 0 || paramId > CFG_PARAM_COUNT) return false;
  value = bleBitsFloat(blegetU32(b, 1));
  return true;
}

// ---------------------------------------------------------------------------
// Config read -- CFG_PARAM_COUNT records of [paramId u8][f32], one per tunable.
//
// The caller passes the values in paramId order: values[0] is CFG_HUE_BPM_LO,
// values[CFG_PARAM_COUNT-1] is CFG_SLEW_BPM_S. The packer writes the paramId for
// each record itself, so the wire format is self-describing and a decoder can
// cross-check that record i really does carry id i+1.
//
// ble_protocol.h stays free of BraceletConfig (that lives in dsp.h) by design --
// the mapping between a paramId and a BraceletConfig field is owned by the
// firmware's config_store, which is the one place that already knows it.
// ---------------------------------------------------------------------------
static inline size_t blePackConfigRead(uint8_t *b, const float *values) {
  for (uint8_t i = 0; i < CFG_PARAM_COUNT; i++) {
    size_t o = (size_t)i * BLE_CONFIG_WRITE_LEN;
    bleputU8(b, o, (uint8_t)(i + 1));
    bleputU32(b, o + 1, bleFloatBits(values[i]));
  }
  return BLE_CONFIG_READ_LEN;
}

static inline bool bleUnpackConfigRead(const uint8_t *b, size_t len, float *values) {
  if (len < BLE_CONFIG_READ_LEN) return false;
  for (uint8_t i = 0; i < CFG_PARAM_COUNT; i++) {
    size_t o = (size_t)i * BLE_CONFIG_WRITE_LEN;
    // Records must arrive in ascending paramId order. A reorder would otherwise
    // silently write the wrong tunable into the wrong slot -- exactly the
    // plausible-wrong-number failure this library exists to prevent.
    if (blegetU8(b, o) != (uint8_t)(i + 1)) return false;
    values[i] = bleBitsFloat(blegetU32(b, o + 1));
  }
  return true;
}
