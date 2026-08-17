/*
 * Byte-layout tests for libraries/BraceletProtocol.
 *
 * Three independent implementations decode these packets (firmware, CLI, web app),
 * and a layout disagreement between them produces plausible-looking wrong numbers
 * rather than an error. So this asserts the layout two ways:
 *
 *   - Round-trip: pack then unpack recovers the input within quantisation.
 *   - Golden bytes: the packed output equals a hardcoded byte vector.
 *
 * The golden fixtures are the ones that matter. A round-trip test passes happily if
 * two fields are swapped, because pack and unpack are swapped in the same way. Only a
 * fixed expected byte string catches that -- and it doubles as the specification the
 * Python and JavaScript decoders are written against.
 *
 * Build & run: tools/ble_packet_test.sh
 */

#include <cmath>
#include <cstdio>
#include <cstring>

#include <ble_protocol.h>

// The test is allowed to depend on both libraries even though neither depends on the
// other, which makes it the right place to enforce that they agree.
#include <dsp.h>

static_assert(BLE_SPECTRUM_BINS == N_BINS,
              "BLE_SPECTRUM_BINS must equal N_BINS in BraceletDSP. If the resonator "
              "bank is resized, the spectrum packet must follow or the web app will "
              "chart a truncated or over-read spectrum with no error anywhere.");

static int failures = 0;

static void fail(const char *what, const char *detail) {
  printf("  FAIL %s: %s\n", what, detail);
  failures++;
}

static void checkNear(const char *what, float got, float want, float tol) {
  if (std::fabs(got - want) > tol) {
    char buf[128];
    snprintf(buf, sizeof buf, "got %.4f want %.4f (tol %.4f)", got, want, tol);
    fail(what, buf);
  }
}

static void checkEq(const char *what, long got, long want) {
  if (got != want) {
    char buf[128];
    snprintf(buf, sizeof buf, "got %ld want %ld", got, want);
    fail(what, buf);
  }
}

static void checkBytes(const char *what, const uint8_t *got, const uint8_t *want,
                       size_t n) {
  for (size_t i = 0; i < n; i++) {
    if (got[i] != want[i]) {
      char buf[256];
      snprintf(buf, sizeof buf, "byte %zu differs: got 0x%02X want 0x%02X\n"
               "       this means the wire layout changed -- bump BLE_PROTOCOL_VERSION\n"
               "       and update the CLI and web app decoders to match",
               i, got[i], want[i]);
      fail(what, buf);
      // Dump both so the diff is obvious.
      printf("       got: ");
      for (size_t j = 0; j < n; j++) printf("%02X ", got[j]);
      printf("\n       want:");
      for (size_t j = 0; j < n; j++) printf("%02X ", want[j]);
      printf("\n");
      return;
    }
  }
}

// ---------------------------------------------------------------------------
static void testVitals() {
  printf("vitals\n");

  BleVitals v;
  v.bpm = 64.3f;
  v.confidence = 0.25f;
  v.arousal = 0.5f;
  v.perfusion = 1.60f;
  v.tempC = 26.25f;
  v.gsrTonic = 1435.0f;
  v.gsrRaw = 1285;
  v.pulseRaw = 84213;   // MAX30102 IR DC, mid-range for good wrist contact
  v.brightness = 60;
  v.mode = 2;
  v.worn = true;
  v.pulseTrusted = true;
  v.strobe = false;

  uint8_t buf[BLE_VITALS_LEN];
  size_t n = blePackVitals(buf, v);
  checkEq("vitals length", (long)n, BLE_VITALS_LEN);

  // flags: worn(0x01) | trusted(0x02) | mode 2 << 3 (0x10) = 0x13
  // bpm 643 = 0x0283 ; perfusion 160 = 0x00A0 ; gsr 1285 = 0x0505
  // pulse 84213 = 0x000148F5 ; temp 2625 = 0x0A41 ; tonic 1435 = 0x059B
  const uint8_t want[BLE_VITALS_LEN] = {
      0x02,        // version
      0x13,        // flags
      0x83, 0x02,  // bpm x10 = 643
      0x40,        // confidence 0.25 -> 64
      0x80,        // arousal 0.5 -> 128
      0xA0, 0x00,  // perfusion x100 = 160
      0x05, 0x05,  // gsr raw 1285
      0xF5, 0x48, 0x01, 0x00,  // pulse raw 84213 (u32 since v2)
      0x41, 0x0A,  // temp x100 = 2625
      0x9B, 0x05,  // gsr tonic 1435
      0x3C,        // brightness 60
      0x00,        // reserved
  };
  checkBytes("vitals golden bytes", buf, want, BLE_VITALS_LEN);

  BleVitals r;
  if (!bleUnpackVitals(buf, n, r)) {
    fail("vitals unpack", "returned false");
    return;
  }
  checkNear("vitals bpm", r.bpm, v.bpm, 0.05f);
  checkNear("vitals confidence", r.confidence, v.confidence, 0.01f);
  checkNear("vitals arousal", r.arousal, v.arousal, 0.01f);
  checkNear("vitals perfusion", r.perfusion, v.perfusion, 0.01f);
  checkNear("vitals tempC", r.tempC, v.tempC, 0.01f);
  checkNear("vitals gsrTonic", r.gsrTonic, v.gsrTonic, 1.0f);
  checkEq("vitals gsrRaw", r.gsrRaw, v.gsrRaw);
  checkEq("vitals pulseRaw", (long)r.pulseRaw, (long)v.pulseRaw);
  checkEq("vitals brightness", r.brightness, v.brightness);
  checkEq("vitals mode", r.mode, v.mode);
  checkEq("vitals worn", r.worn, 1);
  checkEq("vitals pulseTrusted", r.pulseTrusted, 1);
  checkEq("vitals strobe", r.strobe, 0);
}

static void testVitalsEdges() {
  printf("vitals edge cases\n");

  BleVitals v;
  v.tempC = -12.5f;  // sub-zero ambient must survive as a signed value
  uint8_t buf[BLE_VITALS_LEN];
  blePackVitals(buf, v);
  BleVitals r;
  bleUnpackVitals(buf, BLE_VITALS_LEN, r);
  checkNear("negative temperature", r.tempC, -12.5f, 0.01f);

  // Out-of-range values must saturate, never wrap into a plausible wrong number.
  BleVitals big;
  big.bpm = 9999.0f;
  big.perfusion = 5000.0f;
  blePackVitals(buf, big);
  bleUnpackVitals(buf, BLE_VITALS_LEN, r);
  checkNear("bpm saturates", r.bpm, 6553.5f, 0.1f);
  checkNear("perfusion saturates", r.perfusion, 655.35f, 0.1f);

  // The whole reason for protocol v2: an 18-bit IR count must survive intact. At the
  // v1 u16 width this wrapped to 65535 -> 262143 & 0xFFFF = 65535, a number that looks
  // like a saturated but believable reading.
  BleVitals ir;
  ir.pulseRaw = 262143;   // 2^18 - 1, MAX30102 full scale
  blePackVitals(buf, ir);
  bleUnpackVitals(buf, BLE_VITALS_LEN, r);
  checkEq("18-bit IR full scale survives", (long)r.pulseRaw, 262143);

  BleVitals neg;
  neg.bpm = -50.0f;
  blePackVitals(buf, neg);
  bleUnpackVitals(buf, BLE_VITALS_LEN, r);
  checkNear("negative bpm clamps to zero", r.bpm, 0.0f, 0.01f);

  // A packet from a different protocol version must be rejected, not guessed at.
  blePackVitals(buf, v);
  buf[0] = BLE_PROTOCOL_VERSION + 1;
  if (bleUnpackVitals(buf, BLE_VITALS_LEN, r)) {
    fail("version mismatch", "accepted a packet with the wrong version byte");
  }

  // Truncated packets must be rejected.
  blePackVitals(buf, v);
  if (bleUnpackVitals(buf, BLE_VITALS_LEN - 1, r)) {
    fail("short packet", "accepted a truncated vitals packet");
  }
}

static void testSignals() {
  printf("signals\n");

  // Magnitudes chosen for the MAX30102 scale, not the old analog one: cardiac AC in
  // the hundreds-to-thousands of IR counts, on a DC level in the tens of thousands.
  // A -1234.5 filtered value at x10 scaling is 12345, well past what the v1 i16 field
  // could carry.
  BleSignalSample s[BLE_SIGNALS_BATCH];
  for (int i = 0; i < BLE_SIGNALS_BATCH; i++) {
    s[i].pulseFiltered = -1234.5f + 500.0f * (float)i;
    s[i].gsrPhasic = 10.0f * (float)i;
    s[i].pulseRaw = (uint32_t)(84200 + i);
    s[i].gsrRaw = (uint16_t)(1300 + i);
  }

  uint8_t buf[BLE_SIGNALS_LEN];
  size_t n = blePackSignals(buf, 123456u, s, BLE_SIGNALS_BATCH);
  checkEq("signals length", (long)n, BLE_SIGNALS_LEN);

  // header: version, count 5, timestamp 123456 = 0x0001E240
  // sample 0: pulse -1234.5*10 = -12345 = 0xFFFFCFC7 ; raw 84200 = 0x000148E8 ;
  //           gsr phasic 0 ; gsr raw 1300 = 0x0514
  const uint8_t wantHead[18] = {
      0x02, 0x05,              // version, count
      0x40, 0xE2, 0x01, 0x00,  // timestamp 123456
      0xC7, 0xCF, 0xFF, 0xFF,  // pulse filtered x10 = -12345 (i32 since v2)
      0xE8, 0x48, 0x01, 0x00,  // pulse raw 84200 (u32 since v2)
      0x00, 0x00,              // gsr phasic x10 = 0
      0x14, 0x05,              // gsr raw 1300
  };
  checkBytes("signals golden header", buf, wantHead, sizeof wantHead);

  uint32_t ts = 0;
  uint8_t count = 0;
  BleSignalSample r[BLE_SIGNALS_BATCH];
  if (!bleUnpackSignals(buf, n, ts, r, count)) {
    fail("signals unpack", "returned false");
    return;
  }
  checkEq("signals timestamp", (long)ts, 123456);
  checkEq("signals count", count, BLE_SIGNALS_BATCH);
  for (int i = 0; i < BLE_SIGNALS_BATCH; i++) {
    checkNear("signals pulseFiltered", r[i].pulseFiltered, s[i].pulseFiltered, 0.05f);
    checkNear("signals gsrPhasic", r[i].gsrPhasic, s[i].gsrPhasic, 0.05f);
    checkEq("signals pulseRaw", (long)r[i].pulseRaw, (long)s[i].pulseRaw);
    checkEq("signals gsrRaw", r[i].gsrRaw, s[i].gsrRaw);
  }

  // A claimed count larger than the batch must be rejected rather than overrun.
  buf[1] = BLE_SIGNALS_BATCH + 1;
  if (bleUnpackSignals(buf, n, ts, r, count)) {
    fail("signals overlong count", "accepted a count beyond the batch size");
  }
}

static void testSpectrum() {
  printf("spectrum\n");

  float power[BLE_SPECTRUM_BINS];
  for (int i = 0; i < BLE_SPECTRUM_BINS; i++) power[i] = 0.0f;
  power[10] = 1.0f;   // fundamental
  power[21] = 0.8f;   // 2nd harmonic, the competition we want to see
  power[5] = 0.25f;

  uint8_t buf[BLE_SPECTRUM_LEN];
  size_t n = blePackSpectrum(buf, power, BLE_SPECTRUM_BINS, 10, 40.0f, 190.0f);
  checkEq("spectrum length", (long)n, BLE_SPECTRUM_LEN);

  const uint8_t wantHead[8] = {
      0x02, 0x30,  // version, 48 bins
      0x0A, 0x00,  // peak bin 10, reserved
      0x90, 0x01,  // bpmLo x10 = 400
      0x6C, 0x07,  // bpmHi x10 = 1900
  };
  checkBytes("spectrum golden header", buf, wantHead, sizeof wantHead);

  uint8_t bins = 0, peak = 0, out[BLE_SPECTRUM_BINS];
  float lo = 0, hi = 0;
  if (!bleUnpackSpectrum(buf, n, out, bins, peak, lo, hi)) {
    fail("spectrum unpack", "returned false");
    return;
  }
  checkEq("spectrum bins", bins, BLE_SPECTRUM_BINS);
  checkEq("spectrum peak bin", peak, 10);
  checkNear("spectrum bpmLo", lo, 40.0f, 0.05f);
  checkNear("spectrum bpmHi", hi, 190.0f, 0.05f);
  checkEq("spectrum peak normalised to full scale", out[10], 255);
  checkEq("spectrum harmonic bin", out[21], 204);   // 0.8 * 255
  checkEq("spectrum empty bin", out[0], 0);

  // An all-zero bank must not divide by zero.
  for (int i = 0; i < BLE_SPECTRUM_BINS; i++) power[i] = 0.0f;
  blePackSpectrum(buf, power, BLE_SPECTRUM_BINS, 0, 40.0f, 190.0f);
  bleUnpackSpectrum(buf, BLE_SPECTRUM_LEN, out, bins, peak, lo, hi);
  checkEq("spectrum all-zero bank", out[0], 0);
}

static void testConfig() {
  printf("config\n");

  uint8_t buf[BLE_CONFIG_WRITE_LEN];
  size_t n = blePackConfigWrite(buf, CFG_PI_TRUST_MIN, 0.40f);
  checkEq("config write length", (long)n, BLE_CONFIG_WRITE_LEN);

  // 0.40f = 0x3ECCCCCD little-endian
  const uint8_t want[BLE_CONFIG_WRITE_LEN] = {0x05, 0xCD, 0xCC, 0xCC, 0x3E};
  checkBytes("config golden bytes", buf, want, BLE_CONFIG_WRITE_LEN);

  uint8_t id = 0;
  float val = 0.0f;
  if (!bleUnpackConfigWrite(buf, n, id, val)) {
    fail("config unpack", "returned false");
    return;
  }
  checkEq("config paramId", id, CFG_PI_TRUST_MIN);
  checkNear("config value", val, 0.40f, 1e-6f);

  // Unknown and zero parameter ids must be rejected, not applied to something else.
  buf[0] = 0xFF;
  if (bleUnpackConfigWrite(buf, n, id, val)) {
    fail("config unknown param", "accepted an out-of-range parameter id");
  }
  buf[0] = 0x00;
  if (bleUnpackConfigWrite(buf, n, id, val)) {
    fail("config zero param", "accepted parameter id 0");
  }
}

static void testConfigRead() {
  printf("config read\n");

  // Values chosen so each float has a distinctive bit pattern, so a field swapped
  // with its neighbour produces an obviously-wrong byte rather than a near-miss.
  // 1.0f = 0x3F800000, 2.0f = 0x40000000, 0.25f = 0x3E800000, -1.0f = 0xBF800000.
  float values[CFG_PARAM_COUNT] = {
    1.0f, 2.0f, 3.0f, 4.0f, 0.25f,   // hue anchors + piTrustMin
    100.0f, 4000.0f, 0.5f, 0.9f, 8.0f,  // gsr/conf/slew
  };

  uint8_t buf[BLE_CONFIG_READ_LEN];
  size_t n = blePackConfigRead(buf, values);
  checkEq("config read length", (long)n, BLE_CONFIG_READ_LEN);
  checkEq("config read fits one 247-byte MTU", (long)BLE_CONFIG_READ_LEN <= 244, 1);

  // Record 0: paramId 1, value 1.0f = 0x3F800000
  // Record 4: paramId 5, value 0.25f = 0x3E800000
  const uint8_t wantRec0[5] = {0x01, 0x00, 0x00, 0x80, 0x3F};
  const uint8_t wantRec4[5] = {0x05, 0x00, 0x00, 0x80, 0x3E};
  checkBytes("config read record 0", buf, wantRec0, 5);
  checkBytes("config read record 4", buf + 4 * 5, wantRec4, 5);

  float r[CFG_PARAM_COUNT];
  if (!bleUnpackConfigRead(buf, n, r)) {
    fail("config read unpack", "returned false");
    return;
  }
  for (uint8_t i = 0; i < CFG_PARAM_COUNT; i++) {
    checkNear("config read value", r[i], values[i], 1e-6f);
  }

  // A reordered record must be rejected rather than written into the wrong slot.
  blePackConfigRead(buf, values);
  buf[0] = 0x02;   // claims to be paramId 2 in slot 0
  if (bleUnpackConfigRead(buf, n, r)) {
    fail("config read reorder", "accepted a record whose paramId does not match its slot");
  }

  // A truncated read must be rejected.
  blePackConfigRead(buf, values);
  if (bleUnpackConfigRead(buf, BLE_CONFIG_READ_LEN - 1, r)) {
    fail("config read short", "accepted a truncated config read");
  }
}

static void testInvariants() {
  printf("invariants\n");
  // BLE_SPECTRUM_BINS == N_BINS is enforced by a static_assert at the top of this
  // file, so a resized resonator bank breaks the build rather than the charts.
  checkEq("spectrum bins match the resonator bank", (long)BLE_SPECTRUM_BINS, N_BINS);
  checkEq("spectrum fits one 247-byte MTU", (long)BLE_SPECTRUM_LEN <= 244, 1);
  checkEq("signals fits one 247-byte MTU", (long)BLE_SIGNALS_LEN <= 244, 1);
  checkEq("config read fits one 247-byte MTU", (long)BLE_CONFIG_READ_LEN <= 244, 1);
  checkEq("vitals length", (long)BLE_VITALS_LEN, 20);
  checkEq("signals length", (long)BLE_SIGNALS_LEN, 66);
  checkEq("spectrum length", (long)BLE_SPECTRUM_LEN, 56);
}

int main() {
  printf("BLE protocol v%d layout tests\n\n", BLE_PROTOCOL_VERSION);
  testVitals();
  testVitalsEdges();
  testSignals();
  testSpectrum();
  testConfig();
  testConfigRead();
  testInvariants();

  printf("\n");
  if (failures) {
    printf("FAILED: %d check(s)\n", failures);
    return 1;
  }
  printf("all checks passed\n");
  return 0;
}
