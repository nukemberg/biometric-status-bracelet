/*
 * BraceletDSP - pulse, GSR and wear detection for the biometric status bracelet.
 *
 * This header deliberately depends on nothing but <math.h> and <stdint.h>. No
 * Arduino, no FreeRTOS, no BLE. That constraint is load-bearing: it is what lets
 * tools/dsp_v2_parity.sh compile these exact trackers on a host machine and prove
 * them bit-identical to the Python reference in tools/dsp_v2_sim.py. Every bug
 * found in this pipeline so far was found that way. If Arduino or BLE types leak
 * in here, that validation path is lost.
 *
 * Rationale for the algorithms, and the measurements behind the tuning constants,
 * are in docs/DESIGN.md sections 2 and 3.
 */

#pragma once

#include <math.h>
#include <stdint.h>

// GSR front end: 500 Hz ADC decimated to 25 Hz by a 20-sample boxcar. The 40 ms
// boxcar window nulls 25/50/75 Hz exactly, killing the 50 Hz mains hum measured on the
// GSR line at ~200x its noise floor, and delivers the same sqrt(20) noise reduction
// the old 16x oversampling burst did at 1/16 the analogRead() calls.
//
// PPG does NOT go through this path any more. Since the MAX30102 replaced the analog
// pulse sensor, the PPG arrives from the sensor's own FIFO already at 25 Hz (100 Hz
// internal sampling, 4x on-chip averaging), with the part's own ambient-light
// subtraction in place of the boxcar. RAW_HZ/DECIM/RAW_PERIOD_MS now describe the GSR
// channel alone; DSP_HZ remains the rate BOTH trackers run at, which is what every
// coefficient below is derived from.
#define RAW_HZ          500
#define DECIM           20
#define DSP_HZ          (RAW_HZ / DECIM)   // 25 Hz
#define RAW_PERIOD_MS   (1000 / RAW_HZ)    // 2 ms

// ---- Pulse chain ----
#define HP_HZ           0.5f
#define LP_HZ           4.0f    // two cascaded poles; removes the 10.2 Hz tremor
                                // artifact that dominates the raw PPG spectrum
#define BPM_MIN         40.0f   // athletic resting
#define BPM_MAX         190.0f  // dancing; do not narrow this range
#define N_BINS          48
#define BANK_TAU        10.0f

// acPower (perfusion's numerator) used to share BANK_TAU with the resonator bank, but
// the two want opposite things: the bank needs a long time constant for frequency
// resolution (resolution ~1/(2*pi*tau); shortening it blurs neighboring bins together,
// which is exactly wrong when trying to separate a fundamental from its harmonic), while
// perfusion just wants to settle quickly after a contact disturbance so field
// calibration doesn't need 30-60s of stillness per capture (2026-09-02 field session).
// Split so each can be tuned independently -- verified against samples/raw_wrist_73bpm.csv
// and samples/raw_wrist_65bpm_still.csv that decoupling does not change BPM/confidence at
// all (acPower no longer feeds the bank), only how fast PerfIdx settles. 5s roughly halves
// settle time vs the old shared 10s without the twitchiness seen at 2-3s.
#define PERF_TAU        5.0f
#define RENORM_INTERVAL 256

#define SLEW_BPM_PER_S  8.0f
#define CONF_REF        0.18f   // peak/total power ratio treated as fully trusted
#define CONF_GATE       0.06f   // below this the bank is noise; freeze BPM

// ---- GSR chain ----
#define TONIC_TAU       45.0f
#define PHASIC_TAU      0.7f
#define SCR_ATTACK_TAU  0.15f
#define SCR_RELEASE_TAU 3.0f
#define SLOPE_CLAMP     60.0f   // counts/s; rejects contact/motion steps
#define RANGE_TAU       30.0f
#define RANGE_GAIN      2.5f
// Floor on the auto-range denominator, in counts/s. 0.5 was too low to survive a
// real wrist: slope is measured in counts per SECOND at DSP_HZ = 25, so a single
// ADC count of phasic wiggle between two samples is already 25 counts/s, against a
// floored denominator of RANGE_GAIN * 0.5 = 1.25. Arousal pinned at 1.0 on 12-bit
// quantisation dither alone, with raw GSR sitting quietly in a ~100-count band.
// 3.0 (denominator 7.5) keeps a genuine SCR on scale while riding above the dither.
// Provisional -- the honest value comes from a calibrated resting capture (-cal).
#define RANGE_FLOOR     3.0f
#define OUT_ATTACK_TAU  0.10f
#define OUT_RELEASE_TAU 1.50f

// ---- Wear / contact detection ----
// Without this the bank happily tracks noise and reports a confident heart rate for
// nobody. Two independent checks, because each covers the other's blind spot:
//
//  1. GSR electrodes. Skin between the pads reads 1000-1509 counts across every
//     capture taken so far (bio2/bio3/bio4 plus on-wrist checks); off the wrist it
//     sits around 2400 rather than railing. Still a clean gap, but a much smaller
//     one than the logged captures suggested.
//  2. PPG perfusion index (cardiac AC / DC) -- used to judge whether the *pulse* is
//     trustworthy, NOT whether the device is worn. See below.
//
// NOTE: everything in the perfusion table below was measured on the v1 analog PPG
// sensor and is retained as the record of why the design is shaped this way, not as
// a live calibration. The MAX30102 computes the same ratio from a different signal
// entirely -- 18-bit IR counts with a DC level in the tens of thousands, its own LED
// drive and its own ambient subtraction -- so the numbers do not carry over and
// neither does PI_TRUST_MIN. See DESIGN.md section 4.2 for the recapture procedure.
// The threshold is runtime-tunable over BLE (CFG_PI_TRUST_MIN), so correcting it
// needs no reflash.
//
// Measured perfusion() on the v1 analog path, same formula as the runtime metric:
//
//     bio4.csv  good contact, worn   median 0.678  (min 0.482)
//     bio2.log  poor contact, worn   median 0.180  (p10 0.151, p90 0.306)
//     bench     nothing attached     0.17 - 0.20
//
// Poor-but-worn and not-worn are indistinguishable on perfusion alone. No threshold
// separates them, so perfusion cannot gate wear: any value that rejects an empty
// sensor also blanks the panel on a real wearer with mediocre contact. An earlier
// PI_WORN_MIN of 0.15 was derived from a *different* formula (scipy band-pass p2p /
// mean) and sat below the unworn floor, so the PPG half of the gate never rejected
// anything.
//
// GSR does NOT gate wear (-6y4): an unworn sensor is a floating analog input, so its
// reading depends on what the leads are near rather than on contact. Two sessions on
// the same electrodes produced unworn readings of 2400 and 3530 -- not a stable
// "disconnected" signature to threshold against, however the split point is chosen.
//
// PPG IR DC gates wear instead. Off-skin, the LED sees only ambient + a few thousand
// counts of internal offset; on-skin it sees the tuned operating point (~20k-150k,
// DESIGN.md 4.2). On-skin measurement 2026-09-02: off-wrist ~2105, worn ~80000 at the
// retuned LED current -- a >35x gap, not a few-hundred-count split. Both bounds are
// runtime-configurable via irWornMin/Max.
//
// Confidence is deliberately used for neither: an unworn board was observed reporting
// confidence 0.29, as high as a good worn signal, because the bank locks onto noise
// just as happily as onto a pulse.
#define PPG_IR_WORN_MIN   10000       // below this: off skin, or LED current starved
#define PPG_IR_WORN_MAX   262143     // 18-bit full scale; clipping is still worn contact

// Above this the cardiac signal is strong enough to believe the rate. Below it the
// device still knows it is worn -- GSR says so -- but the pulse segment shows a
// searching state rather than a confident number, because at v1 signal quality the
// tracker was measured reporting 113 BPM against a true 88.
//
// UNCALIBRATED against the MAX30102. 0.40 was chosen between bio2's p90 (0.306) and
// bio4's minimum (0.482) on the analog sensor, and is carried forward only because
// some starting value is needed. It is deliberately NOT set to a guess at what the
// MAX30102 will produce: an invented number that happens to look reasonable is worse
// than a known-stale one, because it reads as calibrated. Recapture per DESIGN.md
// section 4.2 and set it over BLE, then move the default here.
#define PI_TRUST_MIN    0.40f
#define WEAR_ON_MS      2000      // contact must hold this long before we trust it
#define WEAR_OFF_MS     5000      // and drop out this long before we let go

static inline float emaAlpha(float tauSeconds) {
  return 1.0f - expf(-1.0f / (tauSeconds * (float)DSP_HZ));
}

static inline float onepoleAlpha(float cornerHz) {
  return 1.0f - expf(-2.0f * (float)M_PI * cornerHz / (float)DSP_HZ);
}

static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// ============================================================================
// LOOP SCHEDULING JITTER
// ============================================================================
// The 20-sample boxcar nulls 50 Hz only because its window is exactly 40 ms. That
// assumes the sampling loop actually runs every 2 ms. A NimBLE controller task shares
// core 0 with it and can preempt, so this measures the real period rather than
// assuming it. Baseline it before the radio exists, then compare.
//
// Pure integer bookkeeping, no Arduino: the caller supplies micros().
struct JitterMonitor {
  uint32_t last = 0;
  uint32_t minUs = 0xFFFFFFFF;
  uint32_t maxUs = 0;
  uint64_t sumUs = 0;
  uint32_t count = 0;
  uint32_t overruns = 0;      // periods more than 50% past target
  bool primed = false;

  void reset() {
    minUs = 0xFFFFFFFF; maxUs = 0; sumUs = 0; count = 0; overruns = 0;
  }

  void tick(uint32_t nowUs, uint32_t targetUs) {
    if (!primed) { last = nowUs; primed = true; return; }
    uint32_t dt = nowUs - last;      // unsigned wraparound is correct here
    last = nowUs;
    if (dt < minUs) minUs = dt;
    if (dt > maxUs) maxUs = dt;
    sumUs += dt;
    count++;
    if (dt > targetUs + targetUs / 2) overruns++;
  }

  float meanUs() const { return count ? (float)((double)sumUs / count) : 0.0f; }
};

// Initial BPM before the bank has converged. Shared by every consumer so the
// firmware and the offline reference start from the same state.
#define DEFAULT_BPM     78.0f

// ============================================================================
// RUNTIME-TUNABLE CONFIG
// ============================================================================
// Everything here can be changed after boot (currently via the button/BLE control
// path; -1jc). Deliberately excludes anything that sizes a fixed array or is baked
// into the wire protocol -- N_BINS, BPM_MIN/MAX, BANK_TAU and the sample-rate
// constants stay compile-time #defines, because BLE_SPECTRUM_BINS is static_assert'd
// equal to N_BINS and every array in PulseTracker is sized by it. Runtime-resizing
// the bank would need dynamic allocation this project has no reason to want.
//
// The #defines above remain the single source of truth for the numeric defaults, so
// existing comments that reference them by name stay accurate, and this struct is
// purely additive: nothing that already worked changes behaviour by existing.
struct BraceletConfig {
  // LED gradient anchors (main_armband.ino's pulseHueFor). Kept here rather than in
  // the sketch so the whole tunable set has one shape, even though dsp.h itself does
  // not read these.
  float hueBpmLo   = 50.0f;
  float hueBpmHi   = 190.0f;
  float hueAtLo    = 48.0f;
  float hueAtHi    = -27.0f;

  float piTrustMin  = PI_TRUST_MIN;
  float irWornMin   = (float)PPG_IR_WORN_MIN;
  float irWornMax   = (float)PPG_IR_WORN_MAX;
  float confGate    = CONF_GATE;
  float confRef     = CONF_REF;
  float slewBpmPerS = SLEW_BPM_PER_S;
  float brightness  = 60.0f;

  // One-point offset against a reference thermometer, added to the BME280 reading.
  // Compensates case self-heating from the 21 WS2812s and the radio, not sensor
  // trim error -- the factory +-0.5C spec is already smaller than that effect.
  float tempOffsetC = 0.0f;

  static BraceletConfig defaults() { return BraceletConfig(); }
};

// ============================================================================
// PPG RESONATOR BANK (rate + confidence + beat phase from one structure)
// ============================================================================
struct PulseTracker {
  float aHP, aLP, aBank, aPerf;
  float baseline, lp1, lp2, filtered;
  bool primed;

  float binBpm[N_BINS];
  float stepRe[N_BINS], stepIm[N_BINS];  // per-bin rotation e^(-j*w_k)
  float rotRe[N_BINS],  rotIm[N_BINS];   // running rotator
  float resRe[N_BINS],  resIm[N_BINS];   // resonator state
  uint32_t n;

  float acPower;                         // EMA of y^2, for the perfusion index
  float bpm, confidence, phase;

  // Runtime-tunable; see BraceletConfig. Defaulted in begin(), changed via
  // applyConfig() without needing to re-run begin() (which would also reset the
  // bank -- a config change should not throw away accumulated tracking).
  float confGate = CONF_GATE;
  float confRef = CONF_REF;
  float slewBpmPerS = SLEW_BPM_PER_S;

  void applyConfig(const BraceletConfig &cfg) {
    confGate = cfg.confGate;
    confRef = cfg.confRef;
    slewBpmPerS = cfg.slewBpmPerS;
  }

  // Index of the winning bin at the last estimate(). Published with the spectrum so a
  // client can mark the peak without re-deriving it and possibly disagreeing.
  uint8_t peakBin = 0;

  // Copies out the per-bin power. Read-only view for telemetry; the bank itself stays
  // private to update()/estimate().
  void binPowers(float *out) const {
    for (int k = 0; k < N_BINS; k++) {
      out[k] = resRe[k] * resRe[k] + resIm[k] * resIm[k];
    }
  }

  // Cardiac AC as a percentage of DC level. For a sinusoid p2p = 2*sqrt(2)*rms.
  float perfusion() const {
    if (baseline < 1.0f) return 0.0f;
    return 100.0f * 2.83f * sqrtf(acPower) / baseline;
  }

  void begin() {
    aHP = onepoleAlpha(HP_HZ);
    aLP = onepoleAlpha(LP_HZ);
    aBank = emaAlpha(BANK_TAU);
    aPerf = emaAlpha(PERF_TAU);
    baseline = lp1 = lp2 = filtered = 0.0f;
    acPower = 0.0f;
    primed = false;
    n = 0;
    bpm = DEFAULT_BPM;
    confidence = 0.0f;
    phase = 0.0f;

    for (int k = 0; k < N_BINS; k++) {
      binBpm[k] = BPM_MIN + (BPM_MAX - BPM_MIN) * (float)k / (float)(N_BINS - 1);
      float w = 2.0f * (float)M_PI * (binBpm[k] / 60.0f) / (float)DSP_HZ;
      stepRe[k] = cosf(w);
      stepIm[k] = -sinf(w);
      rotRe[k] = 1.0f; rotIm[k] = 0.0f;
      resRe[k] = 0.0f; resIm[k] = 0.0f;
    }
  }

  // One 25 Hz sample. ~14 kflop/s at 48 bins -- negligible on an ESP32.
  void update(float x) {
    if (!primed) { baseline = x; primed = true; }

    baseline += aHP * (x - baseline);
    float ac = x - baseline;
    // No gain stage: multiplying scales signal and noise alike, which is exactly
    // why the old 10x boost bought nothing.
    lp1 += aLP * (ac - lp1);
    lp2 += aLP * (lp1 - lp2);
    float y = lp2;
    filtered = y;
    acPower += aPerf * (y * y - acPower);   // envelope for the perfusion index

    for (int k = 0; k < N_BINS; k++) {
      float re = rotRe[k] * stepRe[k] - rotIm[k] * stepIm[k];
      float im = rotRe[k] * stepIm[k] + rotIm[k] * stepRe[k];
      rotRe[k] = re;
      rotIm[k] = im;
      resRe[k] += aBank * (y * re - resRe[k]);
      resIm[k] += aBank * (y * im - resIm[k]);
    }

    if (++n % RENORM_INTERVAL == 0) {
      // Recursive rotation drifts off the unit circle; pull it back.
      for (int k = 0; k < N_BINS; k++) {
        float m = sqrtf(rotRe[k] * rotRe[k] + rotIm[k] * rotIm[k]);
        if (m > 1e-6f) { rotRe[k] /= m; rotIm[k] /= m; }
      }
    }
  }

  // Drop all accumulated evidence. Called when the device leaves the wrist so a
  // fresh wearer is not shown the previous one's rate while the bank re-converges.
  void clearBank() {
    for (int k = 0; k < N_BINS; k++) { resRe[k] = 0.0f; resIm[k] = 0.0f; }
    acPower = 0.0f;
    confidence = 0.0f;
    primed = false;
  }

  // Render-rate work, not per-sample: one atan2 and one pass over the bank,
  // comparing squared magnitudes so the search needs no sqrt.
  void estimate(float dtSeconds) {
    // Search excludes bins 0 and N_BINS-1 (the BPM_MIN/BPM_MAX edges). Both are a
    // known artifact trap on this hardware: on-skin captures 2026-09-02 repeatedly
    // pinned to bin 0 (40 BPM) at low confidence with no real 40 BPM signal present.
    // An edge bin also has no neighbor on one side, so the parabolic interpolation
    // below was already unsafe there -- excluding them fixes both problems at once.
    int best = 1;
    float bestP = -1.0f, totalP = 0.0f;
    for (int k = 0; k < N_BINS; k++) {
      float p = resRe[k] * resRe[k] + resIm[k] * resIm[k];
      totalP += p;
      if (k > 0 && k < N_BINS - 1 && p > bestP) { bestP = p; best = k; }
    }

    float rawBpm = binBpm[best];
    {
      float lo = resRe[best - 1] * resRe[best - 1] + resIm[best - 1] * resIm[best - 1];
      float hi = resRe[best + 1] * resRe[best + 1] + resIm[best + 1] * resIm[best + 1];
      float denom = lo - 2.0f * bestP + hi;
      if (fabsf(denom) > 1e-12f) {
        float shift = clampf(0.5f * (lo - hi) / denom, -1.0f, 1.0f);
        rawBpm += shift * (binBpm[1] - binBpm[0]);
      }
    }

    // Peak share of total bank power: scale-free and bounded. Flat noise gives
    // 1/N_BINS; a dominant rhythm gives much more. Confidence is only mildly
    // discriminative at this SNR (~1.3x separation), so it weights how fast BPM may
    // move rather than gating it outright -- low confidence parks the reading on its
    // last good value instead of letting it jump.
    confidence = bestP / (totalP + 1e-12f);
    peakBin = (uint8_t)best;
    // Instantaneous beat phase. arg(R) alone is only the *offset* of the beat
    // relative to the demodulation reference -- for a signal sitting on the bin
    // frequency R is a near-static phasor, so it does not advance once per beat.
    // Multiplying the rotator back in (R * conj(rot)) restores the running phase.
    float beatRe = resRe[best] * rotRe[best] + resIm[best] * rotIm[best];
    float beatIm = resIm[best] * rotRe[best] - resRe[best] * rotIm[best];
    phase = atan2f(beatIm, beatRe);

    if (confidence >= confGate) {
      float weight = fminf(1.0f, confidence / confRef);
      float limit = slewBpmPerS * weight * dtSeconds;
      bpm += clampf(rawBpm - bpm, -limit, limit);
    }
  }
};

// ============================================================================
// GSR TWO-TIMESCALE AROUSAL ENGINE (auto-ranging)
// ============================================================================
struct GsrTracker {
  float aTonic, aPhasic, aAttack, aRelease, aRange, aOutAttack, aOutRelease;
  float tonic, smooth, prevPhasic, drive, range, arousal;
  bool primed;

  void begin() {
    aTonic = emaAlpha(TONIC_TAU);
    aPhasic = emaAlpha(PHASIC_TAU);
    aAttack = emaAlpha(SCR_ATTACK_TAU);
    aRelease = emaAlpha(SCR_RELEASE_TAU);
    aRange = emaAlpha(RANGE_TAU);
    aOutAttack = emaAlpha(OUT_ATTACK_TAU);
    aOutRelease = emaAlpha(OUT_RELEASE_TAU);
    primed = false;
    reset(0.0f);
  }

  // Long-press recalibration: re-anchor tonic and the auto-range envelope.
  void reset(float x) {
    tonic = smooth = x;
    prevPhasic = 0.0f;
    drive = 0.0f;
    range = RANGE_FLOOR;
    arousal = 0.0f;
  }

  void update(float x) {
    if (!primed) { reset(x); primed = true; }

    smooth += aPhasic * (x - smooth);
    tonic += aTonic * (smooth - tonic);
    float phasic = smooth - tonic;

    // The Grove GSR output *falls* as skin conductance rises, so a falling value is
    // the arousal direction; half-wave rectify so only rises count. The slope is
    // taken on the phasic component rather than the smoothed signal: the raw signal
    // drifts down ~0.9 counts/s (normal tonic habituation), which would otherwise
    // read as permanent arousal.
    float slope = (prevPhasic - phasic) * (float)DSP_HZ;   // counts/s
    prevPhasic = phasic;
    slope = clampf(slope, 0.0f, SLOPE_CLAMP);

    drive += (slope > drive ? aAttack : aRelease) * (slope - drive);

    // Auto-range against recent mean activity, replacing the old hardcoded /120.0
    // scale that pinned the bar at zero for 33% of bio2.log and never passed 0.42.
    // A peak-hold envelope was tried and rejected: one contact artifact pins it and
    // crushes everything after. Trade-off: "resting" reads mid-scale rather than
    // low -- right for an LED display, wrong for a clinical measure.
    range += aRange * (drive - range);
    if (range < RANGE_FLOOR) range = RANGE_FLOOR;

    float target = fminf(1.0f, drive / (RANGE_GAIN * range));
    arousal += (target > arousal ? aOutAttack : aOutRelease) * (target - arousal);
  }
};

// ============================================================================
// WEAR / CONTACT STATE MACHINE
// ============================================================================
struct WearDetect {
  bool worn = false;
  bool candidate = false;
  uint32_t since = 0;

  // Set for the single update on which the device was released. The caller is
  // responsible for acting on it (clearing the resonator bank), rather than this
  // state machine reaching into the tracker -- that coupling would drag the whole
  // pulse engine into a class that only needs to answer one yes/no question.
  bool justReleased = false;

  // Whether the PPG signal is strong enough to trust the rate. Independent of the
  // wear decision, which is also PPG-derived now (IR DC) -- pulseTrusted additionally
  // requires perfusion, so a worn-but-flat signal (bad contact, motion) still reports
  // honestly instead of a confident number.
  bool pulseTrusted = false;

  // Runtime-tunable; see BraceletConfig.
  float piTrustMin = PI_TRUST_MIN;
  float irWornMin = (float)PPG_IR_WORN_MIN;
  float irWornMax = (float)PPG_IR_WORN_MAX;

  void applyConfig(const BraceletConfig &cfg) {
    piTrustMin = cfg.piTrustMin;
    irWornMin = cfg.irWornMin;
    irWornMax = cfg.irWornMax;
  }

  // ppgIr: raw IR DC level from the MAX30102, not GSR (-6y4 -- GSR is a floating
  // analog input when unworn and has no stable disconnected signature).
  bool update(uint32_t ppgIr, float perfusion, uint32_t now) {
    justReleased = false;
    pulseTrusted = perfusion >= piTrustMin;
    bool contact = ((float)ppgIr >= irWornMin && (float)ppgIr <= irWornMax);

    if (contact == worn) {
      candidate = worn;              // steady; nothing pending
    } else if (contact != candidate) {
      candidate = contact;           // state flipped; start the clock
      since = now;
    } else {
      uint32_t need = contact ? WEAR_ON_MS : WEAR_OFF_MS;
      if (now - since >= need) {
        worn = contact;
        if (!worn) justReleased = true;
      }
    }
    return worn;
  }
};
