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

// 500 Hz ADC decimated to 25 Hz by a 20-sample boxcar. The 40 ms boxcar window nulls
// 25/50/75 Hz exactly, killing the 50 Hz mains hum measured on the GSR line at ~200x
// its noise floor, and delivers the same sqrt(20) noise reduction the old 16x
// oversampling burst did at 1/16 the analogRead() calls.
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
#define RANGE_FLOOR     0.5f
#define OUT_ATTACK_TAU  0.10f
#define OUT_RELEASE_TAU 1.50f

// ---- Wear / contact detection ----
// Without this the bank happily tracks noise and reports a confident heart rate for
// nobody. Two independent checks, because each covers the other's blind spot:
//
//  1. GSR electrodes. Skin between the pads reads 1175-1509 counts across every
//     capture taken so far (bio2/bio3/bio4); an open circuit rails to ~3900-4095.
//     That is a 2.6x margin, so it makes a solid hard gate.
//  2. PPG perfusion index (cardiac AC amplitude / DC level), the standard optical
//     contact metric. Measured 1.60% on bio4 (good contact) and 0.40% on bio2 (poor
//     but genuinely worn). The floor sits below both so it cannot false-negative on
//     a real wearer; it is published on serial so it can be tightened once we have a
//     calibrated not-worn capture.
//
// Confidence is deliberately NOT used here: an unworn board was observed reporting
// confidence 0.29, as high as a good worn signal, because the bank locks onto noise
// just as happily as onto a pulse.
#define GSR_WORN_MIN    500       // below this the pin is shorted or unpowered
#define GSR_WORN_MAX    3000      // above this the electrodes are open
#define PI_WORN_MIN     0.15f     // percent; conservative, see note above
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

// Initial BPM before the bank has converged. Shared by every consumer so the
// firmware and the offline reference start from the same state.
#define DEFAULT_BPM     78.0f

// ============================================================================
// PPG RESONATOR BANK (rate + confidence + beat phase from one structure)
// ============================================================================
struct PulseTracker {
  float aHP, aLP, aBank;
  float baseline, lp1, lp2, filtered;
  bool primed;

  float binBpm[N_BINS];
  float stepRe[N_BINS], stepIm[N_BINS];  // per-bin rotation e^(-j*w_k)
  float rotRe[N_BINS],  rotIm[N_BINS];   // running rotator
  float resRe[N_BINS],  resIm[N_BINS];   // resonator state
  uint32_t n;

  float acPower;                         // EMA of y^2, for the perfusion index
  float bpm, confidence, phase;

  // Cardiac AC as a percentage of DC level. For a sinusoid p2p = 2*sqrt(2)*rms.
  float perfusion() const {
    if (baseline < 1.0f) return 0.0f;
    return 100.0f * 2.83f * sqrtf(acPower) / baseline;
  }

  void begin() {
    aHP = onepoleAlpha(HP_HZ);
    aLP = onepoleAlpha(LP_HZ);
    aBank = emaAlpha(BANK_TAU);
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
    acPower += aBank * (y * y - acPower);   // slow envelope for the perfusion index

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
    int best = 0;
    float bestP = -1.0f, totalP = 0.0f;
    for (int k = 0; k < N_BINS; k++) {
      float p = resRe[k] * resRe[k] + resIm[k] * resIm[k];
      totalP += p;
      if (p > bestP) { bestP = p; best = k; }
    }

    float rawBpm = binBpm[best];
    if (best > 0 && best < N_BINS - 1) {
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
    // Instantaneous beat phase. arg(R) alone is only the *offset* of the beat
    // relative to the demodulation reference -- for a signal sitting on the bin
    // frequency R is a near-static phasor, so it does not advance once per beat.
    // Multiplying the rotator back in (R * conj(rot)) restores the running phase.
    float beatRe = resRe[best] * rotRe[best] + resIm[best] * rotIm[best];
    float beatIm = resIm[best] * rotRe[best] - resRe[best] * rotIm[best];
    phase = atan2f(beatIm, beatRe);

    if (confidence >= CONF_GATE) {
      float weight = fminf(1.0f, confidence / CONF_REF);
      float limit = SLEW_BPM_PER_S * weight * dtSeconds;
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

  bool update(uint16_t gsrRaw, float perfusion, uint32_t now) {
    justReleased = false;
    bool contact = (gsrRaw >= GSR_WORN_MIN && gsrRaw <= GSR_WORN_MAX) &&
                   (perfusion >= PI_WORN_MIN);

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
