/*
 * DSP v2 - Heartbeat & GSR Arousal Engine (validation sketch, no LEDs)
 *
 * Target MCU: Standard ESP32 WROOM-32 (PIN_GSR=34, PIN_PULSE=35) or ESP32-S3.
 *
 * This sketch is the bench-validation counterpart to tools/dsp_v2_sim.py: same
 * sample rates, same coefficients, same update order, so anything verified offline
 * against bio2.log behaves identically here. It streams CSV over serial instead of
 * driving the LED matrix; once the numbers check out on-device, the two tracker
 * blocks below port straight into main_armband.ino.
 *
 * Serial output: Timestamp_ms,BPM,Confidence,Phase,Arousal,Tonic,RawPulse,RawGSR
 *
 * Why this replaces the threshold detector in main_armband.ino
 * ------------------------------------------------------------
 * Measured on bio2.log (150 s, 500 Hz): the cardiac component is only 3-10 ADC
 * counts peak-to-peak against 2.8 counts of broadband noise, i.e. SNR below 1 for
 * most of the recording. The old threshold-crossing detector reported 113.8 BPM
 * against a true ~88 BPM, with 610 ms of IBI jitter and a 10.7 s dropout. No
 * threshold scheme survives that SNR.
 *
 * Instead a bank of complex one-pole resonators (a sliding DFT with exponential
 * forgetting) integrates the signal over many cardiac cycles. It yields rate,
 * confidence and beat *phase* from one structure, and it degrades smoothly rather
 * than stalling -- which is what the LEDs actually need.
 */

#include <Arduino.h>
#include <math.h>

// ============================================================================
// MCU HARDWARE SELECTION (Uncomment for ESP32-S3)
// ============================================================================
//#define MCU_ESP32_S3

#ifdef MCU_ESP32_S3
  #define PIN_GSR         1
  #define PIN_PULSE       2
  #define PIN_BUTTON      5
#else // Standard ESP32 DevKit WROOM-32
  #define PIN_GSR         34
  #define PIN_PULSE       35
  #define PIN_BUTTON      18
#endif

// ============================================================================
// SAMPLE RATES
// ============================================================================
// The ADC runs at 500 Hz and a 20-sample boxcar decimates to 25 Hz. A 20-sample
// average at 500 Hz is a 40 ms window, whose sinc nulls land exactly on 25/50/75 Hz
// -- a free, deep notch on the 50 Hz mains hum measured on the GSR line at ~200x
// the noise floor. It also gives the same sqrt(20) noise reduction as the old 16x
// oversampling burst while making 1/16 as many analogRead() calls.
#define RAW_HZ          500
#define DECIM           20
#define DSP_HZ          (RAW_HZ / DECIM)   // 25 Hz
#define RAW_PERIOD_MS   (1000 / RAW_HZ)    // 2 ms

// ============================================================================
// PULSE CHAIN TUNING
// ============================================================================
#define HP_HZ           0.5f    // baseline tracker
#define LP_HZ           4.0f    // two cascaded poles; kills the 10.2 Hz tremor
                                // artifact that dominates the raw PPG spectrum
#define BPM_MIN         40.0f   // athletic resting
#define BPM_MAX         190.0f  // dancing; do not narrow this range
#define N_BINS          48
#define BANK_TAU        10.0f
#define RENORM_INTERVAL 256     // rotator renormalisation period, in samples

// Confidence is only mildly discriminative at this SNR (~1.3x separation between
// good and bad windows), so it weights how fast BPM may move rather than gating it
// outright. Low confidence parks the reading on its last good value.
#define SLEW_BPM_PER_S  8.0f
#define CONF_REF        0.18f   // peak/total power ratio treated as fully trusted
#define CONF_GATE       0.06f   // below this the bank is noise; freeze BPM

// ============================================================================
// GSR CHAIN TUNING
// ============================================================================
#define TONIC_TAU       45.0f
#define PHASIC_TAU      0.7f
#define SCR_ATTACK_TAU  0.15f
#define SCR_RELEASE_TAU 3.0f
#define SLOPE_CLAMP     60.0f   // counts/s; rejects contact/motion steps (measured
                                // up to 570 counts/s on bio2.log)
#define RANGE_TAU       30.0f
#define RANGE_GAIN      2.5f    // drive must hit 2.5x its recent mean to peg the bar
#define RANGE_FLOOR     0.5f    // counts/s; stops a dead sensor being amplified
#define OUT_ATTACK_TAU  0.10f
#define OUT_RELEASE_TAU 1.50f

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
// SHARED STATE (same portMUX pattern as main_armband.ino)
// ============================================================================
struct BiometricData {
  float bpm = 78.0f;
  float confidence = 0.0f;
  float phase = 0.0f;       // radians; beat phase for LED animation
  float arousal = 0.0f;     // 0.0 (calm) .. 1.0 (peak)
  float tonic = 0.0f;       // slow skin-conductance baseline, ADC counts
  uint16_t pulseRaw = 0;
  uint16_t gsrRaw = 0;
} bio;

portMUX_TYPE bioMux = portMUX_INITIALIZER_UNLOCKED;
inline void bioDataLock()   { portENTER_CRITICAL(&bioMux); }
inline void bioDataUnlock() { portEXIT_CRITICAL(&bioMux); }

// ============================================================================
// DSP 1: PPG RESONATOR BANK
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

  float bpm, confidence, phase;

  void begin() {
    aHP = onepoleAlpha(HP_HZ);
    aLP = onepoleAlpha(LP_HZ);
    aBank = emaAlpha(BANK_TAU);
    baseline = lp1 = lp2 = filtered = 0.0f;
    primed = false;
    n = 0;
    bpm = 78.0f;
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

  // One 25 Hz sample. ~14 kflop/s total at 48 bins -- negligible on an ESP32.
  void update(float x) {
    if (!primed) { baseline = x; primed = true; }

    baseline += aHP * (x - baseline);
    float ac = x - baseline;
    // No gain stage here: multiplying scales signal and noise alike, which is
    // exactly what made the old 10x boost useless.
    lp1 += aLP * (ac - lp1);
    lp2 += aLP * (lp1 - lp2);
    float y = lp2;
    filtered = y;

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

  // Called at render rate (60 Hz), not per sample: one atan2 and one pass over the
  // bank, comparing squared magnitudes so there is no sqrt in the search.
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

    // Peak share of total bank power: scale-free and bounded. Flat noise across the
    // bank gives 1/N_BINS; a dominant rhythm gives much more.
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
// DSP 2: GSR TWO-TIMESCALE AROUSAL
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

    // Auto-range against recent mean activity. This replaces the hardcoded /120.0
    // scale, which zeroed the old output for 33% of bio2.log. A peak-hold envelope
    // was tried and rejected: one contact artifact pins it and crushes everything
    // after. The trade-off is that "resting" reads mid-scale rather than low --
    // correct for an LED display, wrong for a clinical measure.
    range += aRange * (drive - range);
    if (range < RANGE_FLOOR) range = RANGE_FLOOR;

    float target = fminf(1.0f, drive / (RANGE_GAIN * range));
    arousal += (target > arousal ? aOutAttack : aOutRelease) * (target - arousal);
  }
};

PulseTracker pulse;
GsrTracker gsr;

// ============================================================================
// CORE 0 TASK: SAMPLING & DSP
// ============================================================================
volatile bool gsrResetRequest = false;

void TaskSensorDSP(void *pvParameters) {
  uint32_t accPulse = 0, accGsr = 0;
  uint8_t accN = 0;
  uint16_t lastPulse = 0, lastGsr = 0;

  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(RAW_PERIOD_MS);

  for (;;) {
    // One read per channel per tick. The boxcar below does the averaging, so the
    // old 16-read burst is redundant.
    lastPulse = analogRead(PIN_PULSE);
    lastGsr   = analogRead(PIN_GSR);
    accPulse += lastPulse;
    accGsr   += lastGsr;

    if (++accN >= DECIM) {
      float xp = (float)accPulse / (float)DECIM;
      float xg = (float)accGsr / (float)DECIM;
      accPulse = accGsr = 0;
      accN = 0;

      if (gsrResetRequest) {
        gsrResetRequest = false;
        gsr.reset(xg);
      }

      pulse.update(xp);
      gsr.update(xg);

      bioDataLock();
      bio.arousal = gsr.arousal;
      bio.tonic = gsr.tonic;
      bio.pulseRaw = lastPulse;
      bio.gsrRaw = lastGsr;
      bioDataUnlock();
    }

    // Real sleep, not a spin. The old `while (micros() - start < 2000) {}` held
    // core 0 at 100% duty and blocked the idle task from saving any power.
    vTaskDelayUntil(&lastWake, period);
  }
}

// ============================================================================
// BUTTON (polled at render rate, not at 500 Hz)
// ============================================================================
unsigned long buttonPressStart = 0;
bool lastButtonState = HIGH;

void processButton() {
  bool currentState = digitalRead(PIN_BUTTON);

  if (lastButtonState == HIGH && currentState == LOW) {
    buttonPressStart = millis();
  } else if (lastButtonState == LOW && currentState == HIGH) {
    unsigned long duration = millis() - buttonPressStart;
    if (duration >= 1000) {
      gsrResetRequest = true;
      Serial.println(F("# GSR baseline and auto-range reset"));
    }
  }
  lastButtonState = currentState;
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pulse.begin();
  gsr.begin();

  Serial.println(F("# DSP v2 validation sketch"));
  Serial.println(F("Timestamp_ms,BPM,Confidence,Phase,Arousal,Tonic,RawPulse,RawGSR"));

  xTaskCreatePinnedToCore(TaskSensorDSP, "SensorDSP", 4096, NULL, 2, NULL, 0);
}

// ============================================================================
// CORE 1: ESTIMATION + CSV STREAM
// ============================================================================
unsigned long lastEstimateMs = 0;
unsigned long lastPrintMs = 0;

void loop() {
  unsigned long now = millis();

  // Estimation runs at the LED render rate in the real firmware; keep it here so
  // the tuning transfers unchanged.
  float dt = (now - lastEstimateMs) / 1000.0f;
  lastEstimateMs = now;
  if (dt > 0.0f && dt < 1.0f) {
    pulse.estimate(dt);
    bioDataLock();
    bio.bpm = pulse.bpm;
    bio.confidence = pulse.confidence;
    bio.phase = pulse.phase;
    bioDataUnlock();
  }

  processButton();

  if (now - lastPrintMs >= 100) {   // 10 Hz CSV
    lastPrintMs = now;
    bioDataLock();
    float bpm = bio.bpm, conf = bio.confidence, phase = bio.phase;
    float arousal = bio.arousal, tonic = bio.tonic;
    uint16_t rp = bio.pulseRaw, rg = bio.gsrRaw;
    bioDataUnlock();

    Serial.print(now);          Serial.print(',');
    Serial.print(bpm, 1);       Serial.print(',');
    Serial.print(conf, 3);      Serial.print(',');
    Serial.print(phase, 2);     Serial.print(',');
    Serial.print(arousal, 3);   Serial.print(',');
    Serial.print(tonic, 1);     Serial.print(',');
    Serial.print(rp);           Serial.print(',');
    Serial.println(rg);
  }

  delay(16);   // ~60 Hz, matching the FastLED render loop in main_armband.ino
}
