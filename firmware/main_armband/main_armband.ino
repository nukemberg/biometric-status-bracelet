/*
 * Biometric Status Bracelet - Production Main Firmware (DSP v2 Resonator Engine)
 * Framework: Arduino / ESP32 FreeRTOS Dual-Core Engine
 *
 * The pulse and GSR engines here were validated offline against bio2.log by
 * tools/dsp_v2_sim.py and proven bit-identical to this C++ by tools/dsp_v2_parity.sh.
 * firmware/dsp_v2/dsp_v2.ino is the standalone bench version that streams CSV.
 *
 * Why threshold beat detection was dropped
 * ----------------------------------------
 * On this hardware the cardiac component of the PPG signal is 3-10 ADC counts
 * peak-to-peak against 2.8 counts of broadband noise -- SNR below 1 for most of
 * bio2.log. Measured on that capture, the previous engines reported 113.8 BPM
 * (450 ms refractory) and 187 BPM (285 ms dynamic refractory, pegged at its 210 BPM
 * cap) against a true rate of ~88 BPM, both with multi-second dropouts. Lowering the
 * refractory window makes it worse, because the extra triggers are noise, not beats.
 *
 * Instead a bank of complex one-pole resonators (a sliding DFT with exponential
 * forgetting) integrates over many cardiac cycles, yielding rate, confidence and
 * beat *phase* from one structure. The LEDs are driven by a phase-locked oscillator,
 * so the animation stays smooth at frame rate and cannot stall when individual beats
 * are unrecoverable -- which is the behaviour that actually matters here.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <FastLED.h>
#include <math.h>

// ============================================================================
// MCU HARDWARE SELECTION
// ============================================================================
// ESP32-S3 is the deployment target; the WROOM-32 DevKit is the bench rig every
// capture in samples/ was taken on. Analog sensors must stay on ADC1 -- ADC2 is
// unusable on both chips while the radio is active.
//#define MCU_ESP32_S3

#ifdef MCU_ESP32_S3
  #define PIN_GSR         1     // ADC1_CH0
  #define PIN_PULSE       2     // ADC1_CH1
  #define PIN_SDA         8
  #define PIN_SCL         9
  #define PIN_BUTTON      5
  #define PIN_LED         4
#else // Standard ESP32 DevKit WROOM-32
  #define PIN_GSR         34
  #define PIN_PULSE       35
  #define PIN_SDA         21
  #define PIN_SCL         22
  #define PIN_BUTTON      18
  #define PIN_LED         4
#endif

#define NUM_LEDS        21
#define COLOR_ORDER     GRB
#define LED_TYPE        WS2812B

CRGB leds[NUM_LEDS];
Adafruit_BME280 bme;
bool bmeConnected = false;

// All pulse/GSR/wear DSP lives in libraries/BraceletDSP. It is kept free of
// Arduino and BLE types so tools/dsp_v2_parity.sh can compile these exact
// trackers on a host and prove them equal to the Python reference.
#include <dsp.h>

struct BiometricData {
  uint16_t pulseRaw = 0;
  float bpm = DEFAULT_BPM;
  float confidence = 0.0;   // 0..1, peak share of resonator-bank power
  float phase = 0.0;        // radians, beat phase from the winning resonator

  float perfusion = 0.0;    // percent, cardiac AC / DC -- PPG contact quality
  bool worn = false;        // both sensors report skin contact

  uint16_t gsrRaw = 0;
  float gsrTonic = 0.0;
  float gsrExcitement = 0.0;

  float tempC = 25.0;
  float humidity = 50.0;

  uint8_t displayMode = 0;
  uint8_t brightness = 60;
} bio;

portMUX_TYPE bioMux = portMUX_INITIALIZER_UNLOCKED;

inline void bioDataLock() { portENTER_CRITICAL(&bioMux); }
inline void bioDataUnlock() { portEXIT_CRITICAL(&bioMux); }

PulseTracker pulse;
GsrTracker gsr;

volatile bool gsrResetRequest = false;

WearDetect wear;

// ============================================================================
// BUTTON INTERACTION HANDLER (polled from the render loop at ~60 Hz)
// ============================================================================
unsigned long buttonPressStart = 0;
bool lastButtonState = HIGH;

void processButton() {
  bool currentState = digitalRead(PIN_BUTTON);

  if (lastButtonState == HIGH && currentState == LOW) {
    buttonPressStart = millis();
  } else if (lastButtonState == LOW && currentState == HIGH) {
    unsigned long duration = millis() - buttonPressStart;

    if (duration > 50 && duration < 1000) {
      bioDataLock();
      bio.displayMode = (bio.displayMode + 1) % 3;
      bioDataUnlock();
    } else if (duration >= 1000) {
      // Handed to the DSP task so the reset lands on a decimated sample.
      gsrResetRequest = true;
    }
  }
  lastButtonState = currentState;
}

// ============================================================================
// CORE 0 TASK: SENSOR SAMPLING & DSP
// ============================================================================
void TaskSensorDSP(void *pvParameters) {
  uint32_t accPulse = 0, accGsr = 0;
  uint8_t accN = 0;
  uint16_t lastPulse = 0, lastGsr = 0;
  unsigned long lastBmeRead = 0;

  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(RAW_PERIOD_MS);

  for (;;) {
    // One read per channel per tick; the boxcar below does the averaging, so the
    // old 16-read burst was redundant work.
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

      float pi = pulse.perfusion();
      bool worn = wear.update(lastGsr, pi, (uint32_t)millis());
      // The state machine reports the release; acting on it is ours to do, so a
      // fresh wearer never sees the previous one's rate while the bank reconverges.
      if (wear.justReleased) pulse.clearBank();

      bioDataLock();
      bio.gsrExcitement = gsr.arousal;
      bio.gsrTonic = gsr.tonic;
      bio.pulseRaw = lastPulse;
      bio.gsrRaw = lastGsr;
      bio.perfusion = pi;
      bio.worn = worn;
      bioDataUnlock();
    }

    if (bmeConnected && (millis() - lastBmeRead > 500)) {
      float t = bme.readTemperature();
      float h = bme.readHumidity();
      bioDataLock();
      if (!isnan(t)) bio.tempC = t;
      if (!isnan(h)) bio.humidity = h;
      bioDataUnlock();
      lastBmeRead = millis();
    }

    // Real sleep, not a spin. The old `while (micros() - startUs < 2000) {}` held
    // core 0 at 100% duty and stopped the idle task from ever saving power.
    vTaskDelayUntil(&lastWake, period);
  }
}

// ============================================================================
// BEAT PHASE OSCILLATOR
// ============================================================================
// A free-running oscillator advances at the tracked BPM every frame and is gently
// pulled toward the resonator phase. The pull is deliberately soft: it keeps the
// animation locked to the real heartbeat without letting a noisy estimate jerk or
// stall it. This replaces the old `beatFade` counter, which was decayed by 12 per
// 500 Hz DSP sample and so went 255 -> 0 in 42 ms -- an invisible flash.
#define PHASE_PULL      0.12f   // fraction of phase error corrected per frame
#define WAVE_LAG        0.35f   // radians of delay per LED away from centre

float beatPhase = 0.0f;

static inline float wrapPi(float a) {
  while (a > (float)M_PI)  a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}

void advanceBeatPhase(float bpm, float resonatorPhase, float confidence, float dt) {
  beatPhase += 2.0f * (float)M_PI * (bpm / 60.0f) * dt;

  // Only chase the resonator when it is worth listening to; otherwise free-run.
  if (confidence >= CONF_GATE) {
    float pull = PHASE_PULL * fminf(1.0f, confidence / CONF_REF);
    beatPhase += pull * wrapPi(resonatorPhase - beatPhase);
  }
  beatPhase = wrapPi(beatPhase);
}

// ============================================================================
// PULSE COLOUR GRADIENT
// ============================================================================
// Continuous warm ramp instead of discrete colour bands: a slow resting pulse reads
// orange/yellow, it deepens through red as the rate climbs, and a dancing rate goes
// red-pink. Two reasons to prefer a gradient here beyond the look --
//
//  1. Discrete bands strobe. The tracker occasionally latches onto the 2nd or 3rd
//     harmonic for a few seconds (the 2nd harmonic carries 124 power against the
//     fundamental's 151 on bio4.csv), and a 62 -> 126 BPM excursion crossed three
//     band edges, flashing cyan-green-yellow-red and back. A gradient turns the same
//     excursion into a smooth colour drift.
//  2. It removes the band-edge tuning problem entirely -- there are no edges.
//
// FastLED hue is a uint8 that wraps, so a single descending ramp walks
// orange -> red -> through zero -> pink without any special-casing.
#define HUE_BPM_LO      50.0f     // anchor: slow resting pulse
#define HUE_BPM_HI      190.0f    // anchor: peak dancing rate
#define HUE_AT_LO       48.0f     // orange/yellow
#define HUE_AT_HI       (-27.0f)  // wraps to 229 = pink; crosses pure red at ~140 BPM

// Visual-only smoothing. The beat animation follows BPM instantly via the phase
// oscillator; only the colour is damped, so responsiveness is unaffected.
#define HUE_BPM_TAU     3.0f
float displayBpm = DEFAULT_BPM;

static inline uint8_t pulseHueFor(float bpm) {
  float f = (bpm - HUE_BPM_LO) / (HUE_BPM_HI - HUE_BPM_LO);
  f = clampf(f, 0.0f, 1.0f);
  float h = HUE_AT_LO + f * (HUE_AT_HI - HUE_AT_LO);
  int wrapped = ((int)lroundf(h)) & 0xFF;   // uint8 wraparound gives red -> pink
  return (uint8_t)wrapped;
}

// ============================================================================
// STANDBY DISPLAY (nothing on the wrist)
// ============================================================================
// Slow indigo breath across the whole panel. Deliberately unlike any active state so
// "not being worn" is never mistaken for a reading.
void renderStandby() {
  float breath = 0.5f * (1.0f + sinf((float)millis() * 0.0011f));   // ~0.17 Hz
  uint8_t v = 8 + (uint8_t)(28.0f * breath);
  for (int i = 0; i < NUM_LEDS; i++) leds[i] = CHSV(HUE_BLUE, 200, v);
}

// Systole-like envelope: a sharp bright peak once per cycle rather than a sine.
static inline uint8_t beatEnvelope(float phase) {
  float v = 0.5f * (1.0f + cosf(phase));
  v = v * v;
  v = v * v;                       // ^4, narrows the peak
  return (uint8_t)(v * 255.0f);
}

// ============================================================================
// FASTLED MATRIX RENDER ENGINE (3 SEGMENTS x 7 LEDs)
// ============================================================================
void renderBiometricPanel() {
  bioDataLock();
  float bpm = bio.bpm;
  float confidence = bio.confidence;
  float excitement = bio.gsrExcitement;
  float tempC = bio.tempC;
  uint8_t mode = bio.displayMode;
  bool worn = bio.worn;
  bioDataUnlock();

  FastLED.clear();

  // Nothing on the wrist: show standby and return. Anything else would be inventing
  // a heart rate for an empty room.
  if (!worn) { renderStandby(); return; }

  // --------------------------------------------------------------------------
  // SEGMENT 1: CARDIAC HEARTBEAT PULSE (LEDs 0 to 6)
  // Phase-locked thumping wave; smooth at frame rate, never stalls
  // --------------------------------------------------------------------------
  // Warm gradient: orange/yellow when slow, deep red as it climbs, pink at the top.
  uint8_t pulseHue = pulseHueFor(displayBpm);

  // Low confidence desaturates toward white rather than freezing the animation, so
  // a poor sensor contact reads as "unsure" instead of as a dead panel.
  uint8_t sat = 120 + (uint8_t)(135.0f * fminf(1.0f, confidence / CONF_REF));

  for (int i = 0; i < 7; i++) {
    float dist = fabsf((float)i - 3.0f);
    uint8_t v = beatEnvelope(beatPhase - dist * WAVE_LAG);
    leds[i] = CHSV(pulseHue, sat, v);
  }

  // --------------------------------------------------------------------------
  // SEGMENT 2: GSR EXCITEMENT VU-METER (LEDs 7 to 13)
  // --------------------------------------------------------------------------
  uint8_t litLedsGSR = (uint8_t)(excitement * 7.0 + 0.5);
  if (litLedsGSR > 7) litLedsGSR = 7;

  for (int i = 0; i < 7; i++) {
    int ledIndex = 7 + i;
    if (i < litLedsGSR) {
      uint8_t hue = map(i, 0, 6, 96, 240); // Green -> Purple
      leds[ledIndex] = CHSV(hue, 255, 220);
    } else {
      leds[ledIndex] = CRGB(5, 5, 10);
    }
  }

  // --------------------------------------------------------------------------
  // SEGMENT 3: THERMAL TEMPERATURE GAUGE (LEDs 14 to 20)
  // --------------------------------------------------------------------------
  float tempClamped = constrain(tempC, 25.0, 34.0);
  uint8_t litLedsTemp = map((long)(tempClamped * 10), 250, 340, 1, 7);

  for (int i = 0; i < 7; i++) {
    int ledIndex = 14 + i;
    if (i < litLedsTemp) {
      uint8_t hue = map(i, 0, 6, 160, 0); // Blue -> Red
      leds[ledIndex] = CHSV(hue, 255, 200);
    } else {
      leds[ledIndex] = CRGB(2, 2, 5);
    }
  }

  // --------------------------------------------------------------------------
  // OVERDRIVE / HYPE PARTY STROBE
  // --------------------------------------------------------------------------
  // Auto-trigger keys off the smoothed BPM, so a transient harmonic lock cannot
  // strobe the whole panel for a few seconds.
  if (mode == 1 || (displayBpm > 150.0f && excitement > 0.65)) {
    uint8_t starHue = millis() / 5;
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] += CHSV(starHue + (i * 12), 255, 150);
    }
  } else if (mode == 2) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i].nscale8_video(30);
    }
  }
}

// ============================================================================
// MAIN SETUP & LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pulse.begin();
  gsr.begin();

  FastLED.addLeds<LED_TYPE, PIN_LED, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(bio.brightness);
  FastLED.clear();
  FastLED.show();

  Wire.begin(PIN_SDA, PIN_SCL);
  if (bme.begin(0x76, &Wire) || bme.begin(0x77, &Wire)) {
    bmeConnected = true;
  }

  xTaskCreatePinnedToCore(TaskSensorDSP, "SensorDSP", 4096, NULL, 2, NULL, 0);
}

unsigned long lastSerialPrint = 0;
unsigned long lastFrameMs = 0;

void loop() {
  unsigned long now = millis();
  float dt = (now - lastFrameMs) / 1000.0f;
  lastFrameMs = now;
  if (dt <= 0.0f || dt > 1.0f) dt = 0.016f;   // first frame / after a stall

  // Bank search, confidence and phase are render-rate work, not per-sample work.
  pulse.estimate(dt);
  bioDataLock();
  bio.bpm = pulse.bpm;
  bio.confidence = pulse.confidence;
  bio.phase = pulse.phase;
  float bpm = bio.bpm, confidence = bio.confidence, phase = bio.phase;
  bioDataUnlock();

  advanceBeatPhase(bpm, phase, confidence, dt);

  // Colour-only damping; the beat animation above already used the live BPM.
  displayBpm += fminf(1.0f, dt / HUE_BPM_TAU) * (bpm - displayBpm);

  processButton();

  renderBiometricPanel();
  FastLED.show();

  if (now - lastSerialPrint > 1000) {
    bioDataLock();
    float excitement = bio.gsrExcitement;
    float tempC = bio.tempC;
    uint8_t mode = bio.displayMode;
    bool worn = bio.worn;
    float pi = bio.perfusion;
    uint16_t gsrRaw = bio.gsrRaw;
    bioDataUnlock();

    if (!worn) {
      Serial.print(F("[Standby] Not worn | PerfIdx: "));
      Serial.print(pi, 2);
      Serial.print(F("% (need "));
      Serial.print(PI_WORN_MIN, 2);
      Serial.print(F(") | RawGSR: "));
      Serial.print(gsrRaw);
      Serial.print(F(" (need "));
      Serial.print(GSR_WORN_MIN);
      Serial.print(F("-"));
      Serial.print(GSR_WORN_MAX);
      Serial.println(F(")"));
    } else {
      Serial.print(F("[Biometrics] Live BPM: "));
      Serial.print(bpm, 1);
      Serial.print(F(" (conf "));
      Serial.print(confidence, 2);
      Serial.print(F(") | GSR Excitement: "));
      Serial.print(excitement * 100.0, 1);
      Serial.print(F("% | Temp: "));
      Serial.print(tempC, 1);
      Serial.print(F(" C | PerfIdx: "));
      Serial.print(pi, 2);
      Serial.print(F("% | Mode: "));
      Serial.println(mode);
    }

    lastSerialPrint = now;
  }

  delay(16); // 60 FPS
}
