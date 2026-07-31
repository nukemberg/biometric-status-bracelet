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
#include "ble_service.h"
#include "config_store.h"

// Live tunables. Loaded from NVS at boot (falling back to compiled defaults),
// applied to the trackers and to the LED hue anchors below, and updated by the
// Config BLE characteristic (-pmw, not yet wired) and CMD_RESET_CONFIG.
BraceletConfig cfg;

struct BiometricData {
  uint16_t pulseRaw = 0;
  float bpm = DEFAULT_BPM;
  float confidence = 0.0;   // 0..1, peak share of resonator-bank power
  float phase = 0.0;        // radians, beat phase from the winning resonator

  float perfusion = 0.0;    // percent, cardiac AC / DC -- PPG contact quality
  bool worn = false;        // GSR electrodes report skin contact
  bool pulseTrusted = false;// PPG strong enough to believe the rate

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
// Handed to the DSP task rather than applied inline: clearing the bank from another
// task while update() is midway through the resonator loop would corrupt it.
volatile bool bankResetRequest = false;
volatile bool configResetRequest = false;

// A BLE config write (-pmw) arrives on the NimBLE task but is applied on the DSP
// task, exactly like the reset requests above: applyConfig() touches the trackers
// and a torn read mid-update would be a state nobody tested. Only one pending
// write is kept -- a slider drag sends many, and coalescing to the latest value is
// the desired behaviour, not a loss.
volatile bool configParamPending = false;
uint8_t configParamId = 0;
float configParamValue = 0.0f;
portMUX_TYPE configMux = portMUX_INITIALIZER_UNLOCKED;
// NVS writes are deferred until writes stop arriving for a couple of seconds, so a
// slider drag retunes the device live but only hits flash once. NVS has limited
// erase cycles and a flash write mid-sampling-window adds jitter; debouncing both
// is cheap and removes both concerns. Set on the DSP task, read on the DSP task.
bool configDirty = false;
uint32_t configDirtyMs = 0;

WearDetect wear;

// Measured on core 0, where a NimBLE controller task will later compete for time.
// Reported every 10 s and then reset, so each line describes a fresh window.
JitterMonitor jitter;
portMUX_TYPE jitterMux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================================
// SIGNAL STREAM RING BUFFER
// ============================================================================
// The DSP task produces one sample per 25 Hz tick; the render loop drains five at a
// time and notifies at 5 Hz. Sized well above that so a late drain loses nothing --
// dropping samples silently would put gaps in a stream whose entire purpose is
// offline analysis, and the timestamps would still look continuous.
#define SIGBUF_LEN 32
BleSignalSample sigBuf[SIGBUF_LEN];
uint32_t sigTimestamp[SIGBUF_LEN];
volatile uint8_t sigHead = 0;   // written by the DSP task
volatile uint8_t sigTail = 0;   // written by the render loop
volatile uint32_t sigDropped = 0;
portMUX_TYPE sigMux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================================
// BLE CONTROL HANDLERS
// ============================================================================
// These run on the NimBLE task, not the render loop, so they take the same lock the
// render loop does and defer anything that must land on a DSP sample boundary --
// exactly as the button handler already does.
namespace {

void onSetMode(uint8_t mode) {
  bioDataLock();
  bio.displayMode = mode;
  bioDataUnlock();
}

void onSetBrightness(uint8_t value) {
  bioDataLock();
  bio.brightness = value;
  bioDataUnlock();
  // FastLED keeps its own copy; the render loop does not read bio.brightness.
  FastLED.setBrightness(value);
}

void onRecalibrateGsr() { gsrResetRequest = true; }

void onResetBank() { bankResetRequest = true; }

void onResetConfig() {
  // Deferred to the DSP task loop rather than touching the trackers directly from
  // the NimBLE task -- applyConfig() only writes a few floats so a torn read is
  // unlikely to matter, but staying consistent with how every other cross-task
  // mutation here is handled (gsrResetRequest, bankResetRequest) means there is one
  // pattern to reason about, not two.
  configResetRequest = true;
}

// Config characteristic (-pmw). Read pulls live values straight from cfg -- the
// DSP task is the only writer and float reads are aligned, so a snapshot may mix
// one just-changed field with the rest but never reads a torn byte. Write hands
// the (id, value) to the DSP task under a short critical section so the pair stays
// consistent, and the debounced save fires there once writes settle.
void onGetConfig(float *values) {
  ConfigStore::packValues(cfg, values);
}

void onSetConfigParam(uint8_t paramId, float value) {
  portENTER_CRITICAL(&configMux);
  configParamId = paramId;
  configParamValue = value;
  configParamPending = true;
  portEXIT_CRITICAL(&configMux);
}

void onSetStreams(uint8_t mask) {
  // Rising edge on the signals stream: drop whatever is still sitting in the ring
  // buffer from a previous session rather than draining it as if it were fresh.
  // Without this, re-enabling the stream after it had been off for a while sent one
  // batch of minutes-old samples followed by current ones -- same timestamp jump a
  // consumer would (correctly) flag as a gap, except the data itself was stale, not
  // missing. There is nothing to lose: an unread sample was already just static.
  static uint8_t lastMask = 0;
  if ((mask & STREAM_SIGNALS) && !(lastMask & STREAM_SIGNALS)) {
    portENTER_CRITICAL(&sigMux);
    sigTail = sigHead;
    portEXIT_CRITICAL(&sigMux);
  }
  lastMask = mask;

  Serial.print(F("[BLE] streams now 0x"));
  Serial.println(mask, HEX);
}

}  // namespace

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
    // Guarded because the render loop on core 1 reads and resets these. The
    // critical section is a handful of instructions against a 2000 us period, but
    // it is on the very path being measured, so it is part of what gets reported.
    portENTER_CRITICAL(&jitterMux);
    jitter.tick(micros(), RAW_PERIOD_MS * 1000);
    portEXIT_CRITICAL(&jitterMux);

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
      if (bankResetRequest) {
        bankResetRequest = false;
        pulse.clearBank();
      }
      if (configResetRequest) {
        configResetRequest = false;
        ConfigStore::resetToDefaults();
        cfg = BraceletConfig::defaults();
        pulse.applyConfig(cfg);
        wear.applyConfig(cfg);
        Serial.println(F("[Config] reset to compiled defaults"));
      }

      // Apply a pending BLE config write. Lives here, not in the NimBLE callback, for
      // the same torn-read reason as the reset above. A rejected value (unknown id or
      // out of plausibility bounds) is logged and dropped rather than clamped -- a
      // write the device cannot honour must not silently change something else.
      if (configParamPending) {
        uint8_t id; float val;
        portENTER_CRITICAL(&configMux);
        id = configParamId; val = configParamValue;
        configParamPending = false;
        portEXIT_CRITICAL(&configMux);
        if (ConfigStore::applyParam(cfg, id, val)) {
          pulse.applyConfig(cfg);
          wear.applyConfig(cfg);
          configDirty = true;
          configDirtyMs = millis();
          Serial.print(F("[Config] applied param 0x"));
          Serial.print(id, HEX);
          Serial.print(F(" = "));
          Serial.println(val, 4);
        } else {
          Serial.print(F("[Config] rejected param 0x"));
          Serial.print(id, HEX);
          Serial.print(F(" = "));
          Serial.println(val, 4);
        }
      }
      // Debounced persist: apply live on every write, but only hit flash once writes
      // have been quiet for a couple of seconds. A reset also clears the dirty flag
      // (it erased NVS itself), so this never re-saves stale values after a reset.
      if (configDirty && (millis() - configDirtyMs > 2000)) {
        configDirty = false;
        ConfigStore::save(cfg);
        Serial.println(F("[Config] saved to NVS"));
      }

      pulse.update(xp);
      gsr.update(xg);

      // Only pay for stream collection when a client is actually subscribed.
      if (BleService::streamMask() & STREAM_SIGNALS) {
        portENTER_CRITICAL(&sigMux);
        uint8_t next = (uint8_t)((sigHead + 1) % SIGBUF_LEN);
        if (next == sigTail) {
          // Full: discard the OLDEST so the stream stays current. Dropping the
          // newest instead would keep the data contiguous but drifting ever further
          // behind real time, which looks like clean data and is not.
          sigTail = (uint8_t)((sigTail + 1) % SIGBUF_LEN);
          sigDropped++;
        }
        {
          sigBuf[sigHead].pulseFiltered = pulse.filtered;
          sigBuf[sigHead].gsrPhasic = gsr.smooth - gsr.tonic;
          sigBuf[sigHead].pulseRaw = lastPulse;
          sigBuf[sigHead].gsrRaw = lastGsr;
          sigTimestamp[sigHead] = millis();
          sigHead = next;
        }
        portEXIT_CRITICAL(&sigMux);
      }

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
      bio.pulseTrusted = wear.pulseTrusted;
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
// orange -> red -> through zero -> pink without any special-casing. Anchors are
// runtime-tunable (cfg.hueBpmLo etc, see BraceletConfig in dsp.h) rather than
// #defines, so BLE config writes can retune the gradient without a reflash.

// Visual-only smoothing. The beat animation follows BPM instantly via the phase
// oscillator; only the colour is damped, so responsiveness is unaffected.
#define HUE_BPM_TAU     3.0f
float displayBpm = DEFAULT_BPM;

static inline uint8_t pulseHueFor(float bpm) {
  float f = (bpm - cfg.hueBpmLo) / (cfg.hueBpmHi - cfg.hueBpmLo);
  f = clampf(f, 0.0f, 1.0f);
  float h = cfg.hueAtLo + f * (cfg.hueAtHi - cfg.hueAtLo);
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
  bool pulseTrusted = bio.pulseTrusted;
  bioDataUnlock();

  FastLED.clear();

  // Nothing on the wrist: show standby and return. Anything else would be inventing
  // a heart rate for an empty room.
  if (!worn) { renderStandby(); return; }

  // --------------------------------------------------------------------------
  // SEGMENT 1: CARDIAC HEARTBEAT PULSE (LEDs 0 to 6)
  // Phase-locked thumping wave; smooth at frame rate, never stalls
  // --------------------------------------------------------------------------
  if (!pulseTrusted) {
    // Worn, but the PPG is too weak to believe the rate -- at this signal quality the
    // tracker has been measured reporting 113 BPM against a true 88. Show a slow
    // colourless sweep instead of a confident wrong colour. The other two segments
    // stay live because GSR and temperature are unaffected.
    uint8_t v = 10 + (uint8_t)(30.0f * (0.5f * (1.0f + sinf((float)millis() * 0.004f))));
    for (int i = 0; i < 7; i++) leds[i] = CHSV(0, 0, v);
  } else {
    // Warm gradient: orange/yellow when slow, deep red as it climbs, pink at the top.
    uint8_t pulseHue = pulseHueFor(displayBpm);

    // Low confidence desaturates toward white rather than freezing the animation, so
    // a marginal signal reads as "unsure" instead of as a dead panel.
    uint8_t sat = 120 + (uint8_t)(135.0f * fminf(1.0f, confidence / CONF_REF));

    for (int i = 0; i < 7; i++) {
      float dist = fabsf((float)i - 3.0f);
      uint8_t v = beatEnvelope(beatPhase - dist * WAVE_LAG);
      leds[i] = CHSV(pulseHue, sat, v);
    }
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

  cfg = ConfigStore::load();
  pulse.begin();
  pulse.applyConfig(cfg);
  gsr.begin();
  wear.applyConfig(cfg);

  FastLED.addLeds<LED_TYPE, PIN_LED, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(bio.brightness);
  FastLED.clear();
  FastLED.show();

  Wire.begin(PIN_SDA, PIN_SCL);
  if (bme.begin(0x76, &Wire) || bme.begin(0x77, &Wire)) {
    bmeConnected = true;
  }

  // Started after the DSP task so the jitter monitor has a clean window before the
  // radio exists; -a75 compares the two.
  xTaskCreatePinnedToCore(TaskSensorDSP, "SensorDSP", 4096, NULL, 2, NULL, 0);

  BleService::Handlers h;
  h.setMode = onSetMode;
  h.setBrightness = onSetBrightness;
  h.recalibrateGsr = onRecalibrateGsr;
  h.setStreams = onSetStreams;
  h.resetBank = onResetBank;
  h.resetConfig = onResetConfig;
  h.getConfig = onGetConfig;
  h.setConfigParam = onSetConfigParam;
  BleService::begin("Bracelet", h);
}

unsigned long lastSerialPrint = 0;
unsigned long lastFrameMs = 0;
unsigned long lastJitterPrint = 0;
unsigned long lastVitalsPublish = 0;
unsigned long lastSignalsPublish = 0;
unsigned long lastSpectrumPublish = 0;

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

  // 4 Hz vitals. Snapshot under the lock, pack and send outside it -- NimBLE can
  // block, and holding a portMUX critical section across that would stall the DSP
  // task on core 0, which is precisely the interference we are trying to measure.
  if (now - lastVitalsPublish >= 250) {
    lastVitalsPublish = now;

    BleVitals v;
    bioDataLock();
    v.bpm = bio.bpm;
    v.confidence = bio.confidence;
    v.arousal = bio.gsrExcitement;
    v.perfusion = bio.perfusion;
    v.tempC = bio.tempC;
    v.gsrTonic = bio.gsrTonic;
    v.gsrRaw = bio.gsrRaw;
    v.pulseRaw = bio.pulseRaw;
    v.brightness = bio.brightness;
    v.mode = bio.displayMode;
    v.worn = bio.worn;
    v.pulseTrusted = bio.pulseTrusted;
    bioDataUnlock();
    v.strobe = (v.mode == 1) || (v.pulseTrusted && displayBpm > 150.0f &&
                                 v.arousal > 0.65f);

    BleService::publishVitals(v);
  }

  // Signals. Paced by the render loop rather than a timer: at 60 fps there are ten
  // times more opportunities than the 5 packets/s the 25 Hz producer needs, so the
  // stream self-paces instead of racing a fixed period it cannot quite hit.
  //
  // Samples are copied out but NOT consumed until the notification is accepted. An
  // earlier version consumed first and ignored the return value, so every time the
  // stack's TX buffers were full a batch vanished -- 46 gaps and 15.6 Hz where 25 was
  // expected, while the timestamps still looked continuous.
  if (BleService::streamMask() & STREAM_SIGNALS) {
    BleSignalSample batch[BLE_SIGNALS_BATCH];
    uint32_t firstTs = 0;
    uint8_t n = 0;

    portENTER_CRITICAL(&sigMux);
    uint8_t peek = sigTail;
    while (n < BLE_SIGNALS_BATCH && peek != sigHead) {
      if (n == 0) firstTs = sigTimestamp[peek];
      batch[n++] = sigBuf[peek];
      peek = (uint8_t)((peek + 1) % SIGBUF_LEN);
    }
    portEXIT_CRITICAL(&sigMux);

    if (n == BLE_SIGNALS_BATCH && BleService::publishSignals(firstTs, batch, n)) {
      portENTER_CRITICAL(&sigMux);
      sigTail = peek;
      portEXIT_CRITICAL(&sigMux);
    }
  }

  // Spectrum: 1 Hz. binPowers() copies the bank, so it is done here on core 1 rather
  // than in the sampling task.
  if ((BleService::streamMask() & STREAM_SPECTRUM) &&
      now - lastSpectrumPublish >= 1000) {
    lastSpectrumPublish = now;
    static float binPower[N_BINS];
    pulse.binPowers(binPower);
    BleService::publishSpectrum(binPower, N_BINS, pulse.peakBin, BPM_MIN, BPM_MAX);
  }

  if (now - lastSerialPrint > 1000) {
    bioDataLock();
    float excitement = bio.gsrExcitement;
    float tempC = bio.tempC;
    uint8_t mode = bio.displayMode;
    bool worn = bio.worn;
    bool pulseTrusted = bio.pulseTrusted;
    float pi = bio.perfusion;
    uint16_t gsrRaw = bio.gsrRaw;
    bioDataUnlock();

    if (!worn) {
      Serial.print(F("[Standby] Not worn | PerfIdx: "));
      Serial.print(pi, 2);
      Serial.print(F("% | RawGSR: "));
      Serial.print(gsrRaw);
      Serial.print(F(" (worn needs "));
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
      Serial.print(pulseTrusted ? F("% (trusted)") : F("% (SEARCHING)"));
      Serial.print(F(" | Mode: "));
      Serial.println(mode);
    }

    lastSerialPrint = now;
  }

  if (now - lastJitterPrint > 10000) {
    portENTER_CRITICAL(&jitterMux);
    uint32_t jmin = jitter.minUs, jmax = jitter.maxUs, jn = jitter.count,
             jover = jitter.overruns;
    float jmean = jitter.meanUs();
    jitter.reset();
    portEXIT_CRITICAL(&jitterMux);

    if (jn) {
      Serial.print(F("[Jitter] target "));
      Serial.print(RAW_PERIOD_MS * 1000);
      Serial.print(F("us | min "));
      Serial.print(jmin);
      Serial.print(F(" mean "));
      Serial.print(jmean, 1);
      Serial.print(F(" max "));
      Serial.print(jmax);
      Serial.print(F(" | overruns "));
      Serial.print(jover);
      Serial.print(F("/"));
      Serial.println(jn);
    }
    lastJitterPrint = now;
  }

  delay(16); // 60 FPS
}
