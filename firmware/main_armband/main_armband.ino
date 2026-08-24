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
 *
 * PPG front end: MAX30102, not the ADC
 * ------------------------------------
 * The analog pulse sensor is gone. PPG now comes from a MAX30102 on the same I2C bus
 * as the BME280 (addr 0x57 vs 0x76/0x77 -- no collision), delivering 18-bit IR counts
 * from its own FIFO at 25 Hz. GSR still runs the 500 Hz ADC and the 20-sample boxcar,
 * so this task keeps its 2 ms tick; the PPG channel is simply drained at the same
 * decimation boundary instead of being sampled and averaged.
 *
 * The resonator bank, the filters and every coefficient are unchanged -- the sample
 * rate into PulseTracker is still 25 Hz. What DID change is the meaning of the
 * numbers: raw counts, filtered amplitude and the perfusion index are all on a new
 * scale, so PI_TRUST_MIN and the SNR figures in DESIGN.md section 2 no longer apply
 * and samples/bio2.log and bio4.csv are no longer valid regression cases.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <FastLED.h>
#include <max30102.h>
#include <math.h>
#include <esp_system.h>   // esp_reset_reason() -- was this boot a brownout? see -iee

// ============================================================================
// MCU HARDWARE SELECTION
// ============================================================================
// ESP32-S3 is the deployment target; the WROOM-32 DevKit is the bench rig every
// capture in samples/ was taken on. GSR is the only analog sensor left and must stay
// on ADC1 -- ADC2 is unusable on both chips while the radio is active.
//
// PIN_PPG_INT is now wired and monitored, but only as a diagnostic -- the DSP task
// still polls the FIFO pointers at its 25 Hz decimation boundary rather than draining
// on the interrupt, since that read is I2C and cannot happen inside the ISR. The ISR
// just counts PPG_RDY edges (ppgIntCount) so the measured edge rate can be compared
// against ppgHz (which comes from the FIFO's own sample count) as a second, independent
// check on the MAX30102's oscillator. A real interrupt-driven drain -- ISR sets a flag,
// task drains only when set -- is still future work; polling every tick already costs
// only one 3-byte I2C read per 40 ms, so there is little to gain there.
#define MCU_ESP32_S3

#ifdef MCU_ESP32_S3
  #define PIN_GSR         1     // ADC1_CH0
  #define PIN_SDA         8
  #define PIN_SCL         9
  #define PIN_PPG_INT     10
  #define PIN_BUTTON      5
  #define PIN_LED         6     // moved off GPIO4 -- that pin measured damaged
  #define PIN_STATUS_LED  48    // onboard NeoPixel, S3 devkits only
#else // Standard ESP32 DevKit WROOM-32
  #define PIN_GSR         34
  #define PIN_SDA         21
  #define PIN_SCL         22
  #define PIN_PPG_INT     19
  #define PIN_BUTTON      18
  #define PIN_LED         4
#endif

// ESP32 input-high threshold, 0.75 x 3.3 V. An idle bus below this is not a bus.
#define I2C_VIH_MV      2480

#define NUM_LEDS        21
#define COLOR_ORDER     GRB
#define LED_TYPE        WS2812B

// The strip is glued to the case in a spiral, not a straight run: the data chain
// order is seg1 -> seg3 (physically the middle turn) -> seg2, and seg2's physical
// LED order runs opposite the data chain's, i.e. opposite its own visual
// left-to-right span. Each segment is its first physical LED index plus a
// direction (+1 forward, -1 reversed); segLed() maps a segment-local position
// (0..SEGMENT_LEN-1) to the physical index that actually needs to light. Direction
// only matters for a positional/gradient render -- a uniform fill across a segment
// is unaffected either way.
// Named A/B/C rather than by feature: which physiological reading each one shows
// depends on the caller (renderBiometricPanel uses A=cardiac/B=GSR/C=thermal;
// bootSelfTest uses the same three slots for GSR/pulse/temp status instead).
#define SEGMENT_LEN     7   // NUM_LEDS / 3
struct LedSegment { uint8_t base; int8_t dir; };
constexpr LedSegment SEG_A = {0, 1};    // LEDs 0-6
constexpr LedSegment SEG_B = {13, -1};  // LEDs 7-13, physically reversed
constexpr LedSegment SEG_C = {14, 1};   // LEDs 14-20

static inline uint8_t segLed(const LedSegment &seg, uint8_t i) {
  return (uint8_t)(seg.base + seg.dir * i);
}

CRGB leds[NUM_LEDS];

#ifdef PIN_STATUS_LED
// Onboard NeoPixel, driven as a second FastLED controller alongside the main strip
// (one FastLED.show() flushes both). Independent of worn/display-mode state, so it
// stays a useful "is this thing alive and connected" signal even in Standby.
CRGB statusLed[1];
#endif
Adafruit_BME280 bme;
bool bmeConnected = false;

// Shares SDA/SCL with the BME280. Addresses do not collide (0x57 vs 0x76/0x77).
Max30102 ppg;
bool ppgConnected = false;

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
  uint32_t pulseRaw = 0;    // MAX30102 IR, 18-bit counts
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

// PPG front-end health, written only by the DSP task and read only by the render loop
// for the periodic serial report -- the same single-writer pattern as the reset request
// flags above, so no lock is needed for two aligned 32-bit values.
//
// ppgHz is not a nicety. Every resonator bin frequency in dsp.h is derived from
// DSP_HZ = 25, but the MAX30102 clocks its FIFO off its own oscillator, so if this
// reads 25.6 then every BPM the device reports is 2.4 % low with nothing else looking
// wrong. Measuring it is the only way that error is ever visible. See DESIGN.md 4.2.
volatile float ppgHz = 0.0f;

// PPG_RDY edge count from PIN_PPG_INT, single-writer (ISR) / single-reader (main loop
// stat report) on a 32-bit word -- no lock needed, same pattern as ppgHz above.
volatile uint32_t ppgIntCount = 0;

void IRAM_ATTR onPpgInt() {
  ppgIntCount++;
}

// Measured on core 0, where a NimBLE controller task will later compete for time.
// Reported every 10 s and then reset, so each line describes a fresh window.
JitterMonitor jitter;
portMUX_TYPE jitterMux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================================
// SIGNAL STREAM RING BUFFER
// ============================================================================
// One entry per MAX30102 FIFO sample -- nominally 25 Hz, the same rate as before, but
// now paced by the sensor's oscillator rather than by this task's tick. The render
// loop drains five at a time and notifies at 5 Hz. Sized well above that so a late
// drain loses nothing -- dropping samples silently would put gaps in a stream whose
// entire purpose is offline analysis, and the timestamps would still look continuous.
//
// Pushing per PPG sample rather than per decimation tick keeps the pulse trace
// gapless even when a tick drains zero or two samples, which is exactly what the two
// clocks running independently will produce.
//
// Consequence worth knowing: the timestamp is when the sample was DRAINED, not when
// the sensor took it. A tick that hands over two samples stamps both within a few
// microseconds of each other, even though they are 40 ms apart in the sensor's own
// time. The samples themselves are real and correctly ordered; only the arrival time
// is bursty. Synthesising evenly spaced timestamps was rejected -- it would invent
// data that looks more precise than what we actually know.
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
  FastLED.setBrightness(value);
  cfg.brightness = (float)value;
  ConfigStore::save(cfg);
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
// ============================================================================
// BME280 SAMPLE VALIDATION
// ============================================================================
// The BME280 has been seen reporting physically impossible values (185 C is the
// one that showed up in the field). Adafruit_BME280 has no error return: a read
// that gets corrupted on the wire, or that lands while the chip is mid-reset,
// still comes back as a finite float computed from garbage raw counts. So the
// plausibility check has to live here.
//
// Two gates, both needed:
//   1. Absolute range. A worn armband's ambient never leaves 0..50 C or
//      0..100 %RH. Anything outside that is not a reading.
//   2. Slew rate. The sensor's thermal mass cannot move degrees in half a second,
//      so a corrupted sample that happens to land inside the range still shows up
//      as a jump. Only armed once one good sample has seeded it.
//
// A rejected sample is dropped, not clamped -- bio.tempC keeps the last good
// value, which is the honest answer across a 500 ms gap. A long run of rejects
// means the part or the bus is actually broken, so re-init rather than serve a
// frozen value forever.
static const float TEMP_MIN_C = 0.0f;
static const float TEMP_MAX_C = 50.0f;
static const float TEMP_MAX_SLEW_C = 5.0f;   // per 500 ms read interval
static const float HUMIDITY_MIN_PCT = 0.0f;
static const float HUMIDITY_MAX_PCT = 100.0f;
static const uint8_t BME_MAX_REJECTS = 10;   // 5 s of bad reads -> re-init

void TaskSensorDSP(void *pvParameters) {
  uint32_t accGsr = 0;
  uint8_t accN = 0;
  uint32_t lastIr = 0;
  uint16_t lastGsr = 0;
  unsigned long lastBmeRead = 0;
  float lastGoodTempC = 0.0f;
  bool bmeSeeded = false;
  bool bmeTriggered = false;
  uint8_t bmeRejects = 0;
  unsigned long lastPpgStat = 0;

  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(RAW_PERIOD_MS);

  for (;;) {
    // Guarded because the render loop on core 1 reads and resets these. The
    // critical section is a handful of instructions against a 2000 us period, but
    // it is on the very path being measured, so it is part of what gets reported.
    portENTER_CRITICAL(&jitterMux);
    jitter.tick(micros(), RAW_PERIOD_MS * 1000);
    portEXIT_CRITICAL(&jitterMux);

    // GSR only. One read per tick; the boxcar below does the averaging, so the old
    // 16-read burst was redundant work. PPG no longer touches the ADC at all.
    lastGsr = analogRead(PIN_GSR);
    accGsr += lastGsr;

    if (++accN >= DECIM) {
      float xg = (float)accGsr / (float)DECIM;
      accGsr = 0;
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
        bio.brightness = (uint8_t)(cfg.brightness + 0.5f);
        FastLED.setBrightness(bio.brightness);
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
          BleService::log("[Config] rejected param 0x%02X = %.4f", id, val);
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

      gsr.update(xg);

      // Drain whatever the MAX30102 has ready. At 25 Hz against a 25 Hz tick this is
      // normally one sample, occasionally zero or two as the two clocks drift past
      // each other -- all three are expected, none is an error. Eight is far more
      // headroom than that needs, and bounds the I2C burst so a stalled tick cannot
      // hold the bus for a whole FIFO's worth of data.
      uint32_t irBatch[8];
      uint8_t irCount = ppgConnected ? ppg.read(irBatch, nullptr, 8) : 0;

      for (uint8_t i = 0; i < irCount; i++) {
        lastIr = irBatch[i];
        pulse.update((float)lastIr);

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
          sigBuf[sigHead].pulseFiltered = pulse.filtered;
          sigBuf[sigHead].gsrPhasic = gsr.smooth - gsr.tonic;
          sigBuf[sigHead].pulseRaw = lastIr;
          sigBuf[sigHead].gsrRaw = lastGsr;
          sigTimestamp[sigHead] = millis();
          sigHead = next;
          portEXIT_CRITICAL(&sigMux);
        }
      }

      float pi = pulse.perfusion();
      bool worn = wear.update(lastGsr, pi, (uint32_t)millis());
      // The state machine reports the release; acting on it is ours to do, so a
      // fresh wearer never sees the previous one's rate while the bank reconverges.
      if (wear.justReleased) pulse.clearBank();

      bioDataLock();
      bio.gsrExcitement = gsr.arousal;
      bio.gsrTonic = gsr.tonic;
      bio.pulseRaw = lastIr;
      bio.gsrRaw = lastGsr;
      bio.perfusion = pi;
      bio.worn = worn;
      bio.pulseTrusted = wear.pulseTrusted;
      bioDataUnlock();
    }

    // PPG rate/overflow census, kept on this task so nothing else touches the driver.
    if (ppgConnected && (millis() - lastPpgStat > 10000)) {
      lastPpgStat = millis();
      float hz = ppg.measuredHz((uint32_t)millis());
      if (hz > 0.0f) ppgHz = hz;
    }

    if (bmeConnected && (millis() - lastBmeRead > 500)) {
      lastBmeRead = millis();

      // Two-phase, because a forced conversion takes about 10 ms and this task owes
      // its next tick in 2. takeForcedMeasurement() cannot cover that: it polls the
      // status register's measuring bit, which this part never sets, so it returns
      // immediately rather than waiting. So trigger here and read on the next pass
      // 500 ms later, by which time the conversion is long finished. Cost is one
      // read interval of latency on a signal that moves in minutes.
      if (!bmeTriggered) {
        // Nothing converted yet -- trigger only. (Not `continue`: the tail of this
        // loop is the vTaskDelayUntil that gives the tick back.)
        bme.takeForcedMeasurement();
        bmeTriggered = true;
      } else {
        float t = bme.readTemperature();
        float h = bme.readHumidity();
        bme.takeForcedMeasurement();   // start the one we will read next time

        bool tOk = !isnan(t) && t >= TEMP_MIN_C && t <= TEMP_MAX_C &&
                   (!bmeSeeded || fabsf(t - lastGoodTempC) <= TEMP_MAX_SLEW_C);
        bool hOk = !isnan(h) && h >= HUMIDITY_MIN_PCT && h <= HUMIDITY_MAX_PCT;

        if (tOk) {
          lastGoodTempC = t;
          bmeSeeded = true;
        }
        if (tOk || hOk) {
          bioDataLock();
          if (tOk) bio.tempC = t;
          if (hOk) bio.humidity = h;
          bioDataUnlock();
        }

        if (tOk && hOk) {
          bmeRejects = 0;
        } else {
          // Only the first reject of a run is logged: a stuck sensor read at 2 Hz
          // would otherwise flush the 20-entry log ring in ten seconds and bury
          // whatever else went wrong.
          if (bmeRejects == 0) {
            BleService::log("[bme] rejected T=%.1fC RH=%.0f%% (last good %.1fC)",
                            t, h, bmeSeeded ? lastGoodTempC : 0.0f);
          }
          if (++bmeRejects >= BME_MAX_REJECTS) {
            bmeRejects = 0;
            bmeSeeded = false;
            // Same task that owns every other I2C transfer, so re-init here is safe.
            bmeConnected = bme.begin(0x76, &Wire) || bme.begin(0x77, &Wire);
            if (bmeConnected) {
              // begin() resets the chip to the library's default normal mode, which
              // this part will not hold -- put it back into forced mode.
              bme.setSampling(Adafruit_BME280::MODE_FORCED,
                              Adafruit_BME280::SAMPLING_X1,
                              Adafruit_BME280::SAMPLING_NONE,
                              Adafruit_BME280::SAMPLING_X1,
                              Adafruit_BME280::FILTER_OFF);
            }
            bmeTriggered = false;
            BleService::log("[bme] %u consecutive bad reads -- re-init %s",
                            BME_MAX_REJECTS, bmeConnected ? "OK" : "FAILED");
          }
        }
      }
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
// BIOFEEDBACK BREATHING PACER (mode 3)
// ============================================================================
// GSR-adaptive, not a fixed metronome: half-cycle (inhale or exhale) duration shortens
// as arousal rises and lengthens as it settles, so the pacer follows the wearer then
// gently invites a slower breath as GSR shows they're calming. Mirrored independently
// in webapp/index.html (BREATH_HALF_CYCLE_*_S there) -- same idea, not a shared phase
// over the wire, so the two will drift apart in exact timing, not just cosmetically.
#define BREATH_HALF_CYCLE_AROUSED_S 3.0f
#define BREATH_HALF_CYCLE_CALM_S    6.0f
float breathPhase = 0.0f;
uint32_t lastBreathMs = 0;

void advanceBreathPhase(float excitement) {
  uint32_t now = millis();
  float dt = (lastBreathMs == 0) ? 0.0f : (now - lastBreathMs) / 1000.0f;
  lastBreathMs = now;
  if (dt <= 0.0f || dt > 1.0f) dt = 0.02f;   // first frame / after a stall

  float halfCycle = BREATH_HALF_CYCLE_AROUSED_S +
      (BREATH_HALF_CYCLE_CALM_S - BREATH_HALF_CYCLE_AROUSED_S) * (1.0f - excitement);
  breathPhase += ((float)M_PI / halfCycle) * dt;
  breathPhase = fmodf(breathPhase, 2.0f * (float)M_PI);
}

// Full-panel render, not the usual 3-segment split. Two earlier attempts read wrong on
// real hardware: a uniform brightness pulse across all 21 LEDs had nothing directional
// to follow, and a 1-D expanding bar (even once centered on the right physical LED)
// looked like two arcs sliding apart rather than one shape. The physical mount is
// genuinely a 3 (rows) x 7 (cols) rectangle -- SEG_A/SEG_C/SEG_B are the three rows,
// SEG_C the physical center row -- so this grows a pixelated ovaloid blob outward from
// the rectangle's center in both dimensions at once, elliptical rather than circular
// so it reaches all four "corners" together despite the 7:3 aspect ratio. Mirrors the
// expanding/contracting circle in webapp/index.html as the same gesture, not the same
// shape -- a 3x7 grid can't read as a circle, pixelated is the honest version of it.
// Hue is the GSR VU-meter's own range (96 green -> 240 purple) for palette consistency.
static inline void paintBreathPixel(const LedSegment &seg, int row, int col,
                                    float radius, uint8_t hue) {
  uint8_t i = segLed(seg, (uint8_t)col);
  float dr = (float)abs(row - 1);              // rows are 0,1,2; center row is 1
  float dc = (float)abs(col - 3) / 3.0f;        // cols are 0..6; center col is 3
  float dist = sqrtf(dr * dr + dc * dc);
  if (dist <= radius) {
    float edgeFade = radius > 0.0f ? 1.0f - 0.4f * (dist / radius) : 1.0f;
    leds[i] = CHSV(hue, 255, (uint8_t)(210.0f * edgeFade));
  } else {
    leds[i] = CRGB(2, 2, 6);   // dim backdrop, not fully black
  }
}

void renderBreathing(float excitement) {
  advanceBreathPhase(excitement);
  float env = 0.5f - 0.5f * cosf(breathPhase);   // 0 at cycle start (empty), 1 at the peak (full)
  uint8_t hue = (uint8_t)(96.0f + 144.0f * clampf(excitement, 0.0f, 1.0f));

  const float maxDist = 1.41421356f;   // sqrt(2): normalized distance to a far corner
  float radius = env * maxDist;
  for (int col = 0; col < SEGMENT_LEN; col++) {
    paintBreathPixel(SEG_A, 0, col, radius, hue);
    paintBreathPixel(SEG_C, 1, col, radius, hue);
    paintBreathPixel(SEG_B, 2, col, radius, hue);
  }
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

  // Biofeedback mode replaces the usual 3-segment split with one full-panel
  // breathing mark -- returns early rather than falling into the segment renders
  // below, which would just be immediately overwritten.
  if (mode == 3) { renderBreathing(excitement); return; }

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
    for (uint8_t i = 0; i < SEGMENT_LEN; i++) leds[segLed(SEG_A, i)] = CHSV(0, 0, v);
  } else {
    // Warm gradient: orange/yellow when slow, deep red as it climbs, pink at the top.
    uint8_t pulseHue = pulseHueFor(displayBpm);

    // Low confidence desaturates toward white rather than freezing the animation, so
    // a marginal signal reads as "unsure" instead of as a dead panel.
    uint8_t sat = 120 + (uint8_t)(135.0f * fminf(1.0f, confidence / CONF_REF));

    for (uint8_t i = 0; i < SEGMENT_LEN; i++) {
      float dist = fabsf((float)i - 3.0f);
      uint8_t v = beatEnvelope(beatPhase - dist * WAVE_LAG);
      leds[segLed(SEG_A, i)] = CHSV(pulseHue, sat, v);
    }
  }

  // --------------------------------------------------------------------------
  // SEGMENT 2: GSR EXCITEMENT VU-METER (LEDs 7 to 13, physically reversed --
  // see SEG_B above)
  // --------------------------------------------------------------------------
  uint8_t litLedsGSR = (uint8_t)(excitement * 7.0 + 0.5);
  if (litLedsGSR > SEGMENT_LEN) litLedsGSR = SEGMENT_LEN;

  for (uint8_t i = 0; i < SEGMENT_LEN; i++) {
    uint8_t ledIndex = segLed(SEG_B, i);
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

  for (uint8_t i = 0; i < SEGMENT_LEN; i++) {
    uint8_t ledIndex = segLed(SEG_C, i);
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
// ONBOARD STATUS LED (BLE connection + error flash)
// ============================================================================
// Runs every frame regardless of worn/mode/standby state -- unlike the main strip,
// this is meant to answer "is the radio up, connected, and has anything gone wrong"
// at a glance, so it stays live in every display state including Standby.
#ifdef PIN_STATUS_LED
#define STATUS_FLASH_MS 400   // how long a new log entry stays red
void renderStatusLed() {
  uint32_t now = millis();
  if (now - BleService::lastLogMs() < STATUS_FLASH_MS) {
    statusLed[0] = CRGB(60, 0, 0);          // new error/warning -- flash red
  } else if (BleService::isConnected()) {
    statusLed[0] = CRGB(0, 25, 0);          // connected -- dim solid green
  } else {
    // Advertising, nobody connected -- dim breathing blue, ~0.5 Hz.
    float breath = 0.5f * (1.0f + sinf((float)now * 0.0031f));
    statusLed[0] = CRGB(0, 0, 4 + (uint8_t)(18.0f * breath));
  }
}
#endif

// ============================================================================
// BOOT SELF-TEST
// ============================================================================
// All LEDs blue for 0.5s (strip/wiring sanity -- every pixel should visibly light),
// then 2s of per-segment red/green sensor status: segment 1 = GSR, segment 2 = pulse
// (MAX30102), segment 3 = temp (BME280). GSR has no self-ID like an I2C device, so
// "pass" there just means the raw ADC reading is off both rails (not pinned at 0 or
// 4095, i.e. wired to something rather than floating open or shorted) -- the same
// check used in firmware/led_test.
void bootSelfTest() {
  FastLED.showColor(CRGB(0, 0, 255));
  delay(500);

  uint16_t gsrRaw = analogRead(PIN_GSR);
  bool gsrOk = gsrRaw > 5 && gsrRaw < 4090;

  CRGB gsrColor = gsrOk ? CRGB(0, 255, 0) : CRGB(255, 0, 0);
  CRGB pulseColor = ppgConnected ? CRGB(0, 255, 0) : CRGB(255, 0, 0);
  CRGB tempColor = bmeConnected ? CRGB(0, 255, 0) : CRGB(255, 0, 0);

  // Uniform per-segment fill -- direction doesn't matter here, only which physical
  // LEDs belong to which segment, so segLed() with any i-order covers it.
  for (uint8_t i = 0; i < SEGMENT_LEN; i++) leds[segLed(SEG_A, i)] = gsrColor;
  for (uint8_t i = 0; i < SEGMENT_LEN; i++) leds[segLed(SEG_B, i)] = pulseColor;
  for (uint8_t i = 0; i < SEGMENT_LEN; i++) leds[segLed(SEG_C, i)] = tempColor;
  FastLED.show();

  BleService::log("[boot] self-test | GSR raw=%u %s | Pulse %s | Temp %s",
                  gsrRaw, gsrOk ? "OK" : "FAIL",
                  ppgConnected ? "OK" : "FAIL",
                  bmeConnected ? "OK" : "FAIL");

  delay(2000);
  FastLED.clear();
  FastLED.show();
}

// ============================================================================
// I2C BUS DIAGNOSTIC
// ============================================================================
// Both I2C parts going missing at once is a bus fault, not two dead sensors
// (DESIGN.md 1) -- and the distinct bus faults this build can hit are
// indistinguishable from the bmeConnected/ppgConnected booleans alone. So measure the
// idle rail before Wire takes the pins, then scan the bus, and put both into the log
// ring. The ring is replayed to the app on connect, so a failed boot self-reports
// over BLE instead of needing a meter on the bench.
//
//   SDA and SCL both ~3.3 V    bus healthy
//   both ~1.8 V                MH-ET pull-up jumper sits on the 1V8 pad (DESIGN.md 1)
//   SDA ~0 V, SCL ~3.3 V       a slave is stuck mid-byte across an MCU reset and will
//                              hold SDA down until clocked far enough to release it
//   both ~0 V                  no pull-ups fitted, or the sensor rail is down
// Standard I2C bus-recovery clock-out (I2C spec, "Bus Clear" / Fig. 12 in the UM10204
// application note): pulse SCL up to 9 times -- enough to complete any byte a slave
// could be mid-way through -- while SDA is read-only, then issue a STOP condition
// (SDA low -> high while SCL is held high) to reset every slave's state machine to
// idle. Runs only for the specific "a slave is stuck mid-byte across an MCU reset"
// signature i2cIdleProbe() below identifies (SDA low, SCL fine) -- clocking a bus
// that is down for some other reason (no pull-ups, wrong rail) does nothing useful
// and risks disturbing a device that was never actually stuck.
static void i2cBusRecovery() {
  pinMode(PIN_SCL, OUTPUT);
  digitalWrite(PIN_SCL, HIGH);
  pinMode(PIN_SDA, INPUT);

  int pulses = 0;
  for (; pulses < 9 && !digitalRead(PIN_SDA); pulses++) {
    digitalWrite(PIN_SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(PIN_SCL, HIGH);
    delayMicroseconds(5);
  }

  // STOP: SDA low -> high while SCL is high (true here regardless of how many
  // pulses ran above -- the loop always leaves SCL HIGH as its last action).
  pinMode(PIN_SDA, OUTPUT);
  digitalWrite(PIN_SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_SDA, HIGH);
  delayMicroseconds(5);

  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);

  BleService::log("[i2c] bus recovery: %d clock pulse%s, SDA %s",
                  pulses, pulses == 1 ? "" : "s",
                  digitalRead(PIN_SDA) ? "released" : "STILL STUCK");
}

static void i2cIdleProbe() {
  // Read before Wire.begin(), which takes the pins over as open-drain outputs.
  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);
  delayMicroseconds(100);   // let the external pull-ups settle after the pinMode

#ifdef MCU_ESP32_S3
  // GPIO8 = ADC1_CH7 and GPIO9 = ADC1_CH8 on the S3, so read the actual idle voltage.
  // digitalRead cannot do this job: 1.8 V and 0 V both read LOW against the 2.48 V
  // V_IH, and those two are different faults with different fixes.
  uint32_t sda = analogReadMilliVolts(PIN_SDA);
  uint32_t scl = analogReadMilliVolts(PIN_SCL);
  BleService::log("[i2c] idle SDA=%umV SCL=%umV", (unsigned)sda, (unsigned)scl);
  if (sda < I2C_VIH_MV || scl < I2C_VIH_MV) {
    BleService::log("[i2c] idle under V_IH %umV -- bus fault, see DESIGN.md 1",
                    (unsigned)I2C_VIH_MV);
    if (sda < I2C_VIH_MV && scl >= I2C_VIH_MV) i2cBusRecovery();
  }
#else
  // GPIO21/22 on the classic ESP32 are not ADC pins; a logic level is all there is.
  bool sdaHigh = digitalRead(PIN_SDA);
  bool sclHigh = digitalRead(PIN_SCL);
  BleService::log("[i2c] idle SDA=%s SCL=%s", sdaHigh ? "HIGH" : "LOW",
                  sclHigh ? "HIGH" : "LOW");
  if (!sdaHigh && sclHigh) i2cBusRecovery();
#endif
}

// Address-only scan of the 7-bit range, run after Wire.begin() and before any driver
// touches the bus. Logs one line whatever the outcome, so the absence of a device is
// as visible as its presence -- 0x57 is the MAX30102, 0x76/0x77 the BME280.
// Wire.endTransmission() error codes (ESP32 core): 0 success, 1 data too long for the
// TX buffer, 2 NACK on the address byte, 3 NACK on a data byte, 4 other error, 5
// timeout. A scan of an empty bus is normal and returns 2 (NACK-address) at every
// unpopulated address -- that alone is not a fault. 4 or 5 at more than a couple of
// addresses IS: it means the bus itself is stuck or busy, not just quiet, which is a
// different problem with a different fix (see -iee for a broken-solder-joint case
// that produced exactly this signature).
static void i2cScan() {
  char found[64];
  size_t n = 0;
  uint8_t count = 0;
  uint8_t errCount[6] = {0};   // indexed by the endTransmission() code, 0..5
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err <= 5) errCount[err]++;
    if (err != 0) continue;
    count++;
    if (n < sizeof found - 6) {
      n += snprintf(found + n, sizeof found - n, "%s0x%02X", n ? " " : "", addr);
    }
  }
  if (count == 0) {
    BleService::log("[i2c] scan found nothing -- bus is down, not the sensors");
  } else {
    BleService::log("[i2c] scan found %u: %s", (unsigned)count, found);
  }
  // NACK-on-address (code 2) at every other address is the routine "nothing there"
  // response and not worth a line on its own -- only log when something else showed
  // up, since that is the part that actually needs explaining.
  uint8_t other = errCount[1] + errCount[3] + errCount[4] + errCount[5];
  if (other > 0) {
    BleService::log("[i2c] scan errors: len=%u dataNACK=%u other=%u timeout=%u",
                    errCount[1], errCount[3], errCount[4], errCount[5]);
  }
}

// Was this boot a brownout? The hardware brownout detector latches its own reset
// reason distinctly from a normal power-on, so this settles it without a meter --
// see the -iee comment on i2cIdleProbe()/tryInitSensors() above for why that matters.
static const char *resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT (reset pin/button)";
    case ESP_RST_SW:        return "SW (esp_restart or upload)";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT (other)";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP wake";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

// Attempts BME280 + MAX30102 init, sets bmeConnected/ppgConnected, returns whether
// either answered. Callable more than once -- setup() retries it once after a settle
// delay if both come up missing, see the i2c-bus-fault comment at the call site.
static bool tryInitSensors() {
  bmeConnected = bme.begin(0x76, &Wire) || bme.begin(0x77, &Wire);
  if (bmeConnected) {
    // FORCED, not the library's default normal mode. Bench-measured on this part
    // (firmware/bme_test): a write of mode=normal to ctrl_meas is ACKed and reads
    // back correctly, then clears itself to sleep within 1 ms. The chip therefore
    // never converts, the raw registers keep their 0x80000 power-on value, and
    // readTemperature() returns a constant fabricated ~21.5 C forever -- a plausible
    // room temperature, which is why it went unnoticed. Forced mode is retained and
    // converts correctly on the same part, so every reading is explicitly triggered.
    // Pressure is skipped: nothing here uses it, and skipping it shortens the
    // conversion and cuts self-heating.
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1,       // temperature
                    Adafruit_BME280::SAMPLING_NONE,     // pressure -- unused
                    Adafruit_BME280::SAMPLING_X1,       // humidity
                    Adafruit_BME280::FILTER_OFF);
  }
  ppgConnected = ppg.begin(Wire);
  return bmeConnected || ppgConnected;
}

// ============================================================================
// MAIN SETUP & LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  // First line out, before anything else can obscure it. A BROWNOUT here means the
  // BOD tripped on THIS boot specifically -- ties any I2C fault seen on the same
  // boot straight to -av5/-iee instead of leaving it a coincidence.
  BleService::log("[boot] reset reason: %s", resetReasonName(esp_reset_reason()));

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  // GSR is the only ADC channel left; the MAX30102 brings its own 18-bit converter.
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  cfg = ConfigStore::load();
  pulse.begin();
  pulse.applyConfig(cfg);
  gsr.begin();
  wear.applyConfig(cfg);
  bio.brightness = (uint8_t)(cfg.brightness + 0.5f);

  // Brownout staging (-av5): the BLE radio and the WS2812 RMT driver are the two
  // highest-inrush peripherals. On-device testing showed the brownout (E BOD)
  // fires right at radio init, never at the LED driver -- but ONLY when the LED
  // strip is physically connected. The strip's input capacitance on the data GPIO
  // (even high-Z) loads the rail enough to tip the radio's inrush over the brownout
  // threshold. Fix: bring the radio up FIRST with the LED data line still floating,
  // then init the RMT driver once the radio is stable and advertising.
  // Root cause still wants a real current measurement; see -av5.
  //
  // I2C init is back here, before both, matching that original order. A 2026-08-24
  // -iee experiment moved it to after radio+LED instead, on the theory that the
  // SDA-stuck-low boot fault was a depressed rail from the same inrush this staging
  // dodges -- moving it later made no difference (fault reproduced at the same rate
  // regardless), and the actual root cause turned out to be a broken SDA solder
  // joint, unrelated to init order or power sequencing at all. See -iee.

  i2cIdleProbe();

  Wire.begin(PIN_SDA, PIN_SCL);
  // 400 kHz. The bus now carries the 25 Hz PPG drain as well as the 2 Hz BME280 read;
  // at 100 kHz a 6-byte FIFO burst plus its pointer read is a meaningful slice of the
  // 40 ms decimation budget on the same task that has to hit a 2 ms tick.
  Wire.setClock(400000);
  i2cScan();

  if (!tryInitSensors()) {
    // Both missing after a clean scan is the bus-fault signature (DESIGN.md 1). Root
    // cause on this build turned out to be a broken SDA solder joint (-iee) -- not
    // power sequencing or a stuck-slave protocol state -- so this retry mostly just
    // gives an intermittent joint one more chance to make contact. Costs nothing on
    // a healthy boot, since this branch only runs when both are already missing.
    BleService::log("[i2c] both sensors missing -- retrying after settle");
    delay(100);
    tryInitSensors();
  }

  if (ppgConnected) {
    Serial.println(F("[boot] MAX30102 ready (IR, 25 Hz FIFO)"));
  } else {
    BleService::log("[boot] MAX30102 NOT FOUND -- pulse will not track");
  }

  if (ppgConnected) {
    // MAX30102's INT is open-drain, active-low, and the breakout's own pull-up
    // rail feeds it (same rail as SDA/SCL -- see DESIGN.md 1), so plain INPUT here,
    // not INPUT_PULLUP, to avoid fighting that external pull-up.
    pinMode(PIN_PPG_INT, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_PPG_INT), onPpgInt, FALLING);
  }

  // Started after the DSP task so the jitter monitor has a clean window before the
  // radio exists; -a75 compares the two.
  xTaskCreatePinnedToCore(TaskSensorDSP, "SensorDSP", 4096, NULL, 2, NULL, 0);

  Serial.println(F("[boot] starting radio"));
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

  // Radio is up and advertising. NOW init the LED strip -- the RMT driver's GPIO
  // configuration is what interacts with the strip's input capacitance, so
  // deferring it until after the radio's inrush has settled avoids the brownout.
  Serial.println(F("[boot] init LED driver"));
  FastLED.addLeds<LED_TYPE, PIN_LED, COLOR_ORDER>(leds, NUM_LEDS);
#ifdef PIN_STATUS_LED
  // Same FastLED engine, second controller -- one show() flushes both. NOTE:
  // FastLED.setBrightness() below is global across every registered controller, so
  // the BLE brightness slider dims this too; setPixelColor values are already kept
  // low so it stays subtle rather than glary at full brightness.
  FastLED.addLeds<LED_TYPE, PIN_STATUS_LED, GRB>(statusLed, 1);
#endif
  FastLED.setBrightness(bio.brightness);
  FastLED.clear();
  FastLED.show();

  bootSelfTest();

  Serial.println(F("[boot] boot complete"));
}

unsigned long lastSerialPrint = 0;
unsigned long lastFrameMs = 0;
unsigned long lastJitterPrint = 0;
uint32_t lastPpgIntCount = 0;
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
#ifdef PIN_STATUS_LED
  renderStatusLed();
#endif
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
    uint32_t irRaw = bio.pulseRaw;
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
      Serial.print(F(") | IR: "));
      Serial.print(irRaw);
      // Temperature only appeared on the worn line, so a sensor stuck at a
      // plausible-looking constant was invisible on a bench device that is never
      // worn -- which is how a BME280 that never converted went unnoticed.
      Serial.print(F(" | Temp: "));
      Serial.print(tempC, 2);
      Serial.println(F(" C"));
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
      Serial.print(F(" | IR: "));
      Serial.print(irRaw);
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

    // Reported next to the jitter line because it answers the same question for the
    // other front end: is the sample clock the DSP assumes actually the one it gets.
    if (ppgConnected) {
      // Rate is the health metric, not an overflow count -- this part's OVF_COUNTER
      // is not a loss counter (see max30102.h). A dip below ~25 means samples were
      // lost to a stall; a steady offset from 25 scales every BPM by that ratio.
      Serial.print(F("[PPG] fifo "));
      Serial.print(ppgHz, 2);
      Serial.print(F(" Hz (dsp assumes "));
      Serial.print(DSP_HZ);
      Serial.println(F(")"));
      // Independent cross-check on the same oscillator: PPG_RDY edges counted on
      // PIN_PPG_INT over this window, converted to Hz. Should track ppgHz above --
      // a persistent mismatch means the FIFO-pointer math and the interrupt line
      // disagree, which points at the ISR/wiring rather than the oscillator itself.
      uint32_t intCount = ppgIntCount;  // volatile, single read
      uint32_t intDelta = intCount - lastPpgIntCount;
      lastPpgIntCount = intCount;
      Serial.print(F("[PPG] int  "));
      Serial.print(intDelta / 10.0f, 2);
      Serial.println(F(" Hz (PPG_RDY edges on PIN_PPG_INT)"));
    } else {
      Serial.println(F("[PPG] MAX30102 absent -- BPM is frozen at its last value"));
    }
    lastJitterPrint = now;
  }

  delay(16); // 60 FPS
}
