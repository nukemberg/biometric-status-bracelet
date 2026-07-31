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

// All pulse/GSR DSP lives in libraries/BraceletDSP -- the same code the production
// firmware runs and the same code tools/dsp_v2_parity.sh validates on the host.
// This sketch is only the harness around it.
#include <dsp.h>

// ============================================================================
// SHARED STATE (same portMUX pattern as main_armband.ino)
// ============================================================================
struct BiometricData {
  float bpm = DEFAULT_BPM;
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
