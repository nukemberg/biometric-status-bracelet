/*
 * DSP v2 - Heartbeat & GSR Arousal Engine (validation sketch, no LEDs)
 *
 * Target MCU: Standard ESP32 WROOM-32 (PIN_GSR=34) or ESP32-S3. PPG comes from a
 * MAX30102 on I2C, not from the ADC -- see main_armband.ino for the wiring rationale.
 *
 * This sketch is the bench-validation counterpart to tools/dsp_v2_sim.py: same
 * sample rates, same coefficients, same update order, so anything verified offline
 * behaves identically here. It streams CSV over serial instead of driving the LED
 * matrix; once the numbers check out on-device, the two tracker blocks below port
 * straight into main_armband.ino.
 *
 * Serial output: Timestamp_ms,BPM,Confidence,Phase,Arousal,Tonic,RawIr,RawGSR
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
#include <Wire.h>
#include <max30102.h>
#include <math.h>

// ============================================================================
// MCU HARDWARE SELECTION (Uncomment for ESP32-S3)
// ============================================================================
//#define MCU_ESP32_S3

#ifdef MCU_ESP32_S3
  #define PIN_GSR         1
  #define PIN_SDA         8
  #define PIN_SCL         9
  #define PIN_BUTTON      5
#else // Standard ESP32 DevKit WROOM-32
  #define PIN_GSR         34
  #define PIN_SDA         21
  #define PIN_SCL         22
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
  uint32_t irRaw = 0;       // MAX30102 IR, 18-bit counts
  uint16_t gsrRaw = 0;
} bio;

portMUX_TYPE bioMux = portMUX_INITIALIZER_UNLOCKED;
inline void bioDataLock()   { portENTER_CRITICAL(&bioMux); }
inline void bioDataUnlock() { portEXIT_CRITICAL(&bioMux); }

PulseTracker pulse;
GsrTracker gsr;
Max30102 ppg;
bool ppgConnected = false;

// ============================================================================
// CORE 0 TASK: SAMPLING & DSP
// ============================================================================
volatile bool gsrResetRequest = false;

void TaskSensorDSP(void *pvParameters) {
  uint32_t accGsr = 0;
  uint8_t accN = 0;
  uint32_t lastIr = 0;
  uint16_t lastGsr = 0;

  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(RAW_PERIOD_MS);

  for (;;) {
    // GSR only. The boxcar below does the averaging, so the old 16-read burst is
    // redundant. PPG arrives from the MAX30102 FIFO at the decimation boundary.
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

      gsr.update(xg);

      uint32_t irBatch[8];
      uint8_t irCount = ppgConnected ? ppg.read(irBatch, nullptr, 8) : 0;
      for (uint8_t i = 0; i < irCount; i++) {
        lastIr = irBatch[i];
        pulse.update((float)lastIr);
      }

      bioDataLock();
      bio.arousal = gsr.arousal;
      bio.tonic = gsr.tonic;
      bio.irRaw = lastIr;
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

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  ppgConnected = ppg.begin(Wire);

  Serial.println(F("# DSP v2 validation sketch"));
  Serial.println(ppgConnected ? F("# MAX30102 ready")
                              : F("# MAX30102 NOT FOUND -- BPM will not track"));
  Serial.println(F("Timestamp_ms,BPM,Confidence,Phase,Arousal,Tonic,RawIr,RawGSR"));

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
    uint32_t rp = bio.irRaw;
    uint16_t rg = bio.gsrRaw;
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
