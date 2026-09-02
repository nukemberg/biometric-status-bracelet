/*
 * High-Speed Raw Biometric Streamer for offline DSP analysis
 * Target MCU: Standard ESP32 (PIN_GSR=34) or ESP32-S3
 *
 * Captures the two front ends exactly as the firmware sees them, with no software
 * filtering and no interpolation, so tools/dsp_v2_sim.py and the parity harness can
 * replay a recording through the real trackers.
 *
 *   - GSR:  500 Hz analog, one analogRead per 2 ms tick. Decimated 20:1 downstream.
 *   - PPG:  MAX30102 IR, delivered by the sensor's own FIFO at ~25 Hz.
 *
 * Output: Timestamp_ms,RawIr,RawGSR,IrNew
 *
 * The two channels no longer share a clock, so the CSV carries a per-row flag instead
 * of pretending they do. IrNew=1 means RawIr on that row is a genuinely new FIFO
 * sample; IrNew=0 means no PPG sample landed in that 2 ms window and RawIr is 0 and
 * must be ignored -- NOT held over, NOT interpolated. Consumers feed
 * PulseTracker.update() once per IrNew=1 row and run the GSR boxcar over every row.
 *
 * Emitting a zero-order hold instead was considered and rejected. The 20-sample GSR
 * boxcar would then have averaged across IR sample boundaries whenever the two clocks
 * drifted out of alignment, which is an extra low-pass that exists in the capture and
 * not on the device -- and it would have made the C++/Python parity check inexact in a
 * way that looks like a rounding difference rather than a structural one.
 *
 * Bandwidth: ~13 bytes/row at 500 Hz is 6.5 kB/s, comfortably inside 115200 baud.
 * That is why RawIr is zeroed rather than repeated on hold rows -- one character
 * instead of six, on 95 % of rows.
 */

#include <Arduino.h>
#include <Wire.h>
#include <max30102.h>

#define MCU_ESP32_S3

#ifdef MCU_ESP32_S3
  #define PIN_GSR     1
  #define PIN_SDA     8
  #define PIN_SCL     9
  #define PIN_BUTTON  5
#else // Standard ESP32 DevKit WROOM-32
  #define PIN_GSR     34
  #define PIN_SDA     21
  #define PIN_SCL     22
  #define PIN_BUTTON  18
#endif

Max30102 ppg;
bool ppgConnected = false;

// The FIFO can hand over more than one sample in a single 2 ms tick, but a row can
// only carry one. Queue them so every sample the sensor produced appears in the
// capture on its own row -- dropping the extras would put invisible gaps in the PPG
// trace while the timestamps stayed continuous, which is the failure mode this whole
// capture path exists to avoid.
#define IRQ_LEN 64
uint32_t irQueue[IRQ_LEN];
uint8_t irHead = 0, irTail = 0;

inline bool irQueuePush(uint32_t v) {
  uint8_t next = (uint8_t)((irHead + 1) % IRQ_LEN);
  if (next == irTail) return false;   // full; caller reports it
  irQueue[irHead] = v;
  irHead = next;
  return true;
}

inline bool irQueuePop(uint32_t &v) {
  if (irTail == irHead) return false;
  v = irQueue[irTail];
  irTail = (uint8_t)((irTail + 1) % IRQ_LEN);
  return true;
}

uint32_t lostSamples = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  analogReadResolution(12);   // 0 - 4095
  analogSetAttenuation(ADC_11db);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  ppgConnected = ppg.begin(Wire);

  delay(1000);

  Serial.println(ppgConnected ? "# MAX30102 ready"
                              : "# MAX30102 NOT FOUND -- IR column will be empty");
  Serial.println("Timestamp_ms,RawIr,RawGSR,IrNew");
}

void loop() {
  unsigned long startUs = micros();
  unsigned long nowMs = millis();

  uint16_t rawGsr = analogRead(PIN_GSR);

  // Drain the sensor, then release at most one queued sample onto this row.
  if (ppgConnected) {
    uint32_t batch[8];
    uint8_t n = ppg.read(batch, nullptr, 8);
    for (uint8_t i = 0; i < n; i++) {
      if (!irQueuePush(batch[i])) lostSamples++;
    }
  }

  uint32_t ir = 0;
  bool irNew = irQueuePop(ir);

  Serial.print(nowMs);
  Serial.print(',');
  Serial.print(ir);
  Serial.print(',');
  Serial.print(rawGsr);
  Serial.print(',');
  Serial.println(irNew ? 1 : 0);

  // A capture with dropped PPG samples is not a valid regression case, so say so in
  // the stream rather than letting it pass as clean data.
  //
  // Only the local queue is checked. The sensor's own OVF_COUNTER is not a loss
  // counter on this part (see max30102.h) and reads non-zero in normal operation;
  // warning on it would put a WARNING line in every otherwise-perfect capture, which
  // trains you to ignore the warning. Sensor-side loss shows up as a sample count
  // below 25/s, which dsp_v2_sim.py reports per capture.
  static unsigned long lastWarn = 0;
  if (lostSamples && nowMs - lastWarn > 5000) {
    lastWarn = nowMs;
    Serial.print("# WARNING dropped PPG samples in capture queue: ");
    Serial.println(lostSamples);
  }

  // Strict 500.0 Hz timing (2,000 us per frame).
  while (micros() - startUs < 2000) {
    // High-resolution hardware wait loop
  }
}
