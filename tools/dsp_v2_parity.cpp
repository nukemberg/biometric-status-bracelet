// Host-side parity harness for firmware/dsp_v2/dsp_v2.ino.
//
// Includes the real sketch (with the Arduino/FreeRTOS surface stubbed out) and
// replays a raw_streamer CSV through the actual PulseTracker/GsrTracker code, so the
// firmware and tools/dsp_v2_sim.py can be shown to agree numerically rather than
// merely to look alike.
//
// Build & run:  tools/dsp_v2_parity.sh bio2.log

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

// --- Arduino / FreeRTOS stubs ----------------------------------------------
#define ADC_11db 3
#define INPUT_PULLUP 2
#define HIGH 1
#define LOW 0
#define F(s) (s)

typedef uint32_t TickType_t;
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)0)
#define portEXIT_CRITICAL(x) ((void)0)
#define pdMS_TO_TICKS(ms) (ms)

static inline TickType_t xTaskGetTickCount() { return 0; }
static inline void vTaskDelayUntil(TickType_t *, TickType_t) {}
static inline void xTaskCreatePinnedToCore(void (*)(void *), const char *, int,
                                           void *, int, void *, int) {}
static inline int analogRead(int) { return 0; }
static inline void analogReadResolution(int) {}
static inline void analogSetAttenuation(int) {}
static inline void pinMode(int, int) {}
static inline int digitalRead(int) { return HIGH; }
static inline unsigned long millis() { return 0; }
static inline void delay(unsigned long) {}

struct FakeSerial {
  void begin(long) {}
  void print(const char *) {}
  void print(unsigned long) {}
  void print(char) {}
  void print(float, int) {}
  void print(int) {}
  void println(const char *) {}
  void println(unsigned long) {}
  void println(float, int) {}
} Serial;

#include "../firmware/dsp_v2/dsp_v2.ino"

// ---------------------------------------------------------------------------
int main(int argc, char **argv) {
  if (argc < 2) {
    fprintf(stderr, "usage: %s <raw_streamer.csv>\n", argv[0]);
    return 2;
  }
  FILE *fh = fopen(argv[1], "r");
  if (!fh) { perror(argv[1]); return 1; }

  pulse.begin();
  gsr.begin();

  char line[256];
  uint32_t accP = 0, accG = 0;
  int accN = 0;
  double t0 = -1, tPrevEst = 0, tLastReport = 0;

  while (fgets(line, sizeof(line), fh)) {
    double ts, rp, rg;
    if (sscanf(line, "%lf,%lf,%lf", &ts, &rp, &rg) != 3) continue;  // header
    if (t0 < 0) { t0 = ts; tPrevEst = ts; tLastReport = ts; }

    accP += (uint32_t)rp;
    accG += (uint32_t)rg;
    if (++accN < DECIM) continue;

    pulse.update((float)accP / (float)DECIM);
    gsr.update((float)accG / (float)DECIM);
    accP = accG = 0;
    accN = 0;

    // Mirror the sim: estimate once per decimated sample.
    pulse.estimate((float)((ts - tPrevEst) / 1000.0));
    tPrevEst = ts;

    if (ts - tLastReport >= 1000.0) {
      tLastReport = ts;
      printf("%.1f,%.1f,%.2f,%.2f,%.3f,%.1f\n", (ts) / 1000.0, pulse.bpm,
             pulse.confidence, pulse.phase, gsr.arousal, gsr.tonic);
    }
  }
  fclose(fh);
  return 0;
}
