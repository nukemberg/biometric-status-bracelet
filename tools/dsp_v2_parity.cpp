// Host-side parity harness for libraries/BraceletDSP.
//
// Replays a raw_streamer CSV through the real PulseTracker/GsrTracker -- the exact
// code the firmware runs -- and prints the result so it can be diffed against the
// Python reference in tools/dsp_v2_sim.py. If the two disagree, offline validation
// says nothing about on-device behaviour.
//
// Because dsp.h depends only on <math.h> and <stdint.h>, this needs no Arduino
// stubbing at all. Keep it that way: any Arduino or BLE type that leaks into the
// library breaks this harness and costs us the validation path that has caught
// every bug in this pipeline so far.
//
// Build & run:  tools/dsp_v2_parity.sh [capture.csv]

#include <cstdint>
#include <cstdio>

#include <dsp.h>

int main(int argc, char **argv) {
  if (argc < 2) {
    fprintf(stderr, "usage: %s <raw_streamer.csv>\n", argv[0]);
    return 2;
  }
  FILE *fh = fopen(argv[1], "r");
  if (!fh) { perror(argv[1]); return 1; }

  PulseTracker pulse;
  GsrTracker gsr;
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
      printf("%.1f,%.1f,%.2f,%.2f,%.3f,%.1f\n", ts / 1000.0, pulse.bpm,
             pulse.confidence, pulse.phase, gsr.arousal, gsr.tonic);
    }
  }
  fclose(fh);
  return 0;
}
