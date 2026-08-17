// Host-side parity harness for libraries/BraceletDSP.
//
// Replays a raw_streamer CSV through the real PulseTracker/GsrTracker -- the exact
// code the firmware runs -- and prints the result so it can be diffed against the
// Python reference in tools/dsp_v2_sim.py. If the two disagree, offline validation
// says nothing about on-device behaviour.
//
// Input is Timestamp_ms,RawIr,RawGSR,IrNew. GSR is decimated 20:1 from 500 Hz; PPG
// comes off the MAX30102 FIFO and advances only on IrNew=1 rows. Keeping that split
// identical on both sides is the whole reason the flag is in the capture format
// rather than the IR column being zero-order held.
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
  uint32_t accG = 0;
  int accN = 0;
  int rows = 0;
  double t0 = -1, tPrevEst = 0, tLastReport = 0;

  while (fgets(line, sizeof(line), fh)) {
    double ts, ir, rg;
    int irNew;
    // Timestamp_ms,RawIr,RawGSR,IrNew. A three-column analog capture fails this
    // parse on every row and produces no output, which the diff in
    // dsp_v2_parity.sh reports as "no rows to compare" -- the sim rejects the same
    // file with an explicit message.
    if (sscanf(line, "%lf,%lf,%lf,%d", &ts, &ir, &rg, &irNew) != 4) continue;
    if (t0 < 0) { t0 = ts; tPrevEst = ts; tLastReport = ts; }
    rows++;

    // PPG advances only on rows carrying a real FIFO sample; RawIr is 0 elsewhere and
    // must not be fed in. Same ordering as tools/dsp_v2_sim.py::run().
    if (irNew) pulse.update((float)ir);

    accG += (uint32_t)rg;
    if (++accN < DECIM) continue;

    gsr.update((float)accG / (float)DECIM);
    accG = 0;
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
  if (rows == 0) {
    fprintf(stderr,
            "%s: no 4-column rows. Expected Timestamp_ms,RawIr,RawGSR,IrNew from "
            "firmware/raw_streamer. Three-column analog captures are from the "
            "pre-MAX30102 front end and are not valid regression cases.\n",
            argv[1]);
    return 1;
  }
  return 0;
}
