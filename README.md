# Biometric Status Bracelet

A wearable forearm panel that reads heart rate, skin conductance and temperature, and
renders them on a 21-LED WS2812B matrix. Built for festivals and parties — the LEDs
should respond well, which matters more here than clinical accuracy.

**[docs/DESIGN.md](docs/DESIGN.md)** is the main document: hardware, pin maps, the signal
processing design, and what turned out to be true (and false) about these sensors.

## Layout

```
firmware/
  main_armband/   production firmware — DSP, LEDs, wear detection
  dsp_v2/         bench sketch, streams computed values as CSV, no LEDs
  raw_streamer/   500 Hz raw capture for spectral ground truth
  sensor_test/    hardware bring-up smoke test
tools/
  dsp_v2_sim.py     Python reference implementation of the pipeline
  dsp_v2_parity.sh  proves the shipped C++ matches that reference
samples/          captures with known ground truth, used as regression cases
```

## Working on the signal processing

Validate offline before flashing — debugging DSP through LED animations does not work.

```sh
# run the pipeline against a capture, with the legacy engines for comparison
uv run python tools/dsp_v2_sim.py samples/bio4.csv --compare

# confirm the firmware C++ and the Python reference still agree
tools/dsp_v2_parity.sh samples/bio2.log
```

`samples/bio4.csv` is the primary regression case: 226 s of good contact with a ground
truth of 64 BPM from a manual radial count taken during the capture, confirmed by a Welch
peak at 64.1 BPM.

## Build

```sh
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/main_armband
arduino-cli compile --fqbn esp32:esp32:esp32 --upload --port /dev/cu.usbserial-0001 firmware/main_armband
```

ESP32-S3 is the deployment target; uncomment `#define MCU_ESP32_S3` for its pin map. All
bench work and every capture here was taken on an ESP32 WROOM-32 DevKit.
