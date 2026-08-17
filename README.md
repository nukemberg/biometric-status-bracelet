# Biometric Status Bracelet

A wearable forearm panel that reads heart rate, skin conductance and temperature, and
renders them on a 21-LED WS2812B matrix. Built for festivals and parties — the LEDs
should respond well, which matters more here than clinical accuracy.

**[docs/DESIGN.md](docs/DESIGN.md)** is the main document: hardware, pin maps, the signal
processing design, and what turned out to be true (and false) about these sensors.

## Layout

```
libraries/
  BraceletDSP/       the pulse, GSR and wear-detection engines — shared by every
                     consumer, and deliberately free of Arduino and BLE types
  BraceletMAX30102/  MAX30102 PPG driver — config and FIFO drain, nothing else
  BraceletProtocol/  BLE wire format, shared by the firmware, CLI and web app
firmware/
  main_armband/   production firmware — LEDs, tasks, wear gating
  dsp_v2/         bench sketch, streams computed values as CSV, no LEDs
  raw_streamer/   raw capture: 500 Hz GSR + flagged MAX30102 FIFO samples
  sensor_test/    hardware bring-up smoke test
  max30102_probe/ I2C diagnostic — bus scan, config read-back, raw FIFO pointers
webapp/
  index.html        Web Bluetooth dashboard (Chrome on Android)
tools/
  dsp_v2_sim.py       Python reference implementation of the pipeline
  dsp_v2_parity.sh    proves the shipped C++ matches that reference
  make_synthetic_capture.py  deterministic fixture so parity runs without hardware
  ble_packet_test.sh  asserts the BLE wire layout against golden byte fixtures
  dsp_v2_sim.py --from-ble  re-runs the resonator bank against a blectl stream capture
  capture.py          reads serial output from the board
  jitter_notch.py     how much sampling jitter degrades the 50 Hz notch
  blectl.py           read the bracelet over BLE, no USB tether
  bracelet_protocol.py  Python side of the BLE wire format
samples/          captures used as regression cases
```

## Sensors

| | |
|---|---|
| Pulse | **MAX30102** — I²C (0x57), 18-bit IR, own FIFO at 25 Hz |
| Skin conductance | Grove GSR v1.2 — analog, 500 Hz → 25 Hz boxcar |
| Temperature | BME280 — I²C (0x76/0x77), shares the bus with the MAX30102 |

The MAX30102 replaced an analog PPG sensor whose SNR was below 1 on this hardware.
**That swap invalidated every PPG measurement in DESIGN.md §2 and every capture in
`samples/`**, and the recalibration is not done — see §4.2 for the procedure and §3.6 for
what is still owed.

`libraries/BraceletDSP/src/dsp.h` depends only on `<math.h>` and `<stdint.h>`. That is
what lets the parity harness compile the *real* trackers on a host machine — keep Arduino
and BLE types out of it.

## Working on the signal processing

Validate offline before flashing — debugging DSP through LED animations does not work.

```sh
# run the pipeline against a capture
uv run python tools/dsp_v2_sim.py samples/synthetic.csv --compare

# confirm the firmware C++ and the Python reference still agree
tools/dsp_v2_parity.sh          # defaults to samples/synthetic.csv

# assert the BLE wire layout (run after touching BraceletProtocol)
tools/ble_packet_test.sh

# all of the above
just test
```

`samples/synthetic.csv` is a **generated** fixture, not a recording. It exists so parity
is checkable with no hardware, and it does that job well — C++ and Python agree to 0.0000
on BPM, confidence and phase. It says nothing about whether the tracker reads the right
rate off a real wrist.

**There is currently no MAX30102 wrist capture.** The `bio*` files in `samples/` are
analog front-end recordings; their pulse columns are readings of a sensor the bracelet no
longer has, and `dsp_v2_sim.py` rejects their 3-column format rather than reinterpreting
it. Capturing a replacement is step 4 of DESIGN.md §4.2.

## Build

```sh
arduino-cli compile --fqbn esp32:esp32:esp32 --libraries ./libraries firmware/main_armband
arduino-cli compile --fqbn esp32:esp32:esp32 --libraries ./libraries \
  --upload --port /dev/cu.usbserial-0001 firmware/main_armband
```

## Capturing from the board

Opening the serial port asserts DTR/RTS and reboots the ESP32, so a short capture only
ever shows a cold-booting device — the GSR tonic EMA needs ~45 s to settle and the
resonator bank ~10 s. Suppressing that in software does not work on the CP210x adapter
(tested). Hold the port open instead:

```sh
tools/capture.py --seconds 0 --quiet --out session.log &   # resets once, then streams
tail -f session.log
```

Stop the logger before flashing — it holds the port.

Or skip USB entirely and read over BLE, which neither reboots the board nor requires
the wearer to sit next to the laptop:

```sh
tools/blectl.py selftest              # decode the C++ golden fixtures, no device needed
tools/blectl.py scan
tools/blectl.py monitor --csv vitals.csv
```

## Build

ESP32-S3 is the deployment target; uncomment `#define MCU_ESP32_S3` for its pin map. All
bench work and every capture here was taken on an ESP32 WROOM-32 DevKit.
