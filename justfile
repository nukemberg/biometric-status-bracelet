# Biometric Status Bracelet — just recipes
# Usage: just <recipe> [args]
# Lists all recipes with: just

set dotenv-load

# ESP32-S3 DevKit, matching the MCU_ESP32_S3 pin map in main_armband.ino. CDCOnBoot=cdc
# routes Serial over the S3's native USB, which is the only serial the board exposes --
# without it every Serial.print goes to the unpopulated UART0 header and boot looks dead.
FQBN := "esp32:esp32:esp32s3:CDCOnBoot=cdc"
# Native USB CDC enumerates as usbmodem, not the CP210x usbserial of the old WROOM board.
PORT := "cu.usbmodem101"
SKETCH := "firmware/main_armband"
LIBS := "--libraries ./libraries"
# Pinned so `flash` can find what `build` produced. Without it arduino-cli writes to a
# hashed path under the system cache and the two recipes disagree about where the
# binary is.
BUILD := "firmware/main_armband/build"

# ---- build --------------------------------------------------------------

# Compile the firmware (default recipe)
build:
    arduino-cli compile --fqbn {{FQBN}} {{LIBS}} --build-path {{BUILD}} {{SKETCH}}

# Alias
compile: build

# Compile and flash
upload: build
    arduino-cli upload --port /dev/{{PORT}} --fqbn {{FQBN}} --input-dir {{BUILD}} {{SKETCH}}

# Flash without recompiling
flash:
    arduino-cli upload --port /dev/{{PORT}} --fqbn {{FQBN}} --input-dir {{BUILD}} {{SKETCH}}

# ---- serial ----------------------------------------------------------------

# Stream serial output (hold the port open — suppresses DTR/RTS reboot)
monitor *FLAGS="":
    uv run tools/capture.py {{FLAGS}}

# Monitor for N seconds, save to file
monitor-save SECONDS="30" OUT="session.log":
    uv run tools/capture.py --seconds {{SECONDS}} --out {{OUT}}

# ---- raw capture -----------------------------------------------------------

# Build + flash firmware/raw_streamer (500 Hz GSR + MAX30102 IR csv, no DSP)
raw-flash:
    arduino-cli compile --fqbn {{FQBN}} {{LIBS}} --build-path firmware/raw_streamer/build firmware/raw_streamer
    arduino-cli upload --port /dev/{{PORT}} --fqbn {{FQBN}} --input-dir firmware/raw_streamer/build firmware/raw_streamer

# Paced-breathing capture for breath-detection ground truth (needs raw-flash first)
breath-capture OUT="samples/breath_paced.csv" *FLAGS="":
    uv run tools/paced_capture.py --port /dev/{{PORT}} --out {{OUT}} {{FLAGS}}

# ---- BLE -------------------------------------------------------------------

# Scan for the bracelet
scan:
    uv run tools/blectl.py scan

# Device info and protocol version
info:
    uv run tools/blectl.py info

# Stream decoded vitals
monitor-ble *FLAGS="":
    uv run tools/blectl.py monitor {{FLAGS}}

# ---- config ----------------------------------------------------------------

# Read all tunables
config-get:
    uv run tools/blectl.py config get

# Set one tunable (just config-set <param> <value>)
config-set PARAM VALUE:
    uv run tools/blectl.py config set {{PARAM}} {{VALUE}}

# Restore compiled defaults
config-reset:
    uv run tools/blectl.py config reset

# ---- validation ------------------------------------------------------------

# BLE wire-format byte-layout tests
test-ble:
    tools/ble_packet_test.sh

# Python protocol selftest (decodes the C++ golden fixtures)
test-proto:
    uv run tools/blectl.py selftest

# DSP C++ vs Python parity (defaults to the synthetic fixture)
test-parity SAMPLE="samples/synthetic.csv":
    tools/dsp_v2_parity.sh {{SAMPLE}}

# Regenerate the deterministic synthetic capture used by test-parity
synth OUT="samples/synthetic.csv" *FLAGS="":
    uv run tools/make_synthetic_capture.py {{FLAGS}} > {{OUT}}

# Run all automated tests
test: test-ble test-proto test-parity
    echo "all automated tests passed"


# Compile, flash, and start monitoring in the background
deploy: upload
    sleep 2
    just monitor --quiet --out deploy.log &
    tail -f deploy.log

# Flash, watch boot, stop when you see advertising
poke: upload
    just monitor --quiet --out poke.log &
    sleep 15
    pkill -f "capture.py.*poke.log"
    echo "--- last 30 lines ---"
    tail -30 poke.log

# Check webapp JS syntax (no browser needed)
lint-web:
    python3 -c 'import re; html=open("webapp/index.html").read(); m=re.search(r"<script>(.*?)</script>",html,re.S); open("/tmp/_web.mjs","w").write(m.group(1))'
    node --check /tmp/_web.mjs && echo "webapp JS: OK"
    rm -f /tmp/_web.mjs
