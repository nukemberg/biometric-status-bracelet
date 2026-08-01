# Biometric Status Bracelet — just recipes
# Usage: just <recipe> [args]
# Lists all recipes with: just

set dotenv-load

FQBN := "esp32:esp32:esp32"
PORT := "cu.usbserial-0001"
SKETCH := "firmware/main_armband"
LIBS := "--libraries ./libraries"

# ---- build --------------------------------------------------------------

# Compile the firmware (default recipe)
build:
    arduino-cli compile --fqbn {{FQBN}} {{LIBS}} {{SKETCH}}

# Alias
compile: build

# Compile and flash
upload: build
    arduino-cli compile --upload --port /dev/{{PORT}} --fqbn {{FQBN}} {{LIBS}} {{SKETCH}}

# Flash without recompiling
flash:
    arduino-cli upload --port /dev/{{PORT}} --fqbn {{FQBN}} --input-dir {{SKETCH}}/build/esp32.esp32.{{SKETCH}}

# ---- serial ----------------------------------------------------------------

# Stream serial output (hold the port open — suppresses DTR/RTS reboot)
monitor *FLAGS="":
    uv run tools/capture.py {{FLAGS}}

# Monitor for N seconds, save to file
monitor-save SECONDS="30" OUT="session.log":
    uv run tools/capture.py --seconds {{SECONDS}} --out {{OUT}}

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

# DSP C++ vs Python parity (needs a sample file)
test-parity SAMPLE="samples/bio2.log":
    tools/dsp_v2_parity.sh {{SAMPLE}}

# Run all automated tests
test: test-ble test-proto
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
