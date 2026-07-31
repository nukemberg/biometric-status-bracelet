#!/usr/bin/env bash
# Assert the BLE wire layout. Run after any change to libraries/BraceletProtocol.
set -euo pipefail
cd "$(dirname "$0")/.."
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT
c++ -O2 -std=c++17 -Wall -Wextra -Ilibraries/BraceletProtocol/src -Ilibraries/BraceletDSP/src \
    -o "$OUT/ble_packet_test" tools/ble_packet_test.cpp
"$OUT/ble_packet_test"
