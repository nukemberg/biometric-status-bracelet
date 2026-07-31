/*
 * NVS persistence for BraceletConfig.
 *
 * Deliberately thin: this unit knows how to get a BraceletConfig into and out of
 * flash and nothing else. It never reaches into the trackers -- main_armband.ino
 * owns applying a loaded config to PulseTracker/WearDetect and to the LED hue
 * anchors, the same way ble_service.cpp never reaches into the trackers directly.
 */

#pragma once

#include <dsp.h>

namespace ConfigStore {

// Loads from NVS if present and plausible (see below), otherwise returns compiled
// defaults. A device that has never been configured, or whose NVS namespace is
// missing or corrupt, must still boot with known-good behaviour -- never with
// zeroed or garbage tunables.
BraceletConfig load();

// Persists the whole struct.
void save(const BraceletConfig &cfg);

// Erases the stored config. The next load() (or a call to load() right after this)
// falls back to compiled defaults.
void resetToDefaults();

// --- BLE config characteristic helpers (-pmw) ------------------------------
// The parameter ids are the BleConfigParam values from ble_protocol.h. They are
// passed as plain uint8_t here so this header does not need to include the BLE
// protocol header -- config_store is firmware-only, but keeping the dependency
// arrow pointing toward dsp.h (not toward the wire format) matches the rest of
// the project's layering.
//
// The FIELDS table is kept in the same order as the BleConfigParam enum, so a
// paramId maps directly to a table index. These two functions are the single place
// that mapping lives: NVS load/save, the BLE read packer and the BLE write applier
// all go through it, so a new tunable cannot end up wired to one path and not the
// others.

// Validates value against the same plausibility bounds used on NVS load, then sets
// the field. Returns false for an unknown id or an out-of-range value -- a write
// that would put the device into an untested state is rejected, not clamped.
bool applyParam(BraceletConfig &cfg, uint8_t paramId, float value);

// Writes all CFG_PARAM_COUNT values into out, in paramId order, for blePackConfigRead.
void packValues(const BraceletConfig &cfg, float *out);

}  // namespace ConfigStore
