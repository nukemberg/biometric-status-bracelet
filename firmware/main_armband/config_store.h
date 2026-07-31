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

}  // namespace ConfigStore
