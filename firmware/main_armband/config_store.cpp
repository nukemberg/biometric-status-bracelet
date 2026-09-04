#include "config_store.h"

#include <Preferences.h>

#include <ble_protocol.h>   // CFG_PARAM_COUNT, BleConfigParam ids

namespace {

// Bump if BraceletConfig's field set changes shape. A stored blob from an older
// version is discarded rather than partially reinterpreted -- a hue anchor read
// into what used to be a threshold field would silently misbehave rather than fail.
//
// v2: the field set is unchanged, but the MAX30102 front end changed what
// `piTrustMin` MEANS. A value tuned against the analog sensor's perfusion index is
// still perfectly in range for the new one and would be loaded without complaint,
// which is precisely the silent-wrong-number case this store is built to avoid. The
// bump forces every device to fall back to compiled defaults once, so the threshold
// gets re-derived from the new sensor rather than inherited from the old one.
//
// v3: added arousalDisplayGamma/arousalDisplayFloor (-eba). A v2 blob is missing
// these keys, which load() already treats as "one field absent invalidates the
// whole load" -- so the bump is required for that fallback to trigger rather than
// booting with a NaN-checked-but-uninitialized display curve.
constexpr uint8_t CONFIG_SCHEMA_VERSION = 3;

// One key per field rather than a single packed blob. Costs a few more NVS entries
// but means a value out of plausible range can be rejected per-field instead of
// invalidating the whole config, and a schema change only needs adding, not
// re-encoding.
struct FieldSpec {
  const char *key;
  float BraceletConfig::*field;
  float lo, hi;   // plausibility bounds; anything outside this is corrupt, not just
                  // unusual, and is rejected rather than trusted.
};

const FieldSpec FIELDS[] = {
    {"hueBpmLo",   &BraceletConfig::hueBpmLo,   0.0f,    400.0f},
    {"hueBpmHi",   &BraceletConfig::hueBpmHi,   0.0f,    400.0f},
    {"hueAtLo",    &BraceletConfig::hueAtLo,   -400.0f,  400.0f},
    {"hueAtHi",    &BraceletConfig::hueAtHi,   -400.0f,  400.0f},
    {"piTrustMin", &BraceletConfig::piTrustMin, 0.0f,    100.0f},
    {"irWornMin",  &BraceletConfig::irWornMin,  0.0f,    262143.0f},
    {"irWornMax",  &BraceletConfig::irWornMax,  0.0f,    262143.0f},
    {"confGate",   &BraceletConfig::confGate,   0.0f,    1.0f},
    {"confRef",    &BraceletConfig::confRef,    0.0f,    1.0f},
    {"slewBpmPerS",&BraceletConfig::slewBpmPerS,0.0f,    400.0f},
    {"brightness", &BraceletConfig::brightness, 0.0f,    255.0f},
    {"tempOffsetC",&BraceletConfig::tempOffsetC,-10.0f,  10.0f},
    {"arDispGamma",&BraceletConfig::arousalDisplayGamma, 0.05f, 3.0f},
    {"arDispFloor",&BraceletConfig::arousalDisplayFloor, 0.0f,  1.0f},
};
constexpr size_t FIELD_COUNT = sizeof(FIELDS) / sizeof(FIELDS[0]);

constexpr const char *NAMESPACE = "bracelet";

}  // namespace

namespace ConfigStore {

BraceletConfig load() {
  BraceletConfig cfg = BraceletConfig::defaults();

  Preferences prefs;
  if (!prefs.begin(NAMESPACE, /*readOnly=*/true)) {
    // No namespace yet -- a fresh device, or one that has never saved. Not an
    // error; compiled defaults are exactly right here.
    return cfg;
  }

  uint8_t version = prefs.getUChar("schema", 0);
  if (version != CONFIG_SCHEMA_VERSION) {
    prefs.end();
    return cfg;   // absent, or from a shape this firmware no longer understands
  }

  for (size_t i = 0; i < FIELD_COUNT; i++) {
    const FieldSpec &f = FIELDS[i];
    // NaN default sentinel: distinguishes "key absent" from "key present and
    // legitimately zero", which getFloat's own default argument cannot do.
    float v = prefs.getFloat(f.key, NAN);
    if (isnan(v) || v < f.lo || v > f.hi) {
      // One implausible or missing field invalidates the whole load. A config
      // that is half compiled-defaults and half stored values is a state nobody
      // asked for and nobody could have tested.
      prefs.end();
      return BraceletConfig::defaults();
    }
    cfg.*(f.field) = v;
  }

  prefs.end();
  return cfg;
}

void save(const BraceletConfig &cfg) {
  Preferences prefs;
  if (!prefs.begin(NAMESPACE, /*readOnly=*/false)) return;

  // Written last, so a power loss mid-save leaves an old-or-absent version marker
  // rather than a version claiming a body that never fully landed -- load() then
  // falls back to defaults instead of reading a partially written config.
  prefs.putUChar("schema", 0);
  for (size_t i = 0; i < FIELD_COUNT; i++) {
    prefs.putFloat(FIELDS[i].key, cfg.*(FIELDS[i].field));
  }
  prefs.putUChar("schema", CONFIG_SCHEMA_VERSION);

  prefs.end();
}

void resetToDefaults() {
  Preferences prefs;
  if (!prefs.begin(NAMESPACE, /*readOnly=*/false)) return;
  prefs.clear();
  prefs.end();
}

// The FIELDS table is ordered to match the BleConfigParam enum (id = index + 1),
// so a paramId resolves to a field without a second switch. A static_assert would
// be the place to enforce that, but the two are in different headers; the
// ble_packet_test config-read fixture covers the mapping end to end instead.
bool applyParam(BraceletConfig &cfg, uint8_t paramId, float value) {
  if (paramId == 0 || paramId > CFG_PARAM_COUNT) return false;
  const FieldSpec &f = FIELDS[paramId - 1];
  if (isnan(value) || value < f.lo || value > f.hi) return false;
  cfg.*(f.field) = value;
  return true;
}

void packValues(const BraceletConfig &cfg, float *out) {
  for (size_t i = 0; i < FIELD_COUNT; i++) {
    out[i] = cfg.*(FIELDS[i].field);
  }
}

}  // namespace ConfigStore
