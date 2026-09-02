"""Python side of the bracelet BLE wire format.

Mirrors libraries/BraceletProtocol/src/ble_protocol.h. That header is the
specification and its golden-byte fixtures in tools/ble_packet_test.cpp are the
authority; this module must be kept in step with it. `blectl.py --selftest` decodes
the same fixtures, so a drift between the two is caught rather than silently
producing plausible wrong numbers.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

PROTOCOL_VERSION = 2

SVC_BRACELET = "a1b20001-5e8f-4d7a-9c31-0f2e6d4b8a70"
CHR_VITALS = "a1b20002-5e8f-4d7a-9c31-0f2e6d4b8a70"
CHR_SIGNALS = "a1b20003-5e8f-4d7a-9c31-0f2e6d4b8a70"
CHR_SPECTRUM = "a1b20004-5e8f-4d7a-9c31-0f2e6d4b8a70"
CHR_CONTROL = "a1b20005-5e8f-4d7a-9c31-0f2e6d4b8a70"
CHR_CONFIG = "a1b20006-5e8f-4d7a-9c31-0f2e6d4b8a70"
CHR_INFO = "a1b20007-5e8f-4d7a-9c31-0f2e6d4b8a70"
CHR_LOG = "a1b20008-5e8f-4d7a-9c31-0f2e6d4b8a70"

VITALS_LEN = 20
SIGNALS_BATCH = 5
SIGNALS_LEN = 6 + SIGNALS_BATCH * 12
SPECTRUM_BINS = 48
SPECTRUM_LEN = 8 + SPECTRUM_BINS

VF_WORN = 0x01
VF_PULSE_TRUSTED = 0x02
VF_STROBE = 0x04
VF_MODE_MASK = 0x18
VF_MODE_SHIFT = 3

# Control commands
CMD_SET_MODE = 0x01
CMD_SET_BRIGHTNESS = 0x02
CMD_RECALIBRATE_GSR = 0x03
CMD_SET_STREAMS = 0x04
CMD_RESET_BANK = 0x05
CMD_RESET_CONFIG = 0x06
CMD_DUMP_LOG = 0x07
CMD_ENTER_BOOTLOADER = 0x08
# CMD_ENTER_BOOTLOADER carries a mandatory magic argument: it is the one control
# command that takes the device off the air, so the firmware rejects any write that
# does not match exactly. See ble_protocol.h.
CMD_BOOTLOADER_MAGIC = 0xB0

STREAM_SIGNALS = 0x01
STREAM_SPECTRUM = 0x02

CONFIG_PARAMS = {
    "hue_bpm_lo": 0x01,
    "hue_bpm_hi": 0x02,
    "hue_at_lo": 0x03,
    "hue_at_hi": 0x04,
    "pi_trust_min": 0x05,
    "ir_worn_min": 0x06,
    "ir_worn_max": 0x07,
    "conf_gate": 0x08,
    "conf_ref": 0x09,
    "slew_bpm_s": 0x0A,
    "brightness": 0x0B,
}
# Inverse of CONFIG_PARAMS, built once. The read returns records in ascending
# paramId order, so this is also the order decode_config_read yields.
PARAM_BY_ID = {pid: name for name, pid in CONFIG_PARAMS.items()}
CONFIG_PARAM_COUNT = len(CONFIG_PARAMS)
CONFIG_WRITE_LEN = 5
CONFIG_READ_LEN = CONFIG_PARAM_COUNT * CONFIG_WRITE_LEN


class ProtocolError(ValueError):
    """Raised rather than returning a plausible-looking wrong value."""


def _check_version(data: bytes, what: str) -> None:
    if not data:
        raise ProtocolError(f"{what}: empty packet")
    if data[0] != PROTOCOL_VERSION:
        raise ProtocolError(
            f"{what}: device speaks protocol v{data[0]}, this tool speaks "
            f"v{PROTOCOL_VERSION}. Reflash the firmware or update the tool."
        )


@dataclass
class Vitals:
    bpm: float
    confidence: float
    arousal: float
    perfusion: float
    gsr_raw: int
    pulse_raw: int
    temp_c: float
    gsr_tonic: float
    brightness: int
    mode: int
    worn: bool
    pulse_trusted: bool
    strobe: bool

    def format(self) -> str:
        flags = []
        flags.append("worn" if self.worn else "NOT-WORN")
        flags.append("trusted" if self.pulse_trusted else "SEARCHING")
        if self.strobe:
            flags.append("strobe")
        return (
            f"bpm {self.bpm:6.1f}  conf {self.confidence:.2f}  "
            f"arousal {self.arousal:.2f}  perf {self.perfusion:5.2f}%  "
            f"gsr {self.gsr_raw:4d}  ir {self.pulse_raw:6d}  "
            f"{self.temp_c:5.1f}C  mode {self.mode}  [{' '.join(flags)}]"
        )


def decode_vitals(data: bytes) -> Vitals:
    _check_version(data, "vitals")
    if len(data) < VITALS_LEN:
        raise ProtocolError(f"vitals: {len(data)} bytes, expected {VITALS_LEN}")
    flags = data[1]
    # pulse_raw is u32 since protocol v2 -- MAX30102 IR is an 18-bit count.
    bpm, conf, arousal, perf, gsr, pulse, temp, tonic, bright = struct.unpack_from(
        "<HBBHHIhHB", data, 2
    )
    return Vitals(
        bpm=bpm / 10.0,
        confidence=conf / 255.0,
        arousal=arousal / 255.0,
        perfusion=perf / 100.0,
        gsr_raw=gsr,
        pulse_raw=pulse,
        temp_c=temp / 100.0,
        gsr_tonic=float(tonic),
        brightness=bright,
        mode=(flags & VF_MODE_MASK) >> VF_MODE_SHIFT,
        worn=bool(flags & VF_WORN),
        pulse_trusted=bool(flags & VF_PULSE_TRUSTED),
        strobe=bool(flags & VF_STROBE),
    )


@dataclass
class SignalSample:
    pulse_filtered: float
    gsr_phasic: float
    pulse_raw: int   # MAX30102 IR, 18-bit counts
    gsr_raw: int


def decode_signals(data: bytes) -> tuple[int, list[SignalSample]]:
    """Returns (timestamp_ms of the first sample, samples)."""
    _check_version(data, "signals")
    if len(data) < 6:
        raise ProtocolError(f"signals: {len(data)} bytes, expected at least 6")
    count = data[1]
    if count > SIGNALS_BATCH:
        raise ProtocolError(f"signals: claims {count} samples, max {SIGNALS_BATCH}")
    if len(data) < 6 + count * 12:
        raise ProtocolError(f"signals: {len(data)} bytes for {count} samples")
    (ts,) = struct.unpack_from("<I", data, 2)
    out = []
    for i in range(count):
        # v2 sample: i32 filtered x10, u32 raw IR, i16 gsr phasic x10, u16 gsr raw.
        pf, pr, gp, gr = struct.unpack_from("<iIhH", data, 6 + i * 12)
        out.append(SignalSample(pf / 10.0, gp / 10.0, pr, gr))
    return ts, out


@dataclass
class Spectrum:
    peak_bin: int
    bpm_lo: float
    bpm_hi: float
    bins: list[int]

    def bpm_of_bin(self, i: int) -> float:
        if len(self.bins) < 2:
            return self.bpm_lo
        return self.bpm_lo + (self.bpm_hi - self.bpm_lo) * i / (len(self.bins) - 1)

    @property
    def peak_bpm(self) -> float:
        return self.bpm_of_bin(self.peak_bin)


def decode_spectrum(data: bytes) -> Spectrum:
    _check_version(data, "spectrum")
    if len(data) < 8:
        raise ProtocolError(f"spectrum: {len(data)} bytes, expected at least 8")
    nbins = data[1]
    if nbins > SPECTRUM_BINS:
        raise ProtocolError(f"spectrum: claims {nbins} bins, max {SPECTRUM_BINS}")
    if len(data) < 8 + nbins:
        raise ProtocolError(f"spectrum: {len(data)} bytes for {nbins} bins")
    peak = data[2]
    lo, hi = struct.unpack_from("<HH", data, 4)
    return Spectrum(peak, lo / 10.0, hi / 10.0, list(data[8 : 8 + nbins]))


def encode_control(cmd: int, arg: int | None = None) -> bytes:
    return bytes([cmd]) if arg is None else bytes([cmd, arg & 0xFF])


def encode_config_write(param: str, value: float) -> bytes:
    if param not in CONFIG_PARAMS:
        raise ProtocolError(
            f"unknown parameter {param!r}; known: {', '.join(sorted(CONFIG_PARAMS))}"
        )
    return bytes([CONFIG_PARAMS[param]]) + struct.pack("<f", value)


def decode_config_write(data: bytes) -> tuple[str, float]:
    """Mirror of bleUnpackConfigWrite: one [paramId][f32] record."""
    if len(data) < CONFIG_WRITE_LEN:
        raise ProtocolError(f"config write: {len(data)} bytes, expected {CONFIG_WRITE_LEN}")
    param_id = data[0]
    if param_id == 0 or param_id > CONFIG_PARAM_COUNT:
        raise ProtocolError(f"config write: unknown parameter id 0x{param_id:02X}")
    value = struct.unpack_from("<f", data, 1)[0]
    return PARAM_BY_ID[param_id], value


def decode_config_read(data: bytes) -> dict[str, float]:
    """Decode the CFG_PARAM_COUNT records a read returns, in paramId order.

    Mirrors bleUnpackConfigRead: each record must carry the paramId its slot
    implies, so a reorder is rejected rather than silently mislabelled.
    """
    if len(data) < CONFIG_READ_LEN:
        raise ProtocolError(
            f"config read: {len(data)} bytes, expected {CONFIG_READ_LEN}"
        )
    out: dict[str, float] = {}
    for i in range(CONFIG_PARAM_COUNT):
        off = i * CONFIG_WRITE_LEN
        param_id = data[off]
        if param_id != i + 1:
            raise ProtocolError(
                f"config read: record {i} carries id 0x{param_id:02X}, "
                f"expected 0x{i + 1:02X}"
            )
        out[PARAM_BY_ID[param_id]] = struct.unpack_from("<f", data, off + 1)[0]
    return out


# ---------------------------------------------------------------------------
# Fixtures shared with tools/ble_packet_test.cpp. If the C++ golden bytes change,
# these must change with them -- that is the point.
# ---------------------------------------------------------------------------
FIXTURE_VITALS = bytes(
    [0x02, 0x13, 0x83, 0x02, 0x40, 0x80, 0xA0, 0x00, 0x05, 0x05,
     0xF5, 0x48, 0x01, 0x00, 0x41, 0x0A, 0x9B, 0x05, 0x3C, 0x00]
)
# Sample 0 of the signals fixture in ble_packet_test.cpp::testSignals, byte for byte,
# wrapped in a count=1 header so it decodes standalone. The C++ fixture packs five;
# they all have the same 12-byte shape, and one is enough to pin the field offsets,
# which is the thing that actually drifts.
FIXTURE_SIGNALS = bytes(
    [0x02, 0x01, 0x40, 0xE2, 0x01, 0x00,
     0xC7, 0xCF, 0xFF, 0xFF,   # pulse_filtered x10 = -12345
     0xE8, 0x48, 0x01, 0x00,   # pulse_raw 84200
     0x00, 0x00,               # gsr_phasic x10 = 0
     0x14, 0x05]               # gsr_raw 1300
)
FIXTURE_CONFIG_WRITE = bytes([0x05, 0xCD, 0xCC, 0xCC, 0x3E])
# Matches blePackConfigRead in tools/ble_packet_test.cpp::testConfigRead. Values
# chosen for distinctive bit patterns so a swapped field is obviously wrong.
FIXTURE_CONFIG_READ = (
    bytes([0x01]) + struct.pack("<f", 1.0)    # hue_bpm_lo
    + bytes([0x02]) + struct.pack("<f", 2.0)    # hue_bpm_hi
    + bytes([0x03]) + struct.pack("<f", 3.0)    # hue_at_lo
    + bytes([0x04]) + struct.pack("<f", 4.0)    # hue_at_hi
    + bytes([0x05]) + struct.pack("<f", 0.25)   # pi_trust_min
    + bytes([0x06]) + struct.pack("<f", 100.0)  # ir_worn_min
    + bytes([0x07]) + struct.pack("<f", 4000.0) # ir_worn_max
    + bytes([0x08]) + struct.pack("<f", 0.5)    # conf_gate
    + bytes([0x09]) + struct.pack("<f", 0.9)    # conf_ref
    + bytes([0x0A]) + struct.pack("<f", 8.0)    # slew_bpm_s
    + bytes([0x0B]) + struct.pack("<f", 120.0)  # brightness
)


def selftest() -> int:
    """Decode the C++ golden fixtures. Returns the number of failures."""
    failures = 0

    def check(name, got, want, tol=0.0):
        nonlocal failures
        ok = abs(got - want) <= tol if isinstance(want, (int, float)) else got == want
        if not ok:
            print(f"  FAIL {name}: got {got!r} want {want!r}")
            failures += 1

    print(f"protocol v{PROTOCOL_VERSION} selftest against the C++ golden fixtures")

    v = decode_vitals(FIXTURE_VITALS)
    check("bpm", v.bpm, 64.3, 0.05)
    check("confidence", v.confidence, 0.25, 0.01)
    check("arousal", v.arousal, 0.5, 0.01)
    check("perfusion", v.perfusion, 1.60, 0.01)
    check("gsr_raw", v.gsr_raw, 1285)
    check("pulse_raw", v.pulse_raw, 84213)
    check("temp_c", v.temp_c, 26.25, 0.01)
    check("gsr_tonic", v.gsr_tonic, 1435.0, 1.0)
    check("brightness", v.brightness, 60)
    check("mode", v.mode, 2)
    check("worn", v.worn, True)
    check("pulse_trusted", v.pulse_trusted, True)
    check("strobe", v.strobe, False)

    # Signals: the widened pulse fields are the whole point of v2, so the fixture
    # carries values that would not have survived the v1 i16/u16 widths.
    ts, batch = decode_signals(FIXTURE_SIGNALS)
    check("signals timestamp", ts, 123456)
    check("signals sample count", len(batch), 1)
    check("signals pulse_filtered", batch[0].pulse_filtered, -1234.5, 0.05)
    check("signals pulse_raw", batch[0].pulse_raw, 84200)
    check("signals gsr_phasic", batch[0].gsr_phasic, 0.0, 0.05)
    check("signals gsr_raw", batch[0].gsr_raw, 1300)

    check("config write encoding", encode_config_write("pi_trust_min", 0.40),
          FIXTURE_CONFIG_WRITE)
    check("config write decode", decode_config_write(FIXTURE_CONFIG_WRITE)[0],
          "pi_trust_min")
    check("config write decode value", decode_config_write(FIXTURE_CONFIG_WRITE)[1],
          0.40, 1e-6)

    cfg = decode_config_read(FIXTURE_CONFIG_READ)
    check("config read hue_bpm_lo", cfg["hue_bpm_lo"], 1.0)
    check("config read pi_trust_min", cfg["pi_trust_min"], 0.25)
    check("config read ir_worn_max", cfg["ir_worn_max"], 4000.0)
    check("config read slew_bpm_s", cfg["slew_bpm_s"], 8.0)
    check("config read record count", len(cfg), CONFIG_PARAM_COUNT)

    # A reordered record must raise rather than mislabel a field.
    bad = bytearray(FIXTURE_CONFIG_READ)
    bad[0] = 0x02
    try:
        decode_config_read(bytes(bad))
        print("  FAIL config read reorder: decoded a record whose id mismatches its slot")
        failures += 1
    except ProtocolError:
        pass

    # A packet from another protocol version must raise, not decode.
    bad = bytearray(FIXTURE_VITALS)
    bad[0] = PROTOCOL_VERSION + 1
    try:
        decode_vitals(bytes(bad))
        print("  FAIL version mismatch: decoded a packet with the wrong version")
        failures += 1
    except ProtocolError:
        pass

    try:
        decode_vitals(FIXTURE_VITALS[:-1])
        print("  FAIL short packet: decoded a truncated packet")
        failures += 1
    except ProtocolError:
        pass

    print("all checks passed" if not failures else f"FAILED: {failures} check(s)")
    return failures
