"""
rover_protocol.py -- Jetson-side codec for the rover CAN messages.

This is the ONLY part of the protocol that exists in two languages. It has to:
the Teensy runs C++ and the Jetson runs Python, and neither can import the
other's source.

So it is pinned rather than trusted. tests/host/test_golden_vectors.py runs the
native C++ encoder and asserts, byte for byte, that this file produces
identical output for the same inputs, and that both sides agree on every
validation predicate. If anyone edits one side's field order, width,
endianness or CRC, CI fails immediately instead of the two sides silently
disagreeing on a rover at competition.

The safety logic is NOT duplicated here. It lives once, in C++, in
firmware/src/rover_controller.*, and the simulator the host tests run against
is that same compiled code driven over a pipe -- see tools/rover_sim.cpp.

Mirrors firmware/src/rover_protocol.h. Read that file for the design
rationale; PROTOCOL.md section 3 has the full field documentation.
"""

import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# CAN ids, modes, frame sizes, valid command ranges and timing live in
# rover_config.py. Re-exported here so existing callers can keep writing
# rp.CAN_ID_CONTROL.
from rover_config import (  # noqa: E402,F401
    CAN_ID_CONTROL, CAN_ID_TELEM_DRIVE_L, CAN_ID_TELEM_DRIVE_R,
    CAN_ID_TELEM_POWER, CAN_ID_TELEM_STATE, CAN_ID_TELEM_CMD_ECHO,
    MODE_SAFE, MODE_TELEOP, MODE_AUTONOMOUS, MODE_FAULT, MODE_MAX,
    CMD_MIN, CMD_MAX, PROTOCOL_VERSION, PWM_NEUTRAL_US, PWM_SPAN_US,
)

TELEM_IDS = (CAN_ID_TELEM_DRIVE_L, CAN_ID_TELEM_DRIVE_R,
             CAN_ID_TELEM_POWER, CAN_ID_TELEM_STATE, CAN_ID_TELEM_CMD_ECHO)

# --- operating modes (values in rover_config.py) ---
MODE_NAMES = {MODE_SAFE: "SAFE", MODE_TELEOP: "TELEOP",
              MODE_AUTONOMOUS: "AUTONOMOUS", MODE_FAULT: "FAULT"}

COMMANDABLE_MODES  = (MODE_SAFE, MODE_TELEOP, MODE_AUTONOMOUS)
MOTION_MODES = (MODE_TELEOP, MODE_AUTONOMOUS)

# --- CONTROL flags byte (wire byte 5) ---
CTRL_FLAG_STOP           = 0x01
CTRL_FLAG_AUTONOMY_ABORT = 0x02
CTRL_FLAG_RETURN_REQUEST = 0x04
CTRL_INDICATOR_MASK      = 0x38   # bits 3-5
CTRL_INDICATOR_SHIFT     = 3
CTRL_FLAG_C2_LOST        = 0x40   # bit 6: the Jetson reports the C2 link down
# bit 7, v0.6: the autonomy stack produced drive/steer in this frame, not the
# operator. Reserved before v0.6, so the flags byte now has no spare bits --
# the next field goes in the mode byte, whose top five bits are unused.
CTRL_FLAG_FROM_AUTONOMY  = 0x80

# --- command source / authority ---
# Which producer owns which mode. The controller refuses MOTION (and an
# arrival claim) from a source that does not own the current mode, and
# honours the safety flags from either. See sourceOwnsMode() in
# firmware/src/rover_protocol.cpp -- this is the Python mirror of it.
SOURCE_TELEOP   = False
SOURCE_AUTONOMY = True

# --- status indicator ---
INDICATOR_OFF        = 0
INDICATOR_TELEOP     = 1
INDICATOR_AUTONOMOUS = 2
INDICATOR_ARRIVED    = 3
INDICATOR_FAULT      = 4
INDICATOR_MAX        = 7
INDICATOR_NAMES = {INDICATOR_OFF: "OFF", INDICATOR_TELEOP: "TELEOP",
                   INDICATOR_AUTONOMOUS: "AUTONOMOUS",
                   INDICATOR_ARRIVED: "ARRIVED", INDICATOR_FAULT: "FAULT"}

# --- fault flags, uint16 since v0.4 ---
FAULT_COMM_TIMEOUT   = 0x0001
FAULT_OVER_CURRENT   = 0x0002
FAULT_ESTOP_ACTIVE   = 0x0004
FAULT_ENCODER_FAULT  = 0x0008
FAULT_UNDERVOLTAGE   = 0x0010
FAULT_FIRMWARE_FAULT = 0x0020
FAULT_PROTOCOL_ERROR = 0x0040
FAULT_C2_LINK_LOST   = 0x0080
FAULT_SEQ_GAP        = 0x0100
FAULT_CRC_ERROR      = 0x0200
FAULT_CMD_SOURCE     = 0x0400

FAULT_NAMES = [
    (FAULT_COMM_TIMEOUT, "COMM_TIMEOUT"),
    (FAULT_OVER_CURRENT, "OVER_CURRENT"),
    (FAULT_ESTOP_ACTIVE, "ESTOP_ACTIVE"),
    (FAULT_ENCODER_FAULT, "ENCODER_FAULT"),
    (FAULT_UNDERVOLTAGE, "UNDERVOLTAGE"),
    (FAULT_FIRMWARE_FAULT, "FIRMWARE_FAULT"),
    (FAULT_PROTOCOL_ERROR, "PROTOCOL_ERROR"),
    (FAULT_C2_LINK_LOST, "C2_LINK_LOST"),
    (FAULT_SEQ_GAP, "SEQ_GAP"),
    (FAULT_CRC_ERROR, "CRC_ERROR"),
    (FAULT_CMD_SOURCE, "CMD_SOURCE"),
]

# --- link health. Jetson and C2 are SEPARATE and never collapsed. ---
LINK_OK           = 0
LINK_DEGRADED     = 1
LINK_LOST         = 2
LINK_NOT_REPORTED = 3
LINK_NAMES = {LINK_OK: "OK", LINK_DEGRADED: "DEGRADED",
              LINK_LOST: "LOST", LINK_NOT_REPORTED: "NOT_REPORTED"}

# --- low-level controller health ---
CTRL_HEALTH_OK           = 0
CTRL_HEALTH_DEGRADED     = 1
CTRL_HEALTH_FAULT        = 2
CTRL_HEALTH_NOT_REPORTED = 3
CTRL_HEALTH_NAMES = {CTRL_HEALTH_OK: "OK", CTRL_HEALTH_DEGRADED: "DEGRADED",
                     CTRL_HEALTH_FAULT: "FAULT",
                     CTRL_HEALTH_NOT_REPORTED: "NOT_REPORTED"}

# --- what the controller is doing with the last command it accepted ---
# TelemetryCmdEcho.status bits 0-2. First match wins, in this order:
# NO_COMMAND, STOPPED, WRONG_SOURCE, APPLIED.
CMD_STATUS_APPLIED      = 0
CMD_STATUS_STOPPED      = 1
CMD_STATUS_WRONG_SOURCE = 2
CMD_STATUS_NO_COMMAND   = 3
CMD_STATUS_MAX          = 7
CMD_STATUS_NAMES = {CMD_STATUS_APPLIED: "APPLIED",
                    CMD_STATUS_STOPPED: "STOPPED",
                    CMD_STATUS_WRONG_SOURCE: "WRONG_SOURCE",
                    CMD_STATUS_NO_COMMAND: "NO_COMMAND"}

# --- TelemetryCmdEcho.status packing ---
CMD_ECHO_STATUS_MASK   = 0x07
CMD_ECHO_DECODE_MASK   = 0x38   # bits 3-5: why the LAST frame was rejected
CMD_ECHO_DECODE_SHIFT  = 3
CMD_ECHO_RETURN_ACTIVE = 0x40   # bit 6
CMD_ECHO_RESERVED_MASK = 0x80   # bit 7, must be zero

# --- command age sentinels ---
CMD_AGE_UNKNOWN = 0xFFFF
CMD_AGE_MAX     = 0xFFFE

# --- frame geometry: 6 payload bytes + seq + CRC = 8 ---
# Stated rather than derived, because the seq and CRC bytes are not in any
# struct format. test_config_sync.py pins these to the C++ values.
FRAME_DLC        = 8
FRAME_PAYLOAD    = 6
FRAME_SEQ_OFFSET = 6
FRAME_CRC_OFFSET = 7

# Payload layouts. '<' = little-endian, no padding, matching the explicit
# byte-order handling in rover_protocol.cpp.
CONTROL_FMT = "<hhBB"   # drive_cmd, steer_cmd, mode, flags
DRIVE_L_FMT = "<ih"     # enc_left, steer_fb
DRIVE_R_FMT = "<iH"     # enc_right, cmd_age_ms
POWER_FMT   = "<hhH"    # current_ca, voltage_cv, fault_status
STATE_FMT   = "<6B"     # mode, jetson_link, c2_link, ctrl_health, indicator, rsvd
ECHO_FMT    = "<hhBB"   # drive_applied, steer_applied, ack_seq, status

INT32_MIN = -(2 ** 31)
INT32_MAX = 2 ** 31 - 1

# --- decode results, mirroring enum DecodeResult ---
DECODE_OK            = 0
DECODE_BAD_DLC       = 1
DECODE_BAD_CRC       = 2
DECODE_BAD_MODE      = 3
DECODE_BAD_FLAGS     = 4
DECODE_BAD_INDICATOR = 5
DECODE_BAD_RANGE     = 6
DECODE_NAMES = {DECODE_OK: "OK", DECODE_BAD_DLC: "BAD_DLC",
                DECODE_BAD_CRC: "BAD_CRC", DECODE_BAD_MODE: "BAD_MODE",
                DECODE_BAD_FLAGS: "BAD_FLAGS",
                DECODE_BAD_INDICATOR: "BAD_INDICATOR",
                DECODE_BAD_RANGE: "BAD_RANGE"}


# ---------------------------------------------------------------------------
# Predicates and helpers -- mirror the C++ exactly
# ---------------------------------------------------------------------------

def fault_names(bitmask):
    return [name for bit, name in FAULT_NAMES if bitmask & bit]


def is_commandable_mode(mode):
    """May the Jetson send this mode in CONTROL? SAFE counts: it is a
    legitimate command that happens to mean "do not move". FAULT does not:
    only the controller enters it. Mirrors isCommandableMode() in C++."""
    return mode in COMMANDABLE_MODES


def mode_permits_motion(mode):
    """May the rover move in this mode? A WHITELIST, deliberately -- `mode` is a
    uint8, so 256 values fit where 4 are defined."""
    return mode in MOTION_MODES


def source_owns_mode(from_autonomy, mode):
    """May this producer command motion in this mode?

    The autonomy stack owns AUTONOMOUS, the operator owns TELEOP. SAFE has
    no owner because nothing may move in it, so the question does not arise and
    the answer is True rather than a spurious mismatch.

    Mirrors sourceOwnsMode() in firmware/src/rover_protocol.cpp. The controller
    is the one that enforces it -- this copy is so the Jetson can predict the
    verdict it is about to read back.
    """
    if not mode_permits_motion(mode):
        return True
    return bool(from_autonomy) == (mode == MODE_AUTONOMOUS)


def is_valid_command(value):
    """Is this drive/steer value inside CMD_MIN..CMD_MAX (+/-100.0%)?

    Out-of-range values are rejected at decode, never clamped: clamping would
    quietly turn a misunderstood command into full throttle. Mirrors
    isValidCommand() in firmware/src/rover_protocol.cpp.
    """
    return CMD_MIN <= value <= CMD_MAX


def is_known_indicator(indicator):
    return 0 <= indicator <= INDICATOR_FAULT


def wrap_i32(value):
    """Wrap an unbounded accumulator into int32 range; mirrors wrapI32()."""
    return ((value - INT32_MIN) % (2 ** 32)) + INT32_MIN


def clamp_cmd_age_ms(age_ms):
    if age_ms < 0:
        return 0
    return age_ms if age_ms < CMD_AGE_MAX else CMD_AGE_MAX


def pack_cmd_status(status, decode=DECODE_OK, return_active=False):
    """Build a TelemetryCmdEcho.status byte; mirrors packCmdStatus()."""
    packed = status & CMD_ECHO_STATUS_MASK
    packed |= (decode << CMD_ECHO_DECODE_SHIFT) & CMD_ECHO_DECODE_MASK
    if return_active:
        packed |= CMD_ECHO_RETURN_ACTIVE
    return packed


def cmd_status_of(packed):
    return packed & CMD_ECHO_STATUS_MASK


def decode_result_of(packed):
    """Why the LAST frame on the CONTROL id was rejected, or DECODE_OK.

    Separate from the verdict on purpose: a frame can be rejected (a bad CRC)
    while an older accepted command is still being applied.
    """
    return (packed & CMD_ECHO_DECODE_MASK) >> CMD_ECHO_DECODE_SHIFT


def return_active_of(packed):
    return bool(packed & CMD_ECHO_RETURN_ACTIVE)


def seq_delta(previous, current):
    """Frames elapsed from `previous` to `current`, correct across the 255->0
    wrap. 1 is healthy, 0 is a duplicate, >1 means frames were lost."""
    return (current - previous) & 0xFF


# ---------------------------------------------------------------------------
# CRC-8 / SAE-J1850: poly 0x1D, init 0xFF, no reflection, final XOR 0xFF.
# Known-answer check value over b"123456789" is 0x4B.
#
# NOTE: for a CRC-8 the byte is XORed into the WHOLE register. For a CRC-16 it
# goes into the high byte. Copying a CRC-16 loop is the classic way to get a
# plausible-looking wrong answer.
# ---------------------------------------------------------------------------

def crc8_update(crc, data):
    """Running CRC with no init and no final XOR, so it can be chained."""
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def crc8(data):
    """Complete CRC over one buffer: init 0xFF, final XOR 0xFF."""
    return crc8_update(0xFF, data) ^ 0xFF


def frame_crc8(can_id, payload):
    """Frame CRC seeded with the message identifier, AUTOSAR Data-ID style, so
    a payload delivered on the wrong CAN id fails validation. The id is chained
    low byte first, matching the protocol's little-endian convention."""
    id_bytes = bytes((can_id & 0xFF, (can_id >> 8) & 0xFF,
                      (can_id >> 16) & 0xFF, (can_id >> 24) & 0xFF))
    return crc8_update(crc8_update(0xFF, id_bytes), payload) ^ 0xFF


def _seal(can_id, payload, seq):
    """Append the sequence number and CRC to a 6-byte payload -> 8 bytes."""
    assert len(payload) == FRAME_PAYLOAD, "payload must be 6 bytes"
    body = payload + bytes((seq & 0xFF,))
    return body + bytes((frame_crc8(can_id, body),))


def _open(can_id, frame):
    """Validate length and CRC. Returns (result, payload, seq)."""
    if len(frame) != FRAME_DLC:
        return DECODE_BAD_DLC, None, None
    body = frame[:FRAME_CRC_OFFSET]
    if frame_crc8(can_id, body) != frame[FRAME_CRC_OFFSET]:
        return DECODE_BAD_CRC, None, None
    return DECODE_OK, frame[:FRAME_PAYLOAD], frame[FRAME_SEQ_OFFSET]


# ---------------------------------------------------------------------------
# CONTROL
# ---------------------------------------------------------------------------

def pack_control_flags(c2_lost, stop=False, autonomy_abort=False,
                       return_request=False, indicator_request=INDICATOR_OFF,
                       from_autonomy=False):
    flags = 0
    if stop:            flags |= CTRL_FLAG_STOP
    if autonomy_abort:  flags |= CTRL_FLAG_AUTONOMY_ABORT
    if return_request:  flags |= CTRL_FLAG_RETURN_REQUEST
    if c2_lost:         flags |= CTRL_FLAG_C2_LOST
    if from_autonomy:   flags |= CTRL_FLAG_FROM_AUTONOMY
    flags |= (indicator_request << CTRL_INDICATOR_SHIFT) & CTRL_INDICATOR_MASK
    return flags


def encode_control(drive_cmd, steer_cmd, mode, c2_lost, stop=False, seq=0,
                   autonomy_abort=False, return_request=False,
                   indicator_request=INDICATOR_OFF, from_autonomy=False,
                   raw_flags=None):
    """Encode a CONTROL frame. `raw_flags` bypasses pack_control_flags so tests
    can put an arbitrary byte on the wire, the way a faulty sender would.

    `c2_lost` has NO DEFAULT on purpose. It is the only field in this frame
    the controller cannot determine for itself, and a caller that forgot it
    would silently report the base-station link as healthy -- which is the one
    direction of error that lets a rover keep driving in TELEOP with nobody
    able to reach it. A missing argument is a loud failure instead.

    `from_autonomy` DOES default, to False, because that default fails safe in
    the direction that matters: a caller who forgets it gets its commands
    refused in AUTONOMOUS, which is visible and harmless. The opposite default
    would let a forgotten argument drive an autonomous rover.
    """
    flags = pack_control_flags(c2_lost, stop, autonomy_abort, return_request,
                               indicator_request,
                               from_autonomy) if raw_flags is None else raw_flags
    payload = struct.pack(CONTROL_FMT, drive_cmd, steer_cmd, mode, flags)
    return _seal(CAN_ID_CONTROL, payload, seq)


def decode_control(frame, can_id=CAN_ID_CONTROL):
    """Returns (result, dict or None). Mirrors decodeControl() exactly."""
    result, payload, seq = _open(can_id, frame)
    if result != DECODE_OK:
        return result, None
    drive, steer, mode, flags = struct.unpack(CONTROL_FMT, payload)
    if not is_commandable_mode(mode):
        return DECODE_BAD_MODE, None
    # No reserved-bit check: v0.6 took bit 7 for from_autonomy and the flags
    # byte is full. A frame whose source does not own the mode is NOT rejected
    # here -- it is well formed, and authority is the controller's decision.
    indicator = (flags & CTRL_INDICATOR_MASK) >> CTRL_INDICATOR_SHIFT
    if not is_known_indicator(indicator):
        return DECODE_BAD_INDICATOR, None
    # Same for an out-of-range drive or steer: reject, never clamp.
    if not (is_valid_command(drive) and is_valid_command(steer)):
        return DECODE_BAD_RANGE, None
    return DECODE_OK, {
        "drive_cmd": drive, "steer_cmd": steer, "mode": mode,
        "stop": bool(flags & CTRL_FLAG_STOP),
        "autonomy_abort": bool(flags & CTRL_FLAG_AUTONOMY_ABORT),
        "return_request": bool(flags & CTRL_FLAG_RETURN_REQUEST),
        "c2_lost": bool(flags & CTRL_FLAG_C2_LOST),
        "from_autonomy": bool(flags & CTRL_FLAG_FROM_AUTONOMY),
        "indicator_request": indicator, "seq": seq,
    }


# ---------------------------------------------------------------------------
# TELEMETRY
# ---------------------------------------------------------------------------

def encode_telemetry_drive_l(enc_left, steer_fb, seq=0):
    return _seal(CAN_ID_TELEM_DRIVE_L,
                 struct.pack(DRIVE_L_FMT, wrap_i32(enc_left), steer_fb), seq)


def encode_telemetry_drive_r(enc_right, cmd_age_ms, seq=0):
    return _seal(CAN_ID_TELEM_DRIVE_R,
                 struct.pack(DRIVE_R_FMT, wrap_i32(enc_right), cmd_age_ms), seq)


def encode_telemetry_power(current_ca, voltage_cv, fault_status, seq=0):
    return _seal(CAN_ID_TELEM_POWER,
                 struct.pack(POWER_FMT, current_ca, voltage_cv, fault_status), seq)


def encode_telemetry_state(mode, jetson_link, c2_link, controller_health,
                           indicator_state, seq=0, reserved=0):
    return _seal(CAN_ID_TELEM_STATE,
                 struct.pack(STATE_FMT, mode, jetson_link, c2_link,
                             controller_health, indicator_state, reserved), seq)


def decode_telemetry_drive_l(frame, can_id=CAN_ID_TELEM_DRIVE_L):
    result, payload, seq = _open(can_id, frame)
    if result != DECODE_OK:
        return result, None
    enc_left, steer_fb = struct.unpack(DRIVE_L_FMT, payload)
    return DECODE_OK, {"enc_left": enc_left, "steer_fb": steer_fb, "seq": seq}


def decode_telemetry_drive_r(frame, can_id=CAN_ID_TELEM_DRIVE_R):
    result, payload, seq = _open(can_id, frame)
    if result != DECODE_OK:
        return result, None
    enc_right, cmd_age = struct.unpack(DRIVE_R_FMT, payload)
    return DECODE_OK, {"enc_right": enc_right, "cmd_age_ms": cmd_age, "seq": seq}


def decode_telemetry_power(frame, can_id=CAN_ID_TELEM_POWER):
    result, payload, seq = _open(can_id, frame)
    if result != DECODE_OK:
        return result, None
    current_ca, voltage_cv, faults = struct.unpack(POWER_FMT, payload)
    return DECODE_OK, {
        "current_ca": current_ca, "current_a": current_ca / 100.0,
        "voltage_cv": voltage_cv, "voltage_v": voltage_cv / 100.0,
        "fault_status": faults, "seq": seq,
    }


def decode_telemetry_state(frame, can_id=CAN_ID_TELEM_STATE):
    result, payload, seq = _open(can_id, frame)
    if result != DECODE_OK:
        return result, None
    mode, jlink, c2link, health, indicator, reserved = struct.unpack(STATE_FMT, payload)
    return DECODE_OK, {
        "mode": mode, "jetson_link": jlink, "c2_link": c2link,
        "controller_health": health, "indicator_state": indicator,
        "reserved": reserved, "seq": seq,
    }


def encode_telemetry_cmd_echo(drive_applied, steer_applied, ack_seq, status,
                              seq=0):
    return _seal(CAN_ID_TELEM_CMD_ECHO,
                 struct.pack(ECHO_FMT, drive_applied, steer_applied,
                             ack_seq, status), seq)


def decode_telemetry_cmd_echo(frame, can_id=CAN_ID_TELEM_CMD_ECHO):
    result, payload, seq = _open(can_id, frame)
    if result != DECODE_OK:
        return result, None
    drive, steer, ack_seq, status = struct.unpack(ECHO_FMT, payload)
    return DECODE_OK, {
        "drive_applied": drive, "steer_applied": steer,
        "ack_seq": ack_seq, "cmd_status_raw": status,
        # Unpacked for the caller, because every consumer wants the parts and
        # nobody wants to re-derive the shifts.
        "cmd_status": cmd_status_of(status),
        "last_decode": decode_result_of(status),
        "return_active": return_active_of(status),
        "seq": seq,
    }


DECODERS = {
    CAN_ID_TELEM_DRIVE_L: decode_telemetry_drive_l,
    CAN_ID_TELEM_DRIVE_R: decode_telemetry_drive_r,
    CAN_ID_TELEM_POWER:   decode_telemetry_power,
    CAN_ID_TELEM_STATE:   decode_telemetry_state,
    CAN_ID_TELEM_CMD_ECHO: decode_telemetry_cmd_echo,
}
