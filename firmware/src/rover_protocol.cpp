// rover_protocol.cpp -- little-endian serialization, CRC-8 and validation for
// the rover CAN messages. See rover_protocol.h for the design rationale.

#include "rover_protocol.h"

namespace rover {
namespace {

// Explicit little-endian put/get. Byte-at-a-time rather than memcpy of a
// packed struct, so nothing here depends on compiler padding or host
// endianness. This is the C++ counterpart of Python's '<' struct prefix.

inline void putU16(uint8_t* b, uint16_t v) {
    b[0] = static_cast<uint8_t>(v & 0xFF);
    b[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

inline void putU32(uint8_t* b, uint32_t v) {
    b[0] = static_cast<uint8_t>(v & 0xFF);
    b[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    b[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
    b[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

inline uint16_t getU16(const uint8_t* b) {
    return static_cast<uint16_t>(b[0]) | (static_cast<uint16_t>(b[1]) << 8);
}

inline uint32_t getU32(const uint8_t* b) {
    return static_cast<uint32_t>(b[0])
         | (static_cast<uint32_t>(b[1]) << 8)
         | (static_cast<uint32_t>(b[2]) << 16)
         | (static_cast<uint32_t>(b[3]) << 24);
}

// Signed values go over the wire as two's-complement bit patterns. Converting
// through the unsigned type is the defined way to do this: casting a too-large
// unsigned straight to a signed type is implementation-defined before C++20.
inline void putI16(uint8_t* b, int16_t v) { putU16(b, static_cast<uint16_t>(v)); }
inline void putI32(uint8_t* b, int32_t v) { putU32(b, static_cast<uint32_t>(v)); }

inline int16_t getI16(const uint8_t* b) {
    uint16_t u = getU16(b);
    return (u & 0x8000u) ? static_cast<int16_t>(static_cast<int32_t>(u) - 65536)
                         : static_cast<int16_t>(u);
}

inline int32_t getI32(const uint8_t* b) {
    uint32_t u = getU32(b);
    return (u & 0x80000000u)
        ? static_cast<int32_t>(static_cast<int64_t>(u) - 4294967296LL)
        : static_cast<int32_t>(u);
}

// Append the sequence number and CRC to a 6-byte payload, producing DLC 8.
inline uint8_t sealFrame(uint32_t can_id, uint8_t seq, uint8_t* buf) {
    buf[FRAME_SEQ_OFFSET] = seq;
    buf[FRAME_CRC_OFFSET] = frameCrc8(can_id, buf, FRAME_SEQ_OFFSET + 1);
    return FRAME_DLC;
}

// Shared front half of every decode: length, then CRC. Returns DECODE_OK and
// writes the sequence number out, or the reason it failed.
inline DecodeResult openFrame(uint32_t can_id, const uint8_t* buf, uint8_t len,
                              uint8_t& seq_out) {
    if (len != FRAME_DLC) return DECODE_BAD_DLC;
    if (frameCrc8(can_id, buf, FRAME_SEQ_OFFSET + 1) != buf[FRAME_CRC_OFFSET]) {
        return DECODE_BAD_CRC;
    }
    seq_out = buf[FRAME_SEQ_OFFSET];
    return DECODE_OK;
}

}  // namespace

// ---------------------------------------------------------------------------
// CRC-8 / SAE-J1850: poly 0x1D, init 0xFF, MSB-first, final XOR 0xFF.
//
// NOTE FOR ANYONE PORTING THIS: for a CRC-8 the data byte is XORed into the
// WHOLE register (`crc ^= byte`). For a CRC-16 you would XOR it into the high
// byte (`crc ^= byte << 8`). Copying a CRC-16 loop and changing the width is
// the classic way to get a plausible-looking wrong answer. The known-answer
// test in tests/cpp/test_protocol.cpp exists to catch exactly that.
// ---------------------------------------------------------------------------

uint8_t crc8Update(uint8_t crc, const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x1D)
                               : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

uint8_t crc8(const uint8_t* data, size_t len) {
    return static_cast<uint8_t>(crc8Update(0xFF, data, len) ^ 0xFF);
}

uint8_t frameCrc8(uint32_t can_id, const uint8_t* payload, size_t len) {
    // The id is chained low byte first, matching the protocol's little-endian
    // convention. All four bytes are included so an 11-bit and a 29-bit id
    // with the same low bits cannot collide.
    const uint8_t id_bytes[4] = {
        static_cast<uint8_t>(can_id & 0xFF),
        static_cast<uint8_t>((can_id >> 8) & 0xFF),
        static_cast<uint8_t>((can_id >> 16) & 0xFF),
        static_cast<uint8_t>((can_id >> 24) & 0xFF),
    };
    uint8_t crc = crc8Update(0xFF, id_bytes, 4);
    crc = crc8Update(crc, payload, len);
    return static_cast<uint8_t>(crc ^ 0xFF);
}

// ---------------------------------------------------------------------------
// Validation predicates
// ---------------------------------------------------------------------------

bool isCommandableMode(uint8_t mode) {
    return mode == MODE_SAFE || mode == MODE_TELEOP || mode == MODE_AUTONOMOUS;
}

bool modePermitsMotion(uint8_t mode) {
    // Whitelist. Every value not named here -- including SAFE and every
    // undefined value -- means stop. A safety predicate must fail CLOSED.
    return mode == MODE_TELEOP || mode == MODE_AUTONOMOUS;
}

bool sourceOwnsMode(bool from_autonomy, uint8_t mode) {
    // Only modes that permit motion have an owner -- in SAFE nothing may
    // move, so there is nothing for a source to be wrong about and a caller
    // asking the question gets "yes" rather than a spurious mismatch.
    if (!modePermitsMotion(mode)) return true;
    return from_autonomy == (mode == MODE_AUTONOMOUS);
}

bool isValidCommand(int16_t value) {
    return value >= CMD_MIN && value <= CMD_MAX;
}

bool isKnownIndicator(uint8_t indicator) {
    return indicator <= INDICATOR_FAULT;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

int32_t wrapI32(int64_t value) {
    // Shift into unsigned range, wrap with a modulo well defined for
    // negatives, then shift back. Avoids signed overflow entirely, which in
    // C++ is undefined behaviour rather than a wrap.
    const int64_t span = 4294967296LL;  // 2^32
    int64_t shifted = (value - INT32_MIN_V) % span;
    if (shifted < 0) shifted += span;
    return static_cast<int32_t>(shifted + INT32_MIN_V);
}

uint16_t clampCmdAgeMs(int64_t age_ms) {
    if (age_ms < 0) return 0;
    if (age_ms >= static_cast<int64_t>(CMD_AGE_MAX)) return CMD_AGE_MAX;
    return static_cast<uint16_t>(age_ms);
}

uint8_t seqDelta(uint8_t previous, uint8_t current) {
    // Unsigned subtraction, so this is correct across the 255->0 wrap without
    // any special case. Same reasoning as the millis() handling in
    // rover_controller.cpp: subtract, never compare.
    return static_cast<uint8_t>(current - previous);
}

size_t faultNames(uint16_t bitmask, const char** out, size_t cap) {
    static const uint16_t bits[] = {
        FAULT_COMM_TIMEOUT, FAULT_OVER_CURRENT, FAULT_ESTOP_ACTIVE,
        FAULT_ENCODER_FAULT, FAULT_UNDERVOLTAGE, FAULT_FIRMWARE_FAULT,
        FAULT_PROTOCOL_ERROR, FAULT_C2_LINK_LOST, FAULT_SEQ_GAP, FAULT_CRC_ERROR,
    };
    static const char* names[] = {
        "COMM_TIMEOUT", "OVER_CURRENT", "ESTOP_ACTIVE",
        "ENCODER_FAULT", "UNDERVOLTAGE", "FIRMWARE_FAULT",
        "PROTOCOL_ERROR", "C2_LINK_LOST", "SEQ_GAP", "CRC_ERROR",
    };
    size_t n = 0;
    for (size_t i = 0; i < sizeof(bits) / sizeof(bits[0]) && n < cap; ++i) {
        if (bitmask & bits[i]) out[n++] = names[i];
    }
    return n;
}

const char* decodeResultName(DecodeResult r) {
    switch (r) {
        case DECODE_OK:            return "OK";
        case DECODE_BAD_DLC:       return "BAD_DLC";
        case DECODE_BAD_CRC:       return "BAD_CRC";
        case DECODE_BAD_MODE:      return "BAD_MODE";
        case DECODE_BAD_FLAGS:     return "BAD_FLAGS";
        case DECODE_BAD_INDICATOR: return "BAD_INDICATOR";
        case DECODE_BAD_RANGE:     return "BAD_RANGE";
    }
    return "UNKNOWN";
}

// ---------------------------------------------------------------------------
// CONTROL
// ---------------------------------------------------------------------------

uint8_t encodeControl(const ControlMsg& msg, uint8_t* buf) {
    putI16(buf + 0, msg.drive_cmd);
    putI16(buf + 2, msg.steer_cmd);
    buf[4] = msg.mode;
    uint8_t flags = 0;
    if (msg.stop)            flags |= CTRL_FLAG_STOP;
    if (msg.autonomy_abort)  flags |= CTRL_FLAG_AUTONOMY_ABORT;
    if (msg.return_request)  flags |= CTRL_FLAG_RETURN_REQUEST;
    if (msg.c2_lost)         flags |= CTRL_FLAG_C2_LOST;
    if (msg.from_autonomy)   flags |= CTRL_FLAG_FROM_AUTONOMY;
    flags |= static_cast<uint8_t>((msg.indicator_request << CTRL_INDICATOR_SHIFT)
                                  & CTRL_INDICATOR_MASK);
    buf[5] = flags;
    return sealFrame(CAN_ID_CONTROL, msg.seq, buf);
}

DecodeResult decodeControl(uint32_t can_id, const uint8_t* buf, uint8_t len,
                           ControlMsg& out) {
    uint8_t seq = 0;
    DecodeResult r = openFrame(can_id, buf, len, seq);
    if (r != DECODE_OK) return r;

    const uint8_t mode  = buf[4];
    const uint8_t flags = buf[5];

    // A mode the Jetson may not send (undefined, or FAULT, which only the
    // controller enters) means the sender disagrees with us about the
    // protocol. Reject the whole frame rather than storing a value we cannot reason
    // about. Deliberately NOT forward-compatible: a newer peer sending a mode
    // this firmware does not implement must stop this rover.
    if (!isCommandableMode(mode)) return DECODE_BAD_MODE;

    // No reserved-bit check remains: v0.6 took bit 7 for from_autonomy and
    // the flags byte is full. A frame whose source does not own the mode is
    // NOT rejected here -- it is well formed, and refusing it would stop the
    // rover over a Jetson-side mux bug instead of reporting one. Authority is
    // the controller's decision; see RoverController::commandAuthorized().

    const uint8_t indicator =
        static_cast<uint8_t>((flags & CTRL_INDICATOR_MASK) >> CTRL_INDICATOR_SHIFT);
    if (!isKnownIndicator(indicator)) return DECODE_BAD_INDICATOR;

    // Same reasoning for an out-of-range drive or steer: reject, never clamp.
    // See isValidCommand() in rover_protocol.h.
    const int16_t drive = getI16(buf + 0);
    const int16_t steer = getI16(buf + 2);
    if (!isValidCommand(drive) || !isValidCommand(steer)) return DECODE_BAD_RANGE;

    out.drive_cmd        = drive;
    out.steer_cmd        = steer;
    out.mode             = mode;
    out.stop             = (flags & CTRL_FLAG_STOP) != 0;
    out.autonomy_abort   = (flags & CTRL_FLAG_AUTONOMY_ABORT) != 0;
    out.return_request   = (flags & CTRL_FLAG_RETURN_REQUEST) != 0;
    out.c2_lost          = (flags & CTRL_FLAG_C2_LOST) != 0;
    out.from_autonomy    = (flags & CTRL_FLAG_FROM_AUTONOMY) != 0;
    out.indicator_request = indicator;
    out.seq              = seq;
    return DECODE_OK;
}

// ---------------------------------------------------------------------------
// TELEMETRY
// ---------------------------------------------------------------------------

uint8_t encodeTelemetryDriveL(const TelemetryDriveL& msg, uint8_t seq, uint8_t* buf) {
    putI32(buf + 0, msg.enc_left);
    putI16(buf + 4, msg.steer_fb);
    return sealFrame(CAN_ID_TELEM_DRIVE_L, seq, buf);
}

DecodeResult decodeTelemetryDriveL(uint32_t can_id, const uint8_t* buf, uint8_t len,
                                   TelemetryDriveL& out, uint8_t& seq_out) {
    DecodeResult r = openFrame(can_id, buf, len, seq_out);
    if (r != DECODE_OK) return r;
    out.enc_left = getI32(buf + 0);
    out.steer_fb = getI16(buf + 4);
    return DECODE_OK;
}

uint8_t encodeTelemetryDriveR(const TelemetryDriveR& msg, uint8_t seq, uint8_t* buf) {
    putI32(buf + 0, msg.enc_right);
    putU16(buf + 4, msg.cmd_age_ms);
    return sealFrame(CAN_ID_TELEM_DRIVE_R, seq, buf);
}

DecodeResult decodeTelemetryDriveR(uint32_t can_id, const uint8_t* buf, uint8_t len,
                                   TelemetryDriveR& out, uint8_t& seq_out) {
    DecodeResult r = openFrame(can_id, buf, len, seq_out);
    if (r != DECODE_OK) return r;
    out.enc_right   = getI32(buf + 0);
    out.cmd_age_ms  = getU16(buf + 4);
    return DECODE_OK;
}

uint8_t encodeTelemetryPower(const TelemetryPower& msg, uint8_t seq, uint8_t* buf) {
    putI16(buf + 0, msg.current_ca);
    putI16(buf + 2, msg.voltage_cv);
    putU16(buf + 4, msg.fault_status);
    return sealFrame(CAN_ID_TELEM_POWER, seq, buf);
}

DecodeResult decodeTelemetryPower(uint32_t can_id, const uint8_t* buf, uint8_t len,
                                  TelemetryPower& out, uint8_t& seq_out) {
    DecodeResult r = openFrame(can_id, buf, len, seq_out);
    if (r != DECODE_OK) return r;
    out.current_ca   = getI16(buf + 0);
    out.voltage_cv   = getI16(buf + 2);
    out.fault_status = getU16(buf + 4);
    return DECODE_OK;
}

// ---------------------------------------------------------------------------
// Command-status byte
// ---------------------------------------------------------------------------

// Both fields are three bits wide, so the enums they carry have to fit. A
// seventh DecodeResult or a fifth CMD_STATUS_* is fine; an eighth is not, and
// would otherwise silently truncate onto the wire.
static_assert(DECODE_BAD_RANGE <= 7, "DecodeResult no longer fits in 3 bits");
static_assert(CMD_STATUS_NO_COMMAND <= 7, "CMD_STATUS_* no longer fits in 3 bits");

uint8_t packCmdStatus(uint8_t status, DecodeResult decode, bool return_active) {
    uint8_t packed = static_cast<uint8_t>(status & CMD_ECHO_STATUS_MASK);
    packed = static_cast<uint8_t>(
        packed | ((static_cast<uint8_t>(decode) << CMD_ECHO_DECODE_SHIFT)
                  & CMD_ECHO_DECODE_MASK));
    if (return_active) packed = static_cast<uint8_t>(packed | CMD_ECHO_RETURN_ACTIVE);
    return packed;
}

uint8_t cmdStatusOf(uint8_t packed) {
    return static_cast<uint8_t>(packed & CMD_ECHO_STATUS_MASK);
}

DecodeResult decodeResultOf(uint8_t packed) {
    return static_cast<DecodeResult>(
        (packed & CMD_ECHO_DECODE_MASK) >> CMD_ECHO_DECODE_SHIFT);
}

bool returnActiveOf(uint8_t packed) {
    return (packed & CMD_ECHO_RETURN_ACTIVE) != 0;
}

const char* cmdStatusName(uint8_t status) {
    switch (status) {
        case CMD_STATUS_APPLIED:      return "APPLIED";
        case CMD_STATUS_STOPPED:      return "STOPPED";
        case CMD_STATUS_WRONG_SOURCE: return "WRONG_SOURCE";
        case CMD_STATUS_NO_COMMAND:   return "NO_COMMAND";
    }
    return "UNKNOWN";
}

uint8_t encodeTelemetryState(const TelemetryState& msg, uint8_t seq, uint8_t* buf) {
    buf[0] = msg.mode;
    buf[1] = msg.jetson_link;
    buf[2] = msg.c2_link;
    buf[3] = msg.controller_health;
    buf[4] = msg.indicator_state;
    buf[5] = msg.reserved;
    return sealFrame(CAN_ID_TELEM_STATE, seq, buf);
}

DecodeResult decodeTelemetryState(uint32_t can_id, const uint8_t* buf, uint8_t len,
                                  TelemetryState& out, uint8_t& seq_out) {
    DecodeResult r = openFrame(can_id, buf, len, seq_out);
    if (r != DECODE_OK) return r;
    out.mode              = buf[0];
    out.jetson_link       = buf[1];
    out.c2_link           = buf[2];
    out.controller_health = buf[3];
    out.indicator_state   = buf[4];
    out.reserved          = buf[5];
    return DECODE_OK;
}

uint8_t encodeTelemetryCmdEcho(const TelemetryCmdEcho& msg, uint8_t seq, uint8_t* buf) {
    putI16(buf + 0, msg.drive_applied);
    putI16(buf + 2, msg.steer_applied);
    buf[4] = msg.ack_seq;
    buf[5] = msg.status;
    return sealFrame(CAN_ID_TELEM_CMD_ECHO, seq, buf);
}

DecodeResult decodeTelemetryCmdEcho(uint32_t can_id, const uint8_t* buf, uint8_t len,
                                    TelemetryCmdEcho& out, uint8_t& seq_out) {
    DecodeResult r = openFrame(can_id, buf, len, seq_out);
    if (r != DECODE_OK) return r;
    out.drive_applied = getI16(buf + 0);
    out.steer_applied = getI16(buf + 2);
    out.ack_seq       = buf[4];
    out.status        = buf[5];
    return DECODE_OK;
}

}  // namespace rover
