// rover_config.h -- Every protocol and timing constant, in one place.
//
// PURE C++, like the rest of firmware/src: compiles for the Teensy and
// natively for the tests. host/rover_config.py mirrors this file, and
// tests/host/test_config_sync.py fails if the two ever disagree.
//
// What belongs here: values someone might tune or that both ends must agree
// on (bit rate, timeouts, CAN ids, frame sizes, valid ranges, version).
// What does not: values that describe a message's CONTENTS -- fault bits,
// flag masks, indicator codes, link-state codes, command-age sentinels, the
// CRC polynomial -- those stay in rover_protocol.h, because changing one of
// them changes the protocol rather than tuning it.

#ifndef ROVER_CONFIG_H
#define ROVER_CONFIG_H

#include <stdint.h>

namespace rover {

// --- Protocol version ------------------------------------------------------
// Bump whenever a message's layout or meaning changes. Not carried on the
// wire yet; DLC and the application-layer CRC are the only runtime mismatch
// signals. v0.5 is the first revision with the CRC and sequence number; v0.6
// adds the command-source bit and the TELEM_CMD_ECHO acknowledgement frame.
static const uint8_t PROTOCOL_VERSION = 6;

// --- Link ------------------------------------------------------------------
static const uint32_t CAN_BITRATE_HZ = 500000;   // 500 kbps; every node must match

// --- Timing ----------------------------------------------------------------
static const uint32_t DEFAULT_WATCHDOG_TIMEOUT_MS = 300;   // no valid CONTROL -> stop
static const uint32_t DEFAULT_TELEMETRY_PERIOD_MS = 50;    // 20 Hz
static const uint32_t HW_WATCHDOG_MS              = 1000;  // Teensy resets if the loop stalls

// How long a link-quality complaint (a sequence gap or a CRC error) keeps the
// Jetson link reported as DEGRADED after the last occurrence. Long enough that
// an operator actually sees it on a dashboard refreshing at 20 Hz.
static const uint32_t LINK_DEGRADED_HOLD_MS = 1000;

// --- CAN identifiers -------------------------------------------------------
// On CAN the identifier is ALSO the priority: arbitration is bitwise and
// dominant-low, so the NUMERICALLY LOWEST id wins the bus. Commands outrank
// telemetry because a late command can hurt the rover and late telemetry only
// annoys an operator. 0x000-0x0FF is left free for a future e-stop frame.
//
// Telemetry is five frames rather than one: see the snapshot-tearing note in
// rover_protocol.h, which is about what the split MEANS and so stays there.
enum : uint32_t {
    CAN_ID_CONTROL        = 0x100,  // Jetson -> Teensy, highest priority in use
    CAN_ID_TELEM_DRIVE_L  = 0x200,  // left encoder + steering feedback
    CAN_ID_TELEM_DRIVE_R  = 0x201,  // right encoder + command age
    CAN_ID_TELEM_POWER    = 0x202,  // current, voltage, fault word
    CAN_ID_TELEM_STATE    = 0x203,  // mode, link health, controller health
    CAN_ID_TELEM_CMD_ECHO = 0x204,  // what the controller accepted and is acting on
};

// Bus load, so nobody has to recompute it when adding the sixth frame: one
// CONTROL plus five telemetry frames at 20 Hz is 120 frames/s. A Classic CAN
// frame with an 11-bit id and 8 data bytes is ~128 bits once stuffing is
// allowed for, so ~15.4 kbit/s of 500 kbit/s -- about 3%. There is room, but
// it is no longer free: a sixth telemetry frame is a decision, not a detail.

// --- Frame sizes -----------------------------------------------------------
// Every frame is 8 bytes ON PURPOSE: that is Classic CAN's limit, so this
// protocol runs unchanged on a Classic bus (CAN1/CAN2) or a CAN FD bus (CAN3),
// alongside Classic-only motor controllers. 6 payload bytes, then the
// sequence number, then the CRC.
enum : uint8_t {
    FRAME_DLC        = 8,
    FRAME_PAYLOAD    = 6,  // bytes 0..5
    FRAME_SEQ_OFFSET = 6,
    FRAME_CRC_OFFSET = 7,
};

// --- Operating modes -------------------------------------------------------
// The Jetson commands SAFE, TELEOP or AUTONOMOUS in ControlMsg::mode; the
// controller enters FAULT on its own (RoverController::activeMode()) and a
// CONTROL frame carrying it is rejected.
enum : uint8_t {
    MODE_SAFE       = 0,  // stopped on purpose
    MODE_TELEOP     = 1,  // an operator is driving
    MODE_AUTONOMOUS = 2,  // the Jetson is driving
    MODE_FAULT      = 3,  // stopped because a link the mode needs is gone
};
static const uint8_t MODE_MAX = MODE_AUTONOMOUS;   // highest COMMANDABLE mode

// --- Sensor thresholds -----------------------------------------------------
// Placeholders until the real sensors are characterised, but they belong here
// rather than in rover_firmware.cpp: they are values someone will tune, not board
// wiring. Units match the telemetry fields they are compared against.
static const int16_t OVER_CURRENT_CA = 4000;   // 40.00 A
static const int16_t UNDERVOLTAGE_CV = 2000;   // 20.00 V

// --- Drive / steering output stage -----------------------------------------
// Where a command becomes a motor output. Servo/ESC pulse widths are the
// lingua franca of hobby and small-rover drive hardware: 1500 us is neutral
// and +/-500 us is full scale in either direction, which is what CMD_MIN and
// CMD_MAX map onto. A drive stage that speaks something else (a CAN motor
// controller, a DAC) changes these and nothing above them.
static const uint16_t PWM_NEUTRAL_US = 1500;
static const uint16_t PWM_SPAN_US    = 500;   // full scale either side of neutral

// --- Valid command ranges --------------------------------------------------
// drive_cmd and steer_cmd are tenths of a percent, so +/-1000 is full scale.
// Anything outside is rejected at decode (see isValidCommand()).
static const int16_t CMD_MIN = -1000;
static const int16_t CMD_MAX =  1000;

}  // namespace rover

#endif  // ROVER_CONFIG_H
