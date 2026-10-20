// rover_controller.h -- The rover's command/safety state machine.
//
// PURE C++. No Arduino.h, no FlexCAN, no hardware, and the clock is injected.
// That is the point: this is the safety-critical code, so it must be testable
// natively with a fake clock rather than only observable by watching a board.
//
// THE millis() WRAP, which the CircuitPython original did not have
//   CircuitPython's monotonic_ns() is a 64-bit nanosecond counter that will not
//   wrap in any mission this rover survives. Arduino millis() is uint32_t and
//   wraps every ~49.7 days. Unsigned subtraction handles that correctly on its
//   own -- (now - then) is computed modulo 2^32 -- but comparing timestamps
//   directly (now >= then + timeout) does NOT: it breaks the moment the counter
//   wraps past the deadline. So every time comparison here is an elapsed-time
//   SUBTRACTION, never a comparison of two timestamps. The same reasoning
//   applies to sequence numbers; see seqDelta().

#ifndef ROVER_CONTROLLER_H
#define ROVER_CONTROLLER_H

#include <stdint.h>

#include "rover_protocol.h"

namespace rover {

// Injected clock. On the Teensy this is `millis`. In tests it is a counter the
// test advances by hand, so watchdog behaviour is checked deterministically
// instead of with sleeps.
typedef uint32_t (*MillisFn)();

// DEFAULT_WATCHDOG_TIMEOUT_MS, DEFAULT_TELEMETRY_PERIOD_MS and
// LINK_DEGRADED_HOLD_MS come from rover_config.h (via rover_protocol.h).

class RoverController {
public:
    explicit RoverController(MillisFn now_ms,
                             uint32_t watchdog_timeout_ms = DEFAULT_WATCHDOG_TIMEOUT_MS,
                             uint32_t telemetry_period_ms = DEFAULT_TELEMETRY_PERIOD_MS);

    // Feed one received CAN frame. Returns true only if it was a CONTROL frame
    // that fully validated (right id, right DLC, good CRC, known mode, clean
    // flags, drive/steer within CMD_MIN..CMD_MAX). Anything else is ignored
    // and -- critically -- does NOT refresh the command watchdog.
    bool ingestFrame(uint32_t can_id, const uint8_t* buf, uint8_t len);

    // --- safety ----------------------------------------------------------

    uint16_t cmdAgeMs() const;
    bool watchdogTripped() const;

    // The mode the rover is actually EXECUTING, which is not always the one it
    // was commanded: MODE_FAULT when a link that mode needs is gone -- the
    // Jetson (comm timeout, any mode, including before the first frame) or the
    // operator (C2 lost while in TELEOP). Otherwise the commanded mode.
    // Recomputed on every call, not latched: once the link is back, the rover
    // is in whatever mode was last commanded.
    uint8_t activeMode() const;

    // THE fail-safe decision. An explicit stop flag, an active mode that does
    // not positively permit motion (which covers SAFE, a comm timeout and a
    // lost C2 link in TELEOP, via MODE_FAULT) and an autonomy abort while in
    // AUTONOMOUS all force a stop through this one function, so there is
    // exactly one place where "should the rover be moving?" is answered.
    //
    // The comm timeout and the C2 rule are deliberately different. A silent
    // Jetson stops the rover in EVERY mode, because nothing is driving it. C2
    // loss stops it only in TELEOP, because an autonomous Jetson keeps driving
    // perfectly well without the base station -- and the autonomy course is
    // laid out so that it has to.
    //
    // The mode test is a whitelist (modePermitsMotion), not "== SAFE":
    // asking only about SAFE let every undefined mode read as drivable.
    bool effectiveStop() const;

    // The drive/steer actually permitted right now: zeroed under stop AND
    // under an authority mismatch, so a caller cannot accidentally act on a
    // stale command or on one from the wrong half of the stack.
    void commandedOutputs(int16_t& drive, int16_t& steer) const;

    // --- command authority ------------------------------------------------

    // Does the producer of the last accepted CONTROL frame own the current
    // mode? The autonomy stack owns AUTONOMOUS, the operator owns TELEOP, and
    // SAFE has no owner because nothing may move in it.
    //
    // THIS IS A MUX, AND IT LIVES ON THE MCU ON PURPOSE. The Jetson could
    // just as easily refuse to emit an autonomy command while teleoperated --
    // and it should -- but then a single Jetson-side bug, a stale queue or a
    // mode change that races a command in flight is enough to drive an
    // autonomous rover from the teleop path. The controller is the one place
    // that sees the mode and the command in the same frame, so it is the one
    // place the two can be checked against each other.
    //
    // It gates MOTION and the indicator request, never the safety flags: stop,
    // autonomy_abort and c2_lost are honoured from either source, because
    // anyone who can see the rover is in trouble has to be able to stop it.
    bool commandAuthorized() const;

    // The authority failure worth REPORTING, which is not the same question.
    // A stale frame's source is as stale as the rest of it, so this stays
    // false once the watchdog has tripped -- the same rule that keeps
    // c2LinkState() from vouching for a value it can no longer see.
    bool sourceMismatch() const;

    // CMD_STATUS_*: the single most important reason the drive stage is
    // seeing what it is seeing. See the enum in rover_protocol.h for the
    // precedence, which is NO_COMMAND, then STOPPED, then WRONG_SOURCE.
    uint8_t commandStatus() const;

    // The controller is holding a return-to-base request.
    //
    // REPORTED, NOT ACTED ON, and that is deliberate. Driving the rover back
    // is route planning, which Task 4 explicitly places on the autonomy side,
    // and PROTOCOL.md section 9 leaves the stop/abort/return semantics as a
    // team decision rather than a firmware one. So the controller does the
    // part only it can do -- acknowledge the request so the Jetson can prove
    // it arrived -- and changes no safety behaviour. effectiveStop() is
    // untouched.
    //
    // Only meaningful in AUTONOMOUS: an autonomous return is the thing being
    // requested, and under teleoperation the operator is already driving.
    bool returnRequestActive() const;

    // --- transmit scheduling ---------------------------------------------

    // True once per telemetry period. Self-pacing, and it will not burst out a
    // backlog after a stalled loop. Advances the telemetry sequence number, so
    // call it ONCE per cycle and stamp all four frames with telemetrySeq().
    bool telemetryDue();

    // The sequence number for the current cycle. All four telemetry frames of
    // one cycle share it, which is how the Jetson tells a coherent snapshot
    // from one torn across two cycles.
    uint8_t telemetrySeq() const { return telemetry_seq_; }

    // --- telemetry builders ----------------------------------------------

    TelemetryDriveL buildDriveL(int32_t enc_left, int16_t steer_fb) const;
    TelemetryDriveR buildDriveR(int32_t enc_right) const;
    TelemetryPower  buildPower(int16_t current_ca, int16_t voltage_cv,
                               uint16_t sensor_faults = 0,
                               uint16_t extra_faults = 0) const;
    TelemetryState  buildState(uint8_t controller_health = CTRL_HEALTH_NOT_REPORTED) const;

    // The acknowledgement frame. drive_applied/steer_applied are passed in
    // rather than read back from a drive module, so this class keeps knowing
    // nothing about the output stage -- the same shape as buildDriveL() taking
    // sensor readings. Pass what commandedOutputs() returned.
    TelemetryCmdEcho buildCmdEcho(int16_t drive_applied, int16_t steer_applied) const;

    // Every fault the controller itself knows about, OR-ed together. Exposed
    // so the firmware can report the same word it acts on.
    uint16_t faultWord(uint16_t sensor_faults = 0, uint16_t extra_faults = 0) const;

    // --- link and indicator state ----------------------------------------

    // LINK_* for the Jetson link, derived from the command watchdog and from
    // recent sequence gaps or CRC errors. NOT merged with the C2 link: the
    // 2027 autonomy course deliberately includes areas with no C2
    // line-of-sight while onboard autonomy keeps working, so collapsing the
    // two would stop the rover exactly where it is supposed to keep going.
    uint8_t jetsonLinkState() const;

    // LINK_* for the base-station (C2) link. REPORTED, never derived from the
    // Jetson link: the controller has no radio and cannot observe C2 at all,
    // so this is only ever the c2_lost bit the Jetson forwarded.
    //
    // PLACEHOLDER FOR THE PLANNED CONTROLLER-SIDE RADIO: this function is the
    // single swap point. c2_link is already a locally-measurable enum in a
    // byte of its own -- jetson_link beside it is measured locally using the
    // same four values -- so measuring C2 here instead of taking it from the
    // wire changes no byte of any frame. Read PROTOCOL.md 3.5.2 first: the
    // three open questions are policy, not format, and the debounce one is
    // the one that bites. A raw local reading has none of the smoothing
    // C2Monitor applies on the Jetson today, so marginal RF would stop and
    // release a TELEOP rover repeatedly.
    //
    // It goes LINK_NOT_REPORTED in two cases, both of them honest "we do not
    // know" rather than a misleading LINK_OK:
    //   * no valid CONTROL frame has ever arrived -- nobody has told us;
    //   * the command watchdog has tripped -- what we were told is STALE. A
    //     clear bit from ten seconds ago does not mean C2 is fine now, and a
    //     set bit does not mean it is still down.
    uint8_t c2LinkState() const;

    // What the status indicator should actually show. URC scores this light on
    // what the rover is ACTUALLY doing, and the controller knows its own mode,
    // so the Jetson's request is honoured only where it agrees with that mode.
    // See PROTOCOL.md 3.5 for the rule table.
    uint8_t indicatorState() const;

    // --- accessors --------------------------------------------------------

    const ControlMsg& lastControl() const { return last_control_; }
    bool autonomyAbort() const { return last_control_.autonomy_abort; }
    bool returnRequest() const { return last_control_.return_request; }
    bool c2Lost() const { return last_control_.c2_lost; }
    bool commandFromAutonomy() const { return last_control_.from_autonomy; }

    uint32_t controlFramesAccepted() const { return control_frames_accepted_; }
    uint32_t framesIgnored() const { return frames_ignored_; }
    uint32_t seqGapsSeen() const { return seq_gaps_seen_; }
    uint32_t crcErrorsSeen() const { return crc_errors_seen_; }
    uint32_t framesLostEstimate() const { return frames_lost_estimate_; }

    // The reason the most recent frame on CAN_ID_CONTROL was rejected, or
    // DECODE_OK if it was accepted.
    DecodeResult lastDecodeResult() const { return last_decode_; }

private:
    uint32_t elapsedSinceControl() const;
    bool recentlyDegraded() const;

    MillisFn now_ms_;
    uint32_t watchdog_timeout_ms_;
    uint32_t telemetry_period_ms_;

    ControlMsg last_control_;
    bool       have_control_;
    uint32_t   last_control_ms_;
    uint32_t   next_telemetry_ms_;
    uint8_t    telemetry_seq_;

    bool       have_seq_;
    uint8_t    last_seq_;

    DecodeResult last_decode_;
    bool       protocol_error_;
    bool       crc_error_;
    bool       seq_gap_;
    uint32_t   last_degraded_ms_;
    bool       ever_degraded_;

    uint32_t   control_frames_accepted_;
    uint32_t   frames_ignored_;
    uint32_t   seq_gaps_seen_;
    uint32_t   crc_errors_seen_;
    uint32_t   frames_lost_estimate_;
};

}  // namespace rover

#endif  // ROVER_CONTROLLER_H
