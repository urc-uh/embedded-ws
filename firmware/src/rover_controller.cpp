#include "rover_controller.h"

namespace rover {

RoverController::RoverController(MillisFn now_ms,
                                 uint32_t watchdog_timeout_ms,
                                 uint32_t telemetry_period_ms)
    : now_ms_(now_ms),
      watchdog_timeout_ms_(watchdog_timeout_ms),
      telemetry_period_ms_(telemetry_period_ms),
      have_control_(false),
      last_control_ms_(0),
      next_telemetry_ms_(now_ms ? now_ms() : 0),
      telemetry_seq_(0),
      have_seq_(false),
      last_seq_(0),
      last_decode_(DECODE_OK),
      protocol_error_(false),
      crc_error_(false),
      seq_gap_(false),
      last_degraded_ms_(0),
      ever_degraded_(false),
      control_frames_accepted_(0),
      frames_ignored_(0),
      seq_gaps_seen_(0),
      crc_errors_seen_(0),
      frames_lost_estimate_(0) {
    // Boot state is the safest available: stopped, disabled, no command ever
    // received -- so cmdAgeMs() reads CMD_AGE_UNKNOWN, the watchdog is tripped
    // from the first cycle, and the rover cannot move until the Jetson asks.
    last_control_.drive_cmd         = 0;
    last_control_.steer_cmd         = 0;
    last_control_.mode              = MODE_SAFE;
    last_control_.stop              = true;
    last_control_.autonomy_abort    = false;
    last_control_.return_request    = false;
    last_control_.indicator_request = INDICATOR_OFF;
    // Assume the worst about C2 too, matching every other boot default. It is
    // only visible through c2LinkState(), which reports NOT_REPORTED until a
    // real frame arrives, so this cannot masquerade as forwarded state.
    last_control_.c2_lost           = true;
    // Boot as the operator's rover, not autonomy's. The mode boots SAFE
    // so nothing can move either way, but if these two ever get out of step
    // the safe default is the one that refuses autonomous motion until a real
    // frame asks for it.
    last_control_.from_autonomy     = false;
    last_control_.seq               = 0;
}

bool RoverController::ingestFrame(uint32_t can_id, const uint8_t* buf, uint8_t len) {
    // Other traffic on a shared bus is normal, not a fault. Counting it as one
    // would make the protocol-error flag permanently on and therefore useless.
    if (can_id != CAN_ID_CONTROL) { frames_ignored_++; return false; }

    ControlMsg msg;
    const DecodeResult r = decodeControl(can_id, buf, len, msg);
    last_decode_ = r;

    if (r != DECODE_OK) {
        // The peer disagrees with us about the protocol -- a bad CRC, a wrong
        // length, an undefined mode, reserved bits set. It must NOT refresh
        // the watchdog, or a mismatched node could keep the rover alive while
        // sending commands it never actually understood.
        frames_ignored_++;
        if (r == DECODE_BAD_CRC) {
            crc_error_ = true;
            crc_errors_seen_++;
            last_degraded_ms_ = now_ms_();
            ever_degraded_ = true;
        } else {
            protocol_error_ = true;
        }
        return false;
    }

    // Sequence check. seqDelta is an unsigned subtraction, so it is correct
    // across the 255->0 wrap without a special case -- the same "subtract,
    // never compare" rule as the millis() handling.
    //   delta == 1  healthy
    //   delta == 0  duplicate (or a 256-frame loss, which is ~13 s at 20 Hz
    //               and would have tripped the watchdog long before)
    //   delta >  1  delta-1 frames were lost or arrived out of order
    if (have_seq_) {
        const uint8_t delta = seqDelta(last_seq_, msg.seq);
        if (delta != 1) {
            seq_gap_ = true;
            seq_gaps_seen_++;
            if (delta > 1) frames_lost_estimate_ += static_cast<uint32_t>(delta - 1);
            last_degraded_ms_ = now_ms_();
            ever_degraded_ = true;
        } else {
            seq_gap_ = false;
        }
    }
    last_seq_ = msg.seq;
    have_seq_ = true;

    protocol_error_  = false;   // a good frame clears the condition
    crc_error_       = false;
    last_control_    = msg;
    have_control_    = true;
    last_control_ms_ = now_ms_();
    control_frames_accepted_++;
    return true;
}

// Unsigned subtraction, computed modulo 2^32, so this stays correct when
// millis() wraps. Never written as a comparison of two timestamps.
uint32_t RoverController::elapsedSinceControl() const {
    return now_ms_() - last_control_ms_;
}

uint16_t RoverController::cmdAgeMs() const {
    if (!have_control_) return CMD_AGE_UNKNOWN;
    return clampCmdAgeMs(static_cast<int64_t>(elapsedSinceControl()));
}

bool RoverController::watchdogTripped() const {
    if (!have_control_) return true;
    return elapsedSinceControl() >= watchdog_timeout_ms_;
}

uint8_t RoverController::activeMode() const {
    // Nothing is driving: the Jetson has gone silent, or never spoke.
    if (watchdogTripped()) return MODE_FAULT;
    // C2 lost in TELEOP: the operator's commands can no longer reach the
    // Jetson, so nobody is driving. In AUTONOMOUS the Jetson IS driving and is
    // provably alive, or the watchdog above would already have tripped -- and
    // the autonomy course deliberately runs out of base-station line of sight.
    // In SAFE we are stopped anyway.
    if (last_control_.c2_lost && last_control_.mode == MODE_TELEOP) return MODE_FAULT;
    return last_control_.mode;
}

bool RoverController::effectiveStop() const {
    return last_control_.stop
        // Covers SAFE, the comm timeout and C2 lost in TELEOP: see activeMode().
        || !modePermitsMotion(activeMode())
        // An autonomy abort means "stop what you are doing autonomously". In
        // AUTONOMOUS that is a stop; in TELEOP the operator is already in
        // control so it has no autonomous task to abort.
        //
        // NOTE FOR REVIEW: this behaviour is a judgement call, not something
        // the requirement spelled out. The alternative is to expose the flag
        // and let a higher layer act on it -- but a defined field with no
        // effect anywhere is the antipattern we already had to fix once with
        // FAULT_FIRMWARE_FAULT, so it acts here until someone decides better.
        || (last_control_.autonomy_abort && last_control_.mode == MODE_AUTONOMOUS);
}

bool RoverController::commandAuthorized() const {
    if (!have_control_) return false;   // nobody has authority yet
    return sourceOwnsMode(last_control_.from_autonomy, last_control_.mode);
}

bool RoverController::sourceMismatch() const {
    // Suppressed once the frame is stale, for the same reason c2LinkState()
    // suppresses a stale C2 bit: the controller must not report a condition
    // it can no longer observe. A silent Jetson is a comm timeout, which the
    // fault word already says, and claiming a source mismatch on top of it
    // would send an operator chasing the wrong bug.
    if (!have_control_ || watchdogTripped()) return false;
    if (!modePermitsMotion(last_control_.mode)) return false;
    return !commandAuthorized();
}

uint8_t RoverController::commandStatus() const {
    // First match wins; see the CMD_STATUS_* comment in rover_protocol.h.
    if (!have_control_)       return CMD_STATUS_NO_COMMAND;
    if (effectiveStop())      return CMD_STATUS_STOPPED;
    if (!commandAuthorized()) return CMD_STATUS_WRONG_SOURCE;
    return CMD_STATUS_APPLIED;
}

bool RoverController::returnRequestActive() const {
    // Reported, never acted on -- see the header. Not gated on authority:
    // signalling a return is the OPERATOR's call under the 2027 scoring
    // rules (1.e.xi), so it has to survive arriving from the teleop path.
    if (!have_control_ || watchdogTripped()) return false;
    return last_control_.return_request && last_control_.mode == MODE_AUTONOMOUS;
}

void RoverController::commandedOutputs(int16_t& drive, int16_t& steer) const {
    // Two independent reasons to be at zero. A stop is a safety decision; an
    // authority mismatch is "this command is not yours to give right now".
    // They are not the same thing and are reported separately, but both end
    // here, so there is one place where a command becomes an output.
    if (effectiveStop() || !commandAuthorized()) { drive = 0; steer = 0; return; }
    drive = last_control_.drive_cmd;
    steer = last_control_.steer_cmd;
}

bool RoverController::telemetryDue() {
    const uint32_t now = now_ms_();
    // Signed difference of unsigned timestamps: correct across wraparound, and
    // negative while the deadline is still in the future.
    const int32_t since_due = static_cast<int32_t>(now - next_telemetry_ms_);
    if (since_due < 0) return false;

    next_telemetry_ms_ += telemetry_period_ms_;
    // Never let a stalled loop accumulate a backlog it then bursts out.
    if (static_cast<int32_t>(now - next_telemetry_ms_)
            > static_cast<int32_t>(telemetry_period_ms_)) {
        next_telemetry_ms_ = now + telemetry_period_ms_;
    }
    telemetry_seq_++;   // wraps; all four frames of this cycle share it
    return true;
}

bool RoverController::recentlyDegraded() const {
    if (!ever_degraded_) return false;
    return (now_ms_() - last_degraded_ms_) < LINK_DEGRADED_HOLD_MS;
}

uint8_t RoverController::jetsonLinkState() const {
    if (watchdogTripped()) return LINK_LOST;
    if (recentlyDegraded()) return LINK_DEGRADED;
    return LINK_OK;
}

uint8_t RoverController::c2LinkState() const {
    // Never derived from the Jetson link -- only ever reported. Two different
    // reasons to say "not reported", and neither may read as LINK_OK.
    //
    // This body is what a controller-side radio replaces; see the header and
    // PROTOCOL.md 3.5.2. A locally measured link cannot go stale, so the
    // second case below collapses into the first once that lands -- but only
    // for the local reading. A forwarded bit kept as a cross-check is still
    // stale-able and still needs this rule.
    if (!have_control_)    return LINK_NOT_REPORTED;  // nobody has told us
    if (watchdogTripped()) return LINK_NOT_REPORTED;  // what we were told is stale
    return last_control_.c2_lost ? LINK_LOST : LINK_OK;
}

uint16_t RoverController::faultWord(uint16_t sensor_faults, uint16_t extra_faults) const {
    uint16_t faults = static_cast<uint16_t>(sensor_faults | extra_faults);
    if (watchdogTripped()) faults |= FAULT_COMM_TIMEOUT;
    // Without these the rover could halt or degrade while telemetry read "no
    // faults, link healthy" -- a silent stop is its own hazard.
    if (protocol_error_)   faults |= FAULT_PROTOCOL_ERROR;
    if (crc_error_)        faults |= FAULT_CRC_ERROR;
    if (seq_gap_)          faults |= FAULT_SEQ_GAP;
    // Raised in EVERY mode, even though it only stops the rover in TELEOP.
    // During an autonomous run it is the one sign that the rover is out of
    // contact, which an operator needs whether or not it changes the driving.
    // Suppressed once the value is stale: see c2LinkState().
    if (c2LinkState() == LINK_LOST) faults |= FAULT_C2_LINK_LOST;
    // The rover is being commanded by the wrong half of the stack. In the
    // fault word as well as in TELEM_CMD_ECHO, because an operator watching
    // faults is exactly who needs to know that a rover which looks idle is
    // idle on purpose.
    if (sourceMismatch()) faults |= FAULT_CMD_SOURCE;
    return faults;
}

uint8_t RoverController::indicatorState() const {
    // URC requires the light to show what the rover is ACTUALLY doing -- red
    // for autonomous, blue for teleoperation, flashing green on arrival -- and
    // the controller knows its own mode. So the Jetson's request is honoured
    // only where it agrees with that mode; otherwise the controller overrides
    // it. Checked top to bottom, first match wins. PROTOCOL.md 3.5.

    // 1. Nothing is driving this rover, or nothing can reach it. Not "off":
    //    OFF is a legitimate idle state and this is a failure, so say so.
    //    This is exactly MODE_FAULT; see activeMode().
    const uint8_t mode = activeMode();
    if (mode == MODE_FAULT) return INDICATOR_FAULT;

    // 2. Commanded not to move. A legitimate state, not a fault.
    if (mode == MODE_SAFE) return INDICATOR_OFF;

    // 3. A human-driven rover never shows red or green, whatever is requested.
    if (mode == MODE_TELEOP) return INDICATOR_TELEOP;

    // 4. Autonomous. Only the ARRIVED request matters; anything else is red.
    //    `stop` deliberately does not change the light: a paused autonomous
    //    rover is still under autonomous operation.
    //
    //    The request is honoured only from the source that owns this mode.
    //    "I have arrived" is an assertion about what the AUTONOMY did, worth
    //    25 points under rule 1.e.xvii, so a teleop-path frame claiming it
    //    while the rover is autonomous is the one mix-up that could put
    //    flashing green on a rover that has not arrived anywhere.
    if (commandAuthorized()
            && last_control_.indicator_request == INDICATOR_ARRIVED) {
        return INDICATOR_ARRIVED;
    }
    return INDICATOR_AUTONOMOUS;
}

TelemetryDriveL RoverController::buildDriveL(int32_t enc_left, int16_t steer_fb) const {
    TelemetryDriveL m;
    m.enc_left = enc_left;
    m.steer_fb = steer_fb;
    return m;
}

TelemetryDriveR RoverController::buildDriveR(int32_t enc_right) const {
    TelemetryDriveR m;
    m.enc_right  = enc_right;
    m.cmd_age_ms = cmdAgeMs();
    return m;
}

TelemetryPower RoverController::buildPower(int16_t current_ca, int16_t voltage_cv,
                                           uint16_t sensor_faults,
                                           uint16_t extra_faults) const {
    TelemetryPower m;
    m.current_ca   = current_ca;
    m.voltage_cv   = voltage_cv;
    m.fault_status = faultWord(sensor_faults, extra_faults);
    return m;
}

TelemetryState RoverController::buildState(uint8_t controller_health) const {
    TelemetryState m;
    // An ECHO of the mode the controller believes it is in, so the operator
    // can confirm the controller agrees with what was commanded. Without it a
    // dropped or misread mode change is invisible from the outside.
    m.mode              = last_control_.mode;
    m.jetson_link       = jetsonLinkState();
    // Reported, never derived from the Jetson link: they are separate links
    // and must not be treated as equivalent.
    m.c2_link           = c2LinkState();
    m.controller_health = controller_health;
    m.indicator_state   = indicatorState();
    m.reserved          = 0;
    return m;
}

TelemetryCmdEcho RoverController::buildCmdEcho(int16_t drive_applied,
                                               int16_t steer_applied) const {
    TelemetryCmdEcho m;
    m.drive_applied = drive_applied;
    m.steer_applied = steer_applied;
    // The sequence number of the last frame that fully validated, which is
    // what makes this an ACKNOWLEDGEMENT rather than a status: the Jetson can
    // match it against the seq it sent and know that this exact command got
    // through. Meaningless under NO_COMMAND, where no frame has arrived to
    // acknowledge -- see the struct comment.
    m.ack_seq = last_control_.seq;
    m.status  = packCmdStatus(commandStatus(), last_decode_, returnRequestActive());
    return m;
}

}  // namespace rover
