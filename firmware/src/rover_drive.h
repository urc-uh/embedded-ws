// rover_drive.h -- the drive/steering output stage.
//
// PURE C++. No Arduino.h, no hardware, no clock. Like rover_protocol.* and
// rover_controller.*, this compiles unchanged for the Teensy and natively for
// the tests, so the stage that turns a command into a motor output is unit
// tested rather than only observable by watching a rover move.
//
// WHERE THIS SITS
//   Jetson -> CAN -> ingestFrame()        validation   (rover_controller)
//                 -> commandedOutputs()   safety + authority  (rover_controller)
//                 -> DriveModule::update() mixing + output     (here)
//                 -> pins / ESCs / a printed line
//
//   It is the LAST stage, and it is deliberately dumb about policy. It does
//   not know about modes, watchdogs, faults, stop flags or who sent the
//   command. Every one of those decisions has already been made by
//   RoverController, which hands this module a drive, a steer, and one
//   boolean saying whether the drives may be energized at all. Two layers
//   deciding when to stop is how a rover ends up with two answers.
//
// WHAT IT DOES DO
//   1. Refuses to energize. `energized == false` puts every channel at
//      neutral before anything else is considered, so no ordering mistake or
//      later edit can let a stale command through a stop.
//   2. Mixes. left = drive + steer, right = drive - steer, scaled down
//      proportionally if that saturates, so a turn keeps its RATIO instead of
//      being squared off by a clamp.
//   3. Converts to servo/ESC pulse widths, the one output language nearly all
//      small-rover drive hardware understands.
//   4. Reports its own health, which is what finally gives TELEM_STATE's
//      controller_health a real value to carry.
//
// DELIBERATELY NOT HERE: a slew-rate limiter. A ramp would make the drive
// applied differ from the drive accepted, and TELEM_CMD_ECHO's whole job is to
// tell the Jetson what was accepted. Ramping belongs in the ESC or in a motor
// controller that knows the drivetrain's inertia; inventing a rate here would
// mean inventing a tuning constant with nothing behind it.

#ifndef ROVER_DRIVE_H
#define ROVER_DRIVE_H

#include <stdint.h>

#include "rover_protocol.h"

namespace rover {

// One cycle's worth of output.
//
// Three channels, because the drivetrain is not chosen yet: a skid-steer rover
// wires left_* and right_* and ignores the steering channel, while a rover
// with a steering actuator wires steer_* and takes the drive straight through.
// Which one is a HARDWARE decision, so the module emits both rather than
// guessing, and the wiring picks. Do not wire both: the differential mix and
// the steering actuator would then each apply the same turn.
struct DriveOutputs {
    int16_t  left_effort;   // CMD_MIN..CMD_MAX, differential mix
    int16_t  right_effort;  // CMD_MIN..CMD_MAX, differential mix
    int16_t  steer_effort;  // CMD_MIN..CMD_MAX, steering actuator demand
    uint16_t left_pwm_us;   // PWM_NEUTRAL_US +/- PWM_SPAN_US
    uint16_t right_pwm_us;
    uint16_t steer_pwm_us;
    bool     energized;     // false => every channel above is at neutral
};

class DriveModule {
public:
    DriveModule();

    // One control cycle. `drive` and `steer` are the values RoverController
    // authorised (CMD_MIN..CMD_MAX); `energized` is !effectiveStop().
    //
    // Returns the outputs it just produced, which are also available later
    // from outputs(). Calling it with energized == false is the normal way to
    // hold a stop -- it is not an error and does not degrade health.
    const DriveOutputs& update(int16_t drive, int16_t steer, bool energized);

    const DriveOutputs& outputs() const { return out_; }

    // CTRL_HEALTH_* for TELEM_STATE, reported by the stage that actually
    // knows:
    //   NOT_REPORTED  update() has never been called -- nothing is known yet,
    //                 which is honestly different from "fine".
    //   FAULT         LATCHED. A command outside CMD_MIN..CMD_MAX reached this
    //                 module, which cannot happen while decodeControl() is
    //                 doing its job, so something above this layer is broken.
    //                 Not a transient and not self-clearing: it survives until
    //                 reboot, the same bargain as FAULT_FIRMWARE_FAULT.
    //   DEGRADED      the last mix saturated and had to be scaled down, so the
    //                 rover is not delivering the commanded combination of
    //                 speed and turn. Live, not latched -- a real signal for
    //                 an autonomy stack closing a loop, and a normal thing to
    //                 see at full throttle in a tight turn.
    //   OK            otherwise.
    uint8_t health() const;

    // How many times a command arrived outside CMD_MIN..CMD_MAX. Non-zero
    // means a validation bug upstream, not a rover problem.
    uint32_t rangeViolations() const { return range_violations_; }

    uint32_t updates() const { return updates_; }

private:
    void neutral();

    DriveOutputs out_;
    bool     started_;
    bool     range_fault_;   // latched
    bool     mix_saturated_; // live, this cycle only
    uint32_t range_violations_;
    uint32_t updates_;
};

// Map a CMD_MIN..CMD_MAX effort onto a servo/ESC pulse width. Exposed because
// rover_firmware.cpp and the tests both need it and neither should re-derive it.
uint16_t effortToPwmUs(int16_t effort);

}  // namespace rover

#endif  // ROVER_DRIVE_H
