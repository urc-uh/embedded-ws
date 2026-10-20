#include "rover_drive.h"

namespace rover {

// CMD_MAX is the divisor in both the mix scaling and the PWM conversion.
static_assert(CMD_MAX > 0, "CMD_MAX must be positive; it divides below");
static_assert(CMD_MIN == -CMD_MAX, "the command range is assumed symmetric");

namespace {

int16_t clampEffort(int32_t value) {
    if (value > CMD_MAX) return CMD_MAX;
    if (value < CMD_MIN) return CMD_MIN;
    return static_cast<int16_t>(value);
}

}  // namespace

uint16_t effortToPwmUs(int16_t effort) {
    // int32 throughout: effort * PWM_SPAN_US reaches 500000, which overflows
    // int16 long before the division brings it back.
    const int32_t clamped = clampEffort(effort);
    const int32_t offset =
        (clamped * static_cast<int32_t>(PWM_SPAN_US)) / static_cast<int32_t>(CMD_MAX);
    return static_cast<uint16_t>(static_cast<int32_t>(PWM_NEUTRAL_US) + offset);
}

DriveModule::DriveModule()
    : started_(false),
      range_fault_(false),
      mix_saturated_(false),
      range_violations_(0),
      updates_(0) {
    neutral();
}

void DriveModule::neutral() {
    out_.left_effort   = 0;
    out_.right_effort  = 0;
    out_.steer_effort  = 0;
    out_.left_pwm_us   = PWM_NEUTRAL_US;
    out_.right_pwm_us  = PWM_NEUTRAL_US;
    out_.steer_pwm_us  = PWM_NEUTRAL_US;
    out_.energized     = false;
}

const DriveOutputs& DriveModule::update(int16_t drive, int16_t steer, bool energized) {
    started_ = true;
    updates_++;
    mix_saturated_ = false;

    // A command outside the valid range should have been rejected by
    // decodeControl() long before reaching this module. If it got here, the
    // validation layer above is broken -- so clamp (never act on a value we do
    // not understand) and LATCH it, because a silent clamp is exactly how a
    // misunderstood command becomes full throttle.
    if (!isValidCommand(drive) || !isValidCommand(steer)) {
        range_fault_ = true;
        range_violations_++;
    }
    const int32_t d = clampEffort(drive);
    const int32_t s = clampEffort(steer);

    // The stop path, and it comes first so that no amount of later editing can
    // slip a command past it. The caller has already zeroed the commands under
    // a stop; this honours the flag anyway, because two independent reasons to
    // be at neutral is the point.
    if (!energized) {
        neutral();
        return out_;
    }

    // Differential mix. Scaled down proportionally rather than clamped per
    // channel: clamping left to CMD_MAX while right stays where it is changes
    // the RATIO between the two, which is the turn. Scaling keeps the turn and
    // gives up only speed, which is the right thing to lose.
    int32_t left  = d + s;
    int32_t right = d - s;
    int32_t peak  = left >= 0 ? left : -left;
    const int32_t right_mag = right >= 0 ? right : -right;
    if (right_mag > peak) peak = right_mag;
    if (peak > CMD_MAX) {
        // Integer division truncates toward zero, so the result is at or just
        // inside full scale -- never over it.
        left  = (left  * CMD_MAX) / peak;
        right = (right * CMD_MAX) / peak;
        mix_saturated_ = true;
    }

    out_.left_effort  = clampEffort(left);
    out_.right_effort = clampEffort(right);
    out_.steer_effort = static_cast<int16_t>(s);
    out_.left_pwm_us  = effortToPwmUs(out_.left_effort);
    out_.right_pwm_us = effortToPwmUs(out_.right_effort);
    out_.steer_pwm_us = effortToPwmUs(out_.steer_effort);
    out_.energized    = true;
    return out_;
}

uint8_t DriveModule::health() const {
    if (!started_)      return CTRL_HEALTH_NOT_REPORTED;
    if (range_fault_)   return CTRL_HEALTH_FAULT;
    if (mix_saturated_) return CTRL_HEALTH_DEGRADED;
    return CTRL_HEALTH_OK;
}

}  // namespace rover
