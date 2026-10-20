// test_drive.cpp -- the drive/steering output stage.
//
// This is the last layer before a pin, so the tests here are mostly about what
// it must NEVER do: energize while told not to, let an out-of-range value
// through silently, or square off a turn by clamping one side of a mix.

#include "../../firmware/src/rover_drive.h"
#include "test_framework.h"

using namespace rover;

// --- the stop path ----------------------------------------------------------

static void boots_at_neutral_and_reports_nothing_known() {
    DriveModule drv;
    CHECK(!drv.outputs().energized);
    CHECK_EQ(drv.outputs().left_effort, 0);
    CHECK_EQ(drv.outputs().right_effort, 0);
    CHECK_EQ(drv.outputs().steer_effort, 0);
    CHECK_EQ(drv.outputs().left_pwm_us, PWM_NEUTRAL_US);
    CHECK_EQ(drv.outputs().right_pwm_us, PWM_NEUTRAL_US);
    CHECK_EQ(drv.outputs().steer_pwm_us, PWM_NEUTRAL_US);
    // Never updated, so "we do not know" rather than a misleading OK -- the
    // same bargain c2LinkState() makes.
    CHECK_EQ(drv.health(), CTRL_HEALTH_NOT_REPORTED);
}

static void a_de_energized_update_puts_every_channel_at_neutral() {
    DriveModule drv;
    // Full throttle AND de-energized. The caller already zeroes commands under
    // a stop; this is the second, independent reason to be at neutral.
    const DriveOutputs& o = drv.update(1000, 1000, /*energized=*/false);
    CHECK(!o.energized);
    CHECK_EQ(o.left_effort, 0);
    CHECK_EQ(o.right_effort, 0);
    CHECK_EQ(o.steer_effort, 0);
    CHECK_EQ(o.left_pwm_us, PWM_NEUTRAL_US);
    CHECK_EQ(o.right_pwm_us, PWM_NEUTRAL_US);
    CHECK_EQ(o.steer_pwm_us, PWM_NEUTRAL_US);
    CHECK_EQ(drv.health(), CTRL_HEALTH_OK);   // holding a stop is not a fault
}

static void de_energizing_takes_effect_in_the_same_cycle() {
    // No ramp, no hysteresis, no carry-over from the previous cycle: a stop
    // that takes a cycle to land is a stop that can be missed.
    DriveModule drv;
    drv.update(1000, 0, true);
    CHECK_EQ(drv.outputs().left_effort, 1000);
    drv.update(1000, 0, false);
    CHECK_EQ(drv.outputs().left_effort, 0);
    CHECK_EQ(drv.outputs().left_pwm_us, PWM_NEUTRAL_US);
}

// --- mixing -----------------------------------------------------------------

static void straight_ahead_drives_both_sides_equally() {
    DriveModule drv;
    const DriveOutputs& o = drv.update(600, 0, true);
    CHECK_EQ(o.left_effort, 600);
    CHECK_EQ(o.right_effort, 600);
    CHECK_EQ(o.steer_effort, 0);
    CHECK(o.energized);
    CHECK_EQ(drv.health(), CTRL_HEALTH_OK);
}

static void a_turn_in_place_drives_the_sides_opposite() {
    DriveModule drv;
    const DriveOutputs& o = drv.update(0, 400, true);
    CHECK_EQ(o.left_effort, 400);
    CHECK_EQ(o.right_effort, -400);
    CHECK_EQ(o.steer_effort, 400);
}

static void a_saturating_mix_is_scaled_not_clamped() {
    // drive 1000 + steer 1000 would be 2000 on the left and 0 on the right.
    // Clamping gives (1000, 0) -- a hard turn. Scaling gives (1000, 0) too
    // here, but the RATIO is what matters, so check a case where the two
    // answers differ.
    DriveModule drv;
    const DriveOutputs& o = drv.update(800, 400, true);   // 1200 / 400
    // Clamping would give (1000, 400), a ratio of 2.5. Scaling gives
    // (1000, 333), the commanded ratio of 3 preserved.
    CHECK_EQ(o.left_effort, 1000);
    CHECK_EQ(o.right_effort, 333);
    CHECK_EQ(drv.health(), CTRL_HEALTH_DEGRADED);
}

static void saturation_is_live_not_latched() {
    // A tight turn at full throttle is normal, so DEGRADED must clear as soon
    // as the command stops asking for the impossible.
    DriveModule drv;
    drv.update(800, 400, true);
    CHECK_EQ(drv.health(), CTRL_HEALTH_DEGRADED);
    drv.update(400, 200, true);
    CHECK_EQ(drv.health(), CTRL_HEALTH_OK);
}

static void a_scaled_mix_never_exceeds_full_scale() {
    // Exhaustive over the whole legal command square, in steps. Integer
    // division truncates toward zero, so the result must land at or inside
    // full scale -- never one count over it.
    DriveModule drv;
    for (int d = CMD_MIN; d <= CMD_MAX; d += 25) {
        for (int s = CMD_MIN; s <= CMD_MAX; s += 25) {
            const DriveOutputs& o = drv.update(static_cast<int16_t>(d),
                                               static_cast<int16_t>(s), true);
            CHECK(o.left_effort >= CMD_MIN && o.left_effort <= CMD_MAX);
            CHECK(o.right_effort >= CMD_MIN && o.right_effort <= CMD_MAX);
            CHECK(o.left_pwm_us >= PWM_NEUTRAL_US - PWM_SPAN_US);
            CHECK(o.left_pwm_us <= PWM_NEUTRAL_US + PWM_SPAN_US);
            CHECK(o.right_pwm_us >= PWM_NEUTRAL_US - PWM_SPAN_US);
            CHECK(o.right_pwm_us <= PWM_NEUTRAL_US + PWM_SPAN_US);
        }
    }
    CHECK_EQ(drv.rangeViolations(), 0u);
}

static void the_turn_direction_is_never_inverted_by_scaling() {
    // The bug this catches: a scale that flips a sign turns a left into a
    // right at exactly the moment the rover is working hardest.
    DriveModule drv;
    for (int d = CMD_MIN; d <= CMD_MAX; d += 50) {
        for (int s = 50; s <= CMD_MAX; s += 50) {
            const DriveOutputs& o = drv.update(static_cast<int16_t>(d),
                                               static_cast<int16_t>(s), true);
            CHECK(o.left_effort >= o.right_effort);
        }
    }
}

// --- PWM --------------------------------------------------------------------

static void pwm_maps_the_command_range_onto_the_pulse_range() {
    CHECK_EQ(effortToPwmUs(0), PWM_NEUTRAL_US);
    CHECK_EQ(effortToPwmUs(CMD_MAX), PWM_NEUTRAL_US + PWM_SPAN_US);
    CHECK_EQ(effortToPwmUs(CMD_MIN), PWM_NEUTRAL_US - PWM_SPAN_US);
    CHECK_EQ(effortToPwmUs(500), PWM_NEUTRAL_US + PWM_SPAN_US / 2);
    CHECK_EQ(effortToPwmUs(-500), PWM_NEUTRAL_US - PWM_SPAN_US / 2);
}

static void pwm_clamps_rather_than_wrapping_past_full_scale() {
    // effort * PWM_SPAN_US reaches 500000, which overflows int16 long before
    // the division brings it back -- so this is also the int-width guard.
    CHECK_EQ(effortToPwmUs(32767), PWM_NEUTRAL_US + PWM_SPAN_US);
    CHECK_EQ(effortToPwmUs(-32768), PWM_NEUTRAL_US - PWM_SPAN_US);
}

static void pwm_is_monotonic_across_the_whole_command_range() {
    uint16_t previous = effortToPwmUs(CMD_MIN);
    for (int v = CMD_MIN + 1; v <= CMD_MAX; ++v) {
        const uint16_t now = effortToPwmUs(static_cast<int16_t>(v));
        CHECK(now >= previous);
        previous = now;
    }
    CHECK_EQ(previous, PWM_NEUTRAL_US + PWM_SPAN_US);
}

// --- the upstream-bug detector ---------------------------------------------

static void an_out_of_range_command_is_clamped_and_latched_as_a_fault() {
    // decodeControl() rejects these, so one reaching this module means the
    // validation layer above is broken. Clamp (never act on a value we do not
    // understand) and say so loudly rather than quietly obeying.
    DriveModule drv;
    const DriveOutputs& o = drv.update(5000, 0, true);
    CHECK_EQ(o.left_effort, CMD_MAX);
    CHECK_EQ(o.right_effort, CMD_MAX);
    CHECK_EQ(drv.rangeViolations(), 1u);
    CHECK_EQ(drv.health(), CTRL_HEALTH_FAULT);

    // Latched: a validation bug upstream is not a transient, so a clean cycle
    // afterwards must not paper over it.
    drv.update(100, 0, true);
    CHECK_EQ(drv.health(), CTRL_HEALTH_FAULT);
    CHECK_EQ(drv.rangeViolations(), 1u);
}

static void an_out_of_range_steer_is_caught_too() {
    DriveModule drv;
    drv.update(0, -5000, true);
    CHECK_EQ(drv.outputs().steer_effort, CMD_MIN);
    CHECK_EQ(drv.health(), CTRL_HEALTH_FAULT);
}

static void an_out_of_range_command_while_stopped_is_still_caught() {
    // The stop path returns early, so the range check has to come first or a
    // stopped rover would hide the upstream bug until it next moved.
    DriveModule drv;
    drv.update(9000, 0, /*energized=*/false);
    CHECK_EQ(drv.outputs().left_effort, 0);
    CHECK_EQ(drv.health(), CTRL_HEALTH_FAULT);
}

static void the_boundary_values_are_not_violations() {
    DriveModule drv;
    drv.update(CMD_MAX, 0, true);
    drv.update(CMD_MIN, 0, true);
    drv.update(0, CMD_MAX, true);
    drv.update(0, CMD_MIN, true);
    CHECK_EQ(drv.rangeViolations(), 0u);
    CHECK(drv.health() != CTRL_HEALTH_FAULT);
}

static void update_counting_survives_a_long_run() {
    DriveModule drv;
    for (int i = 0; i < 1000; ++i) drv.update(100, 50, true);
    CHECK_EQ(drv.updates(), 1000u);
}

int main() {
    RUN_TEST(boots_at_neutral_and_reports_nothing_known);
    RUN_TEST(a_de_energized_update_puts_every_channel_at_neutral);
    RUN_TEST(de_energizing_takes_effect_in_the_same_cycle);
    RUN_TEST(straight_ahead_drives_both_sides_equally);
    RUN_TEST(a_turn_in_place_drives_the_sides_opposite);
    RUN_TEST(a_saturating_mix_is_scaled_not_clamped);
    RUN_TEST(saturation_is_live_not_latched);
    RUN_TEST(a_scaled_mix_never_exceeds_full_scale);
    RUN_TEST(the_turn_direction_is_never_inverted_by_scaling);
    RUN_TEST(pwm_maps_the_command_range_onto_the_pulse_range);
    RUN_TEST(pwm_clamps_rather_than_wrapping_past_full_scale);
    RUN_TEST(pwm_is_monotonic_across_the_whole_command_range);
    RUN_TEST(an_out_of_range_command_is_clamped_and_latched_as_a_fault);
    RUN_TEST(an_out_of_range_steer_is_caught_too);
    RUN_TEST(an_out_of_range_command_while_stopped_is_still_caught);
    RUN_TEST(the_boundary_values_are_not_violations);
    RUN_TEST(update_counting_survives_a_long_run);
    return testing::summary("test_drive");
}
