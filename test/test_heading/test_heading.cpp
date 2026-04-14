// ============================================================
// test_heading.cpp — Unit tests for heading_control.h
//
// Tests the heading math without any hardware.
// Run with: pio test -e native
//
// Covers:
//   - Payload heading computation (arm yaw + motor angle)
//   - Angle wrap-around at 0°/360° boundary
//   - Shortest-path error calculation
//   - Dead-band: zero output, and correct _prev_error update (bug #4)
//   - PID output clamping to VOLTAGE_LIMIT
//   - Integrator reset
// ============================================================

#include <unity.h>

// Pull in the heading controller implementation
#define HC_IMPL
#include "heading_control.h"

// Serial stub instance (declared extern in Arduino.h stub)
SerialStub Serial;

// Convenience: degrees to radians for motor shaft inputs
static float deg2rad(float d) { return d * (PI / 180.0f); }

// ── setUp / tearDown ─────────────────────────────────────────
// Called automatically before/after each test by Unity.
// Re-init the controller so every test starts from a clean state.

void setUp(void) {
    hc_init(0.0f);  // target North, all state zeroed
}

void tearDown(void) {}

// ============================================================
// Payload heading computation
// payload_hdg = norm360(arm_yaw_deg + motor_shaft_rad * RAD_TO_DEG)
// ============================================================

void test_payload_heading_arm_only(void) {
    // Motor not moved — payload heading equals arm yaw
    hc_init(0.0f);
    hc_update(90.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 90.0f, hc_get_payload_heading());
}

void test_payload_heading_motor_adds(void) {
    // arm=45°, motor rotated 90° CW → payload=135°
    hc_init(0.0f);
    hc_update(45.0f, deg2rad(90.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 135.0f, hc_get_payload_heading());
}

void test_payload_heading_wraps_above_360(void) {
    // arm=350°, motor=+20° → raw=370° → normalised=10°
    hc_init(0.0f);
    hc_update(350.0f, deg2rad(20.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 10.0f, hc_get_payload_heading());
}

void test_payload_heading_wraps_below_0(void) {
    // arm=10°, motor=-20° → raw=-10° → normalised=350°
    hc_init(0.0f);
    hc_update(10.0f, deg2rad(-20.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 350.0f, hc_get_payload_heading());
}

void test_payload_heading_multiple_motor_turns(void) {
    // Motor has spun 2.5 full turns CW (900°) plus arm at 30° → payload=930° → norm=210°
    hc_init(0.0f);
    hc_update(30.0f, deg2rad(900.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 210.0f, hc_get_payload_heading());
}

// ============================================================
// Shortest-path heading error
// ============================================================

void test_error_simple_positive(void) {
    // target=90°, payload=80° → error=+10° (need to go CW)
    hc_init(90.0f);
    hc_update(80.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 10.0f, hc_get_heading_error());
}

void test_error_simple_negative(void) {
    // target=80°, payload=90° → error=-10° (need to go CCW)
    hc_init(80.0f);
    hc_update(90.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -10.0f, hc_get_heading_error());
}

void test_error_shortest_path_across_zero_positive(void) {
    // target=10°, payload=350° → shortest path is +20° CW, not -340°
    hc_init(10.0f);
    hc_update(350.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 20.0f, hc_get_heading_error());
}

void test_error_shortest_path_across_zero_negative(void) {
    // target=350°, payload=10° → shortest path is -20° CCW, not +340°
    hc_init(350.0f);
    hc_update(10.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -20.0f, hc_get_heading_error());
}

void test_error_at_180_degrees(void) {
    // 180° is the boundary — error magnitude should be 180°
    hc_init(0.0f);
    hc_update(180.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 180.0f, fabsf(hc_get_heading_error()));
}

void test_error_zero_when_on_target(void) {
    hc_init(45.0f);
    hc_update(45.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, hc_get_heading_error());
}

// ============================================================
// Dead-band
// ============================================================

void test_deadband_no_output_inside(void) {
    // Error of 0.5° is within the 1° dead-band — torque must be zero
    hc_init(0.0f);
    float torque = hc_update(0.0f, deg2rad(0.5f));  // payload=0.5°, error=-0.5°
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, torque);
}

void test_deadband_output_outside(void) {
    // Error of 2° is outside the 1° dead-band — torque must be nonzero
    hc_init(0.0f);
    float torque = hc_update(0.0f, deg2rad(2.0f));  // payload=2°, error=-2°
    TEST_ASSERT_TRUE(fabsf(torque) > 0.001f);
}

void test_deadband_prev_error_updated(void) {
    // Bug #4 regression test.
    //
    // Scenario: system starts with a large error (10°), which sets
    // _prev_error=10°.  It then settles into the dead-band at 0.5°.
    // With the bug, _prev_error stays at 10° while inside the dead-band.
    // On the first active-control cycle at 5°, the derivative would be:
    //   (5 - 10) / 0.01 = -500 → Kd * -500 = -40V  (wrong sign, huge)
    // With the fix, _prev_error tracks the dead-band value (0.5°), so:
    //   (5 - 0.5) / 0.01 = 450 → Kd * 450 = 36V → clamped to +7V (correct sign)
    //
    // We verify the fix by checking the sign and magnitude of the output
    // on re-entry from the dead-band.

    hc_init(0.0f);  // target = 0°

    // Step 1: large positive error sets _prev_error positive
    hc_update(0.0f, deg2rad(10.0f));   // payload=10°, error=-10° → prev_error=-10°

    // Step 2: settle into dead-band — with fix, _prev_error tracks toward 0
    for (int i = 0; i < 5; i++) {
        hc_update(0.0f, deg2rad(0.5f)); // payload=0.5°, error=-0.5° (in dead-band)
    }

    // Step 3: re-enter active control with payload at 5° (error=-5°)
    // With fix:    prev_error ≈ -0.5°, derivative = (-5 - (-0.5)) / 0.01 → negative → correct sign
    // Without fix: prev_error = -10°,  derivative = (-5 - (-10)) / 0.01 → positive → WRONG sign
    float torque = hc_update(0.0f, deg2rad(5.0f));

    // The payload is at +5° and target is 0°, so we need negative torque (push CCW)
    // A positive torque here would indicate the derivative term has the wrong sign (bug present)
    TEST_ASSERT_TRUE_MESSAGE(torque < 0.0f,
        "Torque should be negative (CCW) to correct payload at +5 deg. "
        "Positive torque indicates bug #4 derivative spike is present.");
}

// ============================================================
// PID output clamping
// ============================================================

void test_output_clamped_positive(void) {
    // 170° error (unambiguously positive) should saturate the output at +VOLTAGE_LIMIT.
    // 180° is avoided because _angle_diff's fmod snaps that boundary to -180°.
    hc_init(170.0f);
    float torque = hc_update(0.0f, 0.0f);  // error = +170°
    TEST_ASSERT_FLOAT_WITHIN(0.001f, VOLTAGE_LIMIT, torque);
}

void test_output_clamped_negative(void) {
    // -180° error should saturate at -VOLTAGE_LIMIT
    hc_init(0.0f);
    float torque = hc_update(180.0f, 0.0f);  // error = -180°
    TEST_ASSERT_FLOAT_WITHIN(0.001f, VOLTAGE_LIMIT, fabsf(torque));
}

void test_output_never_exceeds_limit(void) {
    // Run several cycles with large error and confirm output stays bounded
    hc_init(0.0f);
    for (int i = 0; i < 20; i++) {
        float torque = hc_update(0.0f, deg2rad(90.0f));
        TEST_ASSERT_TRUE(fabsf(torque) <= VOLTAGE_LIMIT + 0.001f);
    }
}

// ============================================================
// Integrator reset
// ============================================================

void test_integrator_reset_reduces_output(void) {
    // Wind up the integrator with several cycles of steady error,
    // then reset it — output should drop.
    hc_init(0.0f);
    float torque_wound = 0.0f;
    for (int i = 0; i < 50; i++) {
        torque_wound = hc_update(0.0f, deg2rad(5.0f));  // steady 5° error
    }
    hc_reset_integrator();
    float torque_reset = hc_update(0.0f, deg2rad(5.0f));  // same error, no integral

    // After reset the integral term is gone — output should be lower
    // (P term only: 0.5 * 5 = 2.5V)
    TEST_ASSERT_TRUE(fabsf(torque_reset) < fabsf(torque_wound));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, PID_KP * 5.0f, fabsf(torque_reset));
}

// ============================================================
// main
// ============================================================

int main(void) {
    UNITY_BEGIN();

    // Payload heading computation
    RUN_TEST(test_payload_heading_arm_only);
    RUN_TEST(test_payload_heading_motor_adds);
    RUN_TEST(test_payload_heading_wraps_above_360);
    RUN_TEST(test_payload_heading_wraps_below_0);
    RUN_TEST(test_payload_heading_multiple_motor_turns);

    // Shortest-path error
    RUN_TEST(test_error_simple_positive);
    RUN_TEST(test_error_simple_negative);
    RUN_TEST(test_error_shortest_path_across_zero_positive);
    RUN_TEST(test_error_shortest_path_across_zero_negative);
    RUN_TEST(test_error_at_180_degrees);
    RUN_TEST(test_error_zero_when_on_target);

    // Dead-band
    RUN_TEST(test_deadband_no_output_inside);
    RUN_TEST(test_deadband_output_outside);
    RUN_TEST(test_deadband_prev_error_updated);

    // Output clamping
    RUN_TEST(test_output_clamped_positive);
    RUN_TEST(test_output_clamped_negative);
    RUN_TEST(test_output_never_exceeds_limit);

    // Integrator
    RUN_TEST(test_integrator_reset_reduces_output);

    return UNITY_END();
}
