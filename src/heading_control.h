#pragma once
// ============================================================
// heading_control.h — Payload Heading Controller
//
// Computes payload absolute heading from:
//   arm_heading  (BNO085 absolute yaw)
//   motor_angle  (AS5048A cumulative shaft angle)
//
// payload_heading = arm_heading + motor_angle
//
// Runs a PID loop to drive heading error to zero.
// ============================================================
#include <Arduino.h>
#include <SimpleFOC.h>
#include "config.h"

// ── Public API ───────────────────────────────────────────────

// Initialise — pass target heading in degrees (0=North, CW positive)
void hc_init(float target_heading_deg);

// Set a new target heading at runtime (degrees, 0-360)
void hc_set_target(float heading_deg);

// Get current target
float hc_get_target();

// Main update — call at PID_RATE_HZ
// arm_yaw_deg:    from imu_get_yaw_deg()
// motor_shaft_rad: from motor.shaft_angle (SimpleFOC, cumulative)
// Returns torque command to pass to motor.move()
float hc_update(float arm_yaw_deg, float motor_shaft_rad);

// Get last computed payload heading (degrees, 0-360)
float hc_get_payload_heading();

// Get last heading error (degrees, signed, -180 to +180)
float hc_get_heading_error();

// NOTE: Motor turns not tracked here — use motor.shaft_angle / TWO_PI directly

// Reset integrator (call if large disturbance occurs)
void hc_reset_integrator();

// ============================================================
// heading_control.cpp
// ============================================================
#ifdef HC_IMPL

static float _target_deg    = TARGET_HEADING_DEG;
static float _payload_hdg   = 0.0f;
static float _heading_error = 0.0f;

// PID state
static float _integral      = 0.0f;
static float _prev_error    = 0.0f;
static float _dt            = 1.0f / PID_RATE_HZ;

// Last torque command
static float _torque_cmd    = 0.0f;

// ── Helpers ──────────────────────────────────────────────────

// Normalise angle to 0–360
static float _norm360(float a) {
    a = fmodf(a, 360.0f);
    if (a < 0.0f) a += 360.0f;
    return a;
}

// Shortest angular difference, result in -180 to +180
static float _angle_diff(float from_deg, float to_deg) {
    float d = fmodf(to_deg - from_deg + 540.0f, 360.0f) - 180.0f;
    return d;
}

// Clamp value to ±limit
static float _clamp(float v, float limit) {
    if (v >  limit) return  limit;
    if (v < -limit) return -limit;
    return v;
}

// ── Public functions ─────────────────────────────────────────

void hc_init(float target_heading_deg) {
    _target_deg   = _norm360(target_heading_deg);
    _integral     = 0.0f;
    _prev_error   = 0.0f;
    _torque_cmd   = 0.0f;
    _payload_hdg  = 0.0f;
    _heading_error= 0.0f;

    #if DEBUG_LEVEL >= 2
    Serial.print("[HC] Target heading: ");
    Serial.print(_target_deg, 1);
    Serial.println(" deg");
    #endif
}

void hc_set_target(float heading_deg) {
    _target_deg = _norm360(heading_deg);
    // Don't reset integrator — smooth transition
}

float hc_get_target() { return _target_deg; }

float hc_update(float arm_yaw_deg, float motor_shaft_rad) {
    // ── 1. Compute payload absolute heading ──────────────────
    //
    // motor.shaft_angle is cumulative in radians (no wrap).
    // Convert to degrees — this is how much the payload has
    // rotated relative to the arm since power-on.
    //
    // NOTE: Sign convention — if motor.shaft_angle positive
    // means payload rotated CW relative to arm (when viewed
    // from above), then this formula is correct.
    // Verify on bench: manually rotate payload CW, check
    // payload_hdg increases. If not, negate motor_angle_deg.
    //
    float motor_angle_deg = motor_shaft_rad * (180.0f / PI);
    _payload_hdg = _norm360(arm_yaw_deg + motor_angle_deg);

    // ── 2. Heading error ─────────────────────────────────────
    _heading_error = _angle_diff(_payload_hdg, _target_deg);

    // ── 3. Dead-band ─────────────────────────────────────────
    if (fabsf(_heading_error) < HEADING_DEADBAND_DEG) {
        // Within dead-band — let integrator bleed slowly
        _integral *= 0.99f;
        // Bug #4 fix: keep _prev_error current so derivative doesn't spike
        // when the system re-enters active control after drifting out.
        _prev_error = _heading_error;
        return 0.0f;
    }

    // ── 4. PID ───────────────────────────────────────────────
    float p_term = PID_KP * _heading_error;

    _integral += PID_KI * _heading_error * _dt;
    // Anti-windup — clamp integrator
    _integral = _clamp(_integral, PID_OUTPUT_LIMIT * 0.5f);

    float derivative = (_heading_error - _prev_error) / _dt;
    float d_term = PID_KD * derivative;

    _prev_error = _heading_error;

    _torque_cmd = p_term + _integral + d_term;
    _torque_cmd = _clamp(_torque_cmd, PID_OUTPUT_LIMIT);

    return _torque_cmd;
}

float hc_get_payload_heading() { return _payload_hdg; }
float hc_get_heading_error()   { return _heading_error; }

// NOTE: Motor turns are not tracked inside heading_control.
// Read directly from SimpleFOC: motor.shaft_angle / TWO_PI

void hc_reset_integrator() {
    _integral   = 0.0f;
    _prev_error = _heading_error;  // preserve current error so derivative doesn't spike on re-entry
    #if DEBUG_LEVEL >= 2
    Serial.println("[HC] Integrator reset");
    #endif
}

#endif // HC_IMPL
