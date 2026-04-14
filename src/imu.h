#pragma once
// ============================================================
// imu.h — BNO085 Interface
// Provides arm absolute heading and tilt data.
// ============================================================
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "config.h"

// ── Public API ───────────────────────────────────────────────

// Call once in setup()
bool imu_init();

// Call every loop — updates internal state from BNO085
// Returns true if fresh data was available
bool imu_update();

// Arm absolute yaw in degrees, 0–360, CW positive (magnetic North = 0)
// Uses Rotation Vector (magnetometer-fused) when reliable,
// falls back to Game Rotation Vector (gyro-only) above MAG_CUTOFF_ALT_M
float imu_get_yaw_deg();

// Raw gyro Z-axis rate in degrees/second (positive = CW from above)
float imu_get_yaw_rate_deg_s();

// Calibration status 0-3 (3 = fully calibrated)
uint8_t imu_get_mag_cal_status();
uint8_t imu_get_gyro_cal_status();

// Switch to gyro-only heading mode (call when above 15km)
void imu_set_gyro_only_mode(bool enable);

// True if IMU is healthy and returning data
bool imu_is_healthy();

// ============================================================
// imu.cpp
// ============================================================
#ifdef IMU_IMPL

static Adafruit_BNO08x  _bno;
static sh2_SensorValue_t _sensor_value;

static float   _yaw_deg        = 0.0f;
static float   _yaw_rate_deg_s = 0.0f;
static uint8_t _mag_cal        = 0;
static uint8_t _gyro_cal       = 0;
static bool    _gyro_only_mode = false;
static bool    _healthy        = false;
static uint32_t _last_update_ms = 0;

// Convert quaternion to Euler yaw (degrees, 0-360 CW)
static float _quat_to_yaw(float qr, float qi, float qj, float qk) {
    // Standard quaternion -> Euler ZYX
    float sinYaw = 2.0f * (qr * qk + qi * qj);
    float cosYaw = 1.0f - 2.0f * (qj * qj + qk * qk);
    float yaw_rad = atan2f(sinYaw, cosYaw);
    float yaw_deg = yaw_rad * 180.0f / PI;
    // Normalise to 0-360
    if (yaw_deg < 0.0f) yaw_deg += 360.0f;
    return yaw_deg;
}

static void _enable_reports() {
    if (_gyro_only_mode) {
        // Game Rotation Vector — gyro-dominant, no magnetometer
        _bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000); // 100Hz
        #if DEBUG_LEVEL >= 2
        Serial.println("[IMU] Mode: Game Rotation Vector (gyro-only)");
        #endif
    } else {
        // Rotation Vector — full fusion including magnetometer
        _bno.enableReport(SH2_ROTATION_VECTOR, 10000);       // 100Hz
        #if DEBUG_LEVEL >= 2
        Serial.println("[IMU] Mode: Rotation Vector (fused with mag)");
        #endif
    }
    // Always enable gyroscope for rate data
    _bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
}

bool imu_init() {
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C

    if (!_bno.begin_I2C(BNO085_ADDR, &Wire)) {
        #if DEBUG_LEVEL >= 1
        Serial.println("[IMU] ERROR: BNO085 not found. Check wiring.");
        #endif
        return false;
    }

    _enable_reports();
    _healthy = true;
    _last_update_ms = millis();  // Bug #2 fix: was 0, causing immediate false-unhealthy at boot

    #if DEBUG_LEVEL >= 2
    Serial.println("[IMU] BNO085 initialised OK");
    #endif
    return true;
}

bool imu_update() {
    if (!_healthy) return false;

    bool got_data = false;

    if (_bno.getSensorEvent(&_sensor_value)) {
        switch (_sensor_value.sensorId) {

            case SH2_ROTATION_VECTOR:
            case SH2_GAME_ROTATION_VECTOR: {
                float qr, qi, qj, qk;
                if (_sensor_value.sensorId == SH2_GAME_ROTATION_VECTOR) {
                    // Bug #3 fix: was reading un.rotationVector for both types.
                    // Game Rotation Vector uses a different union member.
                    qr = _sensor_value.un.gameRotationVector.real;
                    qi = _sensor_value.un.gameRotationVector.i;
                    qj = _sensor_value.un.gameRotationVector.j;
                    qk = _sensor_value.un.gameRotationVector.k;
                } else {
                    qr = _sensor_value.un.rotationVector.real;
                    qi = _sensor_value.un.rotationVector.i;
                    qj = _sensor_value.un.rotationVector.j;
                    qk = _sensor_value.un.rotationVector.k;
                }
                _yaw_deg = _quat_to_yaw(qr, qi, qj, qk);

                // Calibration status only available on full rotation vector
                if (_sensor_value.sensorId == SH2_ROTATION_VECTOR) {
                    uint8_t status = _sensor_value.status & 0x03;
                    _mag_cal  = status;
                }
                got_data = true;
                _last_update_ms = millis();
                break;
            }

            case SH2_GYROSCOPE_CALIBRATED: {
                // Z-axis angular velocity (rad/s → deg/s)
                // BNO085 Z-axis points up when flat;
                // positive = CCW from above in sensor frame.
                // We negate to get CW positive convention.
                _yaw_rate_deg_s = -(_sensor_value.un.gyroscope.z
                                    * 180.0f / PI);
                _gyro_cal = _sensor_value.status & 0x03;
                break;
            }

            default:
                break;
        }
    }

    // Health check — if no data for 2 seconds, flag as unhealthy
    if (millis() - _last_update_ms > 2000) {
        _healthy = false;
        #if DEBUG_LEVEL >= 1
        Serial.println("[IMU] WARNING: No data for 2s — IMU unhealthy");
        #endif
    }

    return got_data;
}

float imu_get_yaw_deg()          { return _yaw_deg; }
float imu_get_yaw_rate_deg_s()   { return _yaw_rate_deg_s; }
uint8_t imu_get_mag_cal_status() { return _mag_cal; }
uint8_t imu_get_gyro_cal_status(){ return _gyro_cal; }
bool imu_is_healthy()            { return _healthy; }

void imu_set_gyro_only_mode(bool enable) {
    if (enable == _gyro_only_mode) return;
    _gyro_only_mode = enable;
    // Re-enable reports in new mode
    _enable_reports();
    // Reset health state so fresh data in new mode isn't penalised
    // by stale timestamp from the old mode
    _last_update_ms = millis();
    _healthy = true;
}

#endif // IMU_IMPL
