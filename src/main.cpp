// ============================================================
// main.cpp — HAB Payload Stabilization System
// Arm Assembly Firmware v1.0
//
// Hardware:
//   MCU:     Adafruit Feather STM32F405 Express
//   Driver:  SimpleFOC Mini v1.1 (DRV8313)
//   Motor:   iPower GBM2804H-100T (12N14P, 10Ω, 154Kv)
//   Encoder: AS5048A (integrated, SPI)
//   IMU:     Adafruit BNO085 (STEMMA QT, I2C)
//   Power:   6× Energizer L91 AA in series (9V nominal)
//
// Control architecture:
//   payload_heading = arm_yaw (BNO085) + motor_angle (AS5048A)
//   PID drives motor to minimise (payload_heading - target)
//   No slip rings. No rotating electrical connections.
//
// Author:  K6ATV
// Date:    April 2026
// ============================================================

// ── Implementation guards (single-file headers) ──────────────
#define IMU_IMPL
#define HC_IMPL
#define BATT_IMPL
#define TELEM_IMPL

// ── Includes ─────────────────────────────────────────────────
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <SimpleFOC.h>

#include "config.h"
#include "imu.h"
#include "heading_control.h"
#include "battery.h"
#include "telemetry.h"

// ── SimpleFOC Objects ────────────────────────────────────────

// AS5048A magnetic encoder on SPI2
// 14-bit resolution, full electrical angle per revolution
MagneticSensorSPI encoder(ENCODER_CS_PIN, 14, 0x3FFF);

// BLDC motor — 7 pole pairs (12N14P)
BLDCMotor motor(MOTOR_POLE_PAIRS);

// 3-PWM driver (IN1, IN2, IN3, EN)
BLDCDriver3PWM driver(DRIVER_PIN_IN1,
                      DRIVER_PIN_IN2,
                      DRIVER_PIN_IN3,
                      DRIVER_PIN_EN);

// ── Commander (runtime parameter tuning via Serial) ──────────
Commander command(Serial);

void onMotor(char* cmd) { command.motor(&motor, cmd); }
void onTarget(char* cmd) {
    float new_target = atof(cmd);
    hc_set_target(new_target);
    Serial.print("[CMD] New target heading: ");
    Serial.print(new_target, 1);
    Serial.println(" deg");
}

// ── State ────────────────────────────────────────────────────
static uint32_t _pid_last_us     = 0;
static uint32_t _start_ms        = 0;
static float    _batt_voltage    = 9.0f;
static bool     _motor_ok        = false;
static bool     _imu_ok          = false;
static bool     _low_batt_warned = false;
static bool     _motor_disabled  = false;  // Bug #5 fix: track post-cutoff state

// ── Setup ────────────────────────────────────────────────────
void setup() {
    // USB serial — wait up to 3 seconds for monitor connection
    Serial.begin(TELEM_BAUD);
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0 < 3000)) {}

    Serial.println("============================================");
    Serial.println(" HAB Payload Stabilization System v1.0");
    Serial.println(" K6ATV — April 2026");
    Serial.println("============================================");

    // ── Battery check ─────────────────────────────────────────
    batt_init();
    _batt_voltage = batt_read_voltage();
    Serial.print("[MAIN] Battery: ");
    Serial.print(_batt_voltage, 2);
    Serial.println(" V");

    if (_batt_voltage < BATT_CUTOFF_VOLTS) {
        Serial.println("[MAIN] WARNING: Battery below cutoff (bench mode).");
        // while (true) { delay(1000); }  // disabled for bench testing
    }

    // ── IMU initialisation ────────────────────────────────────
    _imu_ok = imu_init();
    if (!_imu_ok) {
        Serial.println("[MAIN] WARNING: IMU init failed. "
                       "Heading control will not run.");
    }

    // ── Encoder initialisation ────────────────────────────────
    // Use SPI2 (PB13/PB14/PB15) — hardware SPI on Feather F405
    SPI.setMOSI(PB_15);
    SPI.setMISO(PB_14);
    SPI.setSCLK(PB_13);
    encoder.init();

    Serial.print("[ENC] Initial encoder angle: ");
    Serial.print(encoder.getAngle() * RAD_TO_DEG, 1);
    Serial.println(" deg");

    // ── Motor driver initialisation ───────────────────────────
    driver.voltage_power_supply = SUPPLY_VOLTAGE;
    driver.voltage_limit        = VOLTAGE_LIMIT;
    driver.pwm_frequency        = 20000; // 20kHz — inaudible

    if (driver.init() != 1) {
        Serial.println("[MAIN] FATAL: Driver init failed.");
        while (true) { delay(1000); }
    }

    // ── Motor initialisation ──────────────────────────────────
    motor.linkSensor(&encoder);
    motor.linkDriver(&driver);

    // Torque control via voltage (no current sensing on SimpleFOC Mini)
    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::voltage;

    // Velocity limit — prevent arm from spinning too fast
    // during large corrections or startup
    motor.velocity_limit = 10.0f; // rad/s ≈ 95 RPM

    // FOC modulation
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

    // Note: motor.PID_velocity gains are not set here — the motor runs in
    // torque control mode (MotionControlType::torque), so velocity PID
    // gains have no effect.

    // Disable SimpleFOC serial monitoring (we handle our own)
    motor.useMonitoring(Serial);
    motor.monitor_variables = 0; // disable automatic monitoring

    // Initialise motor — this runs electrical angle alignment
    Serial.println("[MOTOR] Running alignment... (do not move payload)");
    motor.init();
    motor.initFOC();

    if (motor.motor_status == FOCMotorStatus::motor_ready) {
        _motor_ok = true;
        Serial.println("[MOTOR] FOC ready");
    } else {
        // Bug #1 fix: was incorrectly setting _motor_ok = true on failure
        _motor_ok = false;
        Serial.println("[MOTOR] WARNING: FOC init issue — check wiring");
        // Don't halt — may still work, but flag the fault
    }

    // ── Heading controller ────────────────────────────────────
    hc_init(TARGET_HEADING_DEG);

    // ── Telemetry ─────────────────────────────────────────────
    telem_init();

    // ── Commander — runtime tuning ────────────────────────────
    // Usage from Serial monitor:
    //   T<degrees>  — set target heading (e.g. "T180" for South)
    //   M           — motor command (SimpleFOC Commander)
    command.add('T', onTarget, "target heading deg");
    command.add('M', onMotor,  "motor");

    // Reset I2C peripheral after motor init — STM32F4 I2C1 can get stuck
    // during the long blocking FOC alignment sequence
    Wire.end();
    delay(50);
    Wire.begin();
    Wire.setClock(100000);
    imu_reset_health_timer();
    _start_ms = millis();
    Serial.println("[MAIN] System ready. Entering control loop.");
    Serial.println("[MAIN] Send 'T<degrees>' to set target heading.");
    Serial.println("[MAIN] Example: T090 = point East");
}

// ── Loop ─────────────────────────────────────────────────────
void loop() {
    // ── 1. SimpleFOC inner loop (runs as fast as possible) ───
    if (!_motor_disabled) motor.loopFOC();

    // ── 2. Drain IMU SHTP buffer every loop ───────────────────
    imu_update();

    // ── 3. Commander (check for serial commands) ──────────────
    command.run();

    // ── 3. Outer PID loop at PID_RATE_HZ ─────────────────────
    uint32_t now_us = micros();
    if (now_us - _pid_last_us >= (uint32_t)PID_LOOP_PERIOD_US) {
        _pid_last_us = now_us;

        // Update IMU health status
        _imu_ok = imu_is_healthy();

        // Check altitude for magnetometer reliability
        // (Altitude would come from tracker UART in full implementation.
        //  For now, switch to gyro-only mode after 5 hours as conservative
        //  fallback — implement UART altitude parsing in next revision.)
        uint32_t uptime_s = (millis() - _start_ms) / 1000;
        // 5400 seconds ≈ 90 minutes ≈ ~15km at 3m/s ascent
        imu_set_gyro_only_mode(uptime_s > 5400);

        // Run heading controller
        float torque_cmd = 0.0f;

        if (_motor_disabled) {
            // Battery cutoff — motor stays off, telemetry continues
        } else if (_imu_ok) {
            float arm_yaw      = imu_get_yaw_deg();
            float motor_shaft  = motor.shaft_angle; // radians, cumulative

            torque_cmd = hc_update(arm_yaw, motor_shaft);
            motor.move(torque_cmd);
        } else {
            // IMU fault — hold position (zero torque)
            motor.move(0.0f);
            #if DEBUG_LEVEL >= 1
            Serial.println("[MAIN] IMU fault — motor holding");
            #endif
        }

        // Battery monitor
        // Read every 10 PID cycles to avoid ADC overhead
        static uint8_t batt_count = 0;
        if (++batt_count >= 10) {
            batt_count = 0;
            _batt_voltage = batt_read_voltage();

            if (_batt_voltage < BATT_CUTOFF_VOLTS) {
                // Critical — stop motor and flag
                // motor.move(0.0f);           // disabled for bench testing (no battery)
                // motor.disable();
                // _motor_disabled = true;
                static bool _bench_batt_warned = false;
                if (!_bench_batt_warned) {
                    Serial.println("[BATT] CRITICAL: Below cutoff (bench mode — motor not disabled).");
                    _bench_batt_warned = true;
                }
                // Don't halt — keep telemetry running
            } else if (batt_is_low() && !_low_batt_warned) {
                _low_batt_warned = true;
                Serial.print("[BATT] WARNING: Low battery — ");
                Serial.print(_batt_voltage, 2);
                Serial.println(" V");
            }
        }

        // ── 4. Telemetry update ───────────────────────────────
        TelemetryData td;
        td.uptime_s       = uptime_s;
        td.arm_yaw_deg    = imu_get_yaw_deg();
        td.payload_hdg_deg= hc_get_payload_heading();
        td.heading_err_deg= hc_get_heading_error();
        td.motor_turns    = motor.shaft_angle / TWO_PI;
        td.torque_cmd     = torque_cmd;
        td.batt_voltage   = _batt_voltage;
        td.mag_cal        = imu_get_mag_cal_status();
        td.gyro_cal       = imu_get_gyro_cal_status();
        td.imu_ok         = _imu_ok;

        telem_update(td);
    }
}
