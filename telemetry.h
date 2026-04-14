#pragma once
// ============================================================
// telemetry.h — UART Telemetry Output
//
// Outputs a compact ASCII sentence every TELEM_INTERVAL_MS.
// Format (CSV, newline terminated):
//
// $HAB,<uptime_s>,<arm_yaw_deg>,<payload_hdg_deg>,
//      <heading_err_deg>,<motor_turns>,<torque_cmd>,
//      <batt_v>,<mag_cal>,<gyro_cal>,<imu_ok>
//
// Designed to be relayed by the tracker over LoRa.
// Also outputs verbose human-readable debug every 5 seconds
// on USB Serial when DEBUG_LEVEL >= 2.
// ============================================================
#include <Arduino.h>
#include "config.h"

struct TelemetryData {
    uint32_t uptime_s;
    float    arm_yaw_deg;
    float    payload_hdg_deg;
    float    heading_err_deg;
    float    motor_turns;
    float    torque_cmd;
    float    batt_voltage;
    uint8_t  mag_cal;
    uint8_t  gyro_cal;
    bool     imu_ok;
};

// Call once in setup()
void telem_init();

// Call every loop with current data
// Outputs sentence when TELEM_INTERVAL_MS has elapsed
void telem_update(const TelemetryData& d);

// Force immediate output (e.g. on startup or error)
void telem_send_now(const TelemetryData& d);

// ============================================================
// telemetry.cpp
// ============================================================
#ifdef TELEM_IMPL

static uint32_t _last_telem_ms  = 0;
static uint32_t _last_debug_ms  = 0;

void telem_init() {
    // USB serial is already initialised in main setup()
    // Serial3 (PB10/PB11) — only if not conflicting with IN3
    // For safety, we use USB serial (Serial) for telemetry.
    // If tracker is connected via UART, switch to Serial3 here.
    #if DEBUG_LEVEL >= 2
    Serial.println("[TELEM] Telemetry initialised");
    Serial.println("[TELEM] Format: $HAB,uptime,arm_yaw,"
                   "payload_hdg,error,motor_turns,"
                   "torque,batt_v,mag_cal,gyro_cal,imu_ok");
    #endif
}

static void _send_sentence(const TelemetryData& d) {
    // Compact CSV sentence
    Serial.print("$HAB,");
    Serial.print(d.uptime_s);         Serial.print(',');
    Serial.print(d.arm_yaw_deg,   1); Serial.print(',');
    Serial.print(d.payload_hdg_deg,1);Serial.print(',');
    Serial.print(d.heading_err_deg,1);Serial.print(',');
    Serial.print(d.motor_turns,   2); Serial.print(',');
    Serial.print(d.torque_cmd,    2); Serial.print(',');
    Serial.print(d.batt_voltage,  2); Serial.print(',');
    Serial.print(d.mag_cal);          Serial.print(',');
    Serial.print(d.gyro_cal);         Serial.print(',');
    Serial.println(d.imu_ok ? 1 : 0);
}

void telem_update(const TelemetryData& d) {
    uint32_t now = millis();

    // 30-second telemetry sentence
    if (now - _last_telem_ms >= TELEM_INTERVAL_MS) {
        _last_telem_ms = now;
        _send_sentence(d);
    }

    // 5-second human-readable debug
    #if DEBUG_LEVEL >= 2
    if (now - _last_debug_ms >= 5000) {
        _last_debug_ms = now;
        Serial.println("─────────────────────────────────");
        Serial.print("Uptime:      "); Serial.print(d.uptime_s);
        Serial.println(" s");
        Serial.print("Arm yaw:     "); Serial.print(d.arm_yaw_deg, 1);
        Serial.println(" °");
        Serial.print("Payload hdg: "); Serial.print(d.payload_hdg_deg, 1);
        Serial.println(" °");
        Serial.print("Error:       "); Serial.print(d.heading_err_deg, 1);
        Serial.println(" °");
        Serial.print("Motor turns: "); Serial.println(d.motor_turns, 2);
        Serial.print("Torque cmd:  "); Serial.print(d.torque_cmd, 2);
        Serial.println(" V");
        Serial.print("Battery:     "); Serial.print(d.batt_voltage, 2);
        Serial.println(" V");
        Serial.print("Mag cal:     "); Serial.print(d.mag_cal);
        Serial.print("/3  Gyro cal: "); Serial.print(d.gyro_cal);
        Serial.println("/3");
        Serial.print("IMU:         ");
        Serial.println(d.imu_ok ? "OK" : "FAULT");
        Serial.println("─────────────────────────────────");
    }
    #endif
}

void telem_send_now(const TelemetryData& d) {
    _send_sentence(d);
}

#endif // TELEM_IMPL
