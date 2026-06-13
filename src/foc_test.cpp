// ============================================================
// foc_test.cpp — CLOSED-LOOP FOC test (motor + encoder)
//
// Validates the full field-oriented control path the real
// firmware uses: BLDCMotor + AS5048A encoder + torque/voltage
// control. No IMU, no PID heading loop yet — just FOC.
//
// initFOC() ALIGNS the motor: it energizes the coils and watches
// the encoder to learn pole pairs, sensor direction, and the
// zero electrical angle. THE MOTOR WILL MOVE during this — keep
// the shaft FREE (nothing attached).
//
//   • "initFOC: SUCCESS" + smooth response -> FOC works! On to
//     the real stabilization firmware.
//   • "PP check failed" / rough running    -> pole pairs or
//     encoder direction wrong (debug prints the estimate).
//
// BUILD:   pio run -e foctest -t upload
// MONITOR: pio device monitor -e foctest
//
// SERIAL:  T<volts>  torque command (e.g. T1, T0 stop, T-1 rev)
//          M...      SimpleFOC Commander,  D = enter DFU
// ============================================================

#include <Arduino.h>
#include <SimpleFOC.h>
#include "config.h"
#include "dfu_jump.h"

// Encoder (AS5048A, PWM mode) — PWM out on pin 11
MagneticSensorPWM encoder(ENCODER_PWM_PIN, 2, 903);
void doPWM() { encoder.handlePWM(); }

// Motor + driver (as-built pins from config.h)
BLDCMotor motor(MOTOR_POLE_PAIRS);
BLDCDriver3PWM driver(DRIVER_PIN_IN1, DRIVER_PIN_IN2,
                      DRIVER_PIN_IN3, DRIVER_PIN_EN);

// Torque (voltage) command
float target = 0.0f;

Commander command(Serial);
void onTarget(char* cmd) {
    target = atof(cmd);
    Serial.print("[FOC] torque target = ");
    Serial.print(target, 2);
    Serial.println(" V");
}
void onMotor(char* cmd) { command.motor(&motor, cmd); }
void onDfu(char* cmd)   { (void)cmd; jumpToBootloader(); }

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0 < 3000)) {}
    delay(500);

    Serial.println("====================================================");
    Serial.println(" CLOSED-LOOP FOC TEST — motor + encoder");
    Serial.println("====================================================");
    SimpleFOCDebug::enable(&Serial);

    // ── Encoder ───────────────────────────────────────────────
    encoder.init();
    encoder.enableInterrupt(doPWM);
    Serial.print("[FOC] encoder initial angle = ");
    Serial.print(encoder.getAngle() * RAD_TO_DEG, 1);
    Serial.println(" deg");

    // ── Driver ────────────────────────────────────────────────
    driver.voltage_power_supply = SUPPLY_VOLTAGE;
    driver.voltage_limit        = VOLTAGE_LIMIT;
    driver.pwm_frequency        = 20000;
    if (driver.init() != 1) {
        Serial.println("[FOC] FATAL: driver init failed.");
        while (true) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(150); }
    }

    // ── Motor — closed-loop torque (voltage), same as main.cpp ─
    motor.linkSensor(&encoder);
    motor.linkDriver(&driver);
    motor.controller        = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::voltage;
    motor.foc_modulation    = FOCModulationType::SpaceVectorPWM;
    motor.voltage_limit     = VOLTAGE_LIMIT;
    motor.velocity_limit    = 20.0f;
    motor.voltage_sensor_align = 3.0f;   // modest voltage during alignment

    motor.init();

    Serial.println("[FOC] initFOC() — ALIGNING, keep shaft free...");
    int ok = motor.initFOC();

    Serial.print("[FOC] initFOC result: ");
    Serial.println(ok == 1 ? "SUCCESS" : "FAILED");
    Serial.print("[FOC] motor_status = ");
    Serial.println((int)motor.motor_status);
    Serial.print("[FOC] zero_electric_angle = ");
    Serial.println(motor.zero_electric_angle, 4);
    Serial.print("[FOC] sensor_direction = ");
    Serial.println(motor.sensor_direction == Direction::CW ? "CW" : "CCW");
    Serial.print("[FOC] pole_pairs (configured) = ");
    Serial.println(motor.pole_pairs);

    command.add('T', onTarget, "torque (V)");
    command.add('M', onMotor,  "motor");
    command.add('D', onDfu,    "DFU");

    Serial.println("[FOC] Ready. Send T<volts>:  T1  T0(stop)  T-1(rev)");
    Serial.println("[FOC] A small T (e.g. T1) should spin it SMOOTHLY.");
}

void loop() {
    motor.loopFOC();      // FOC commutation using encoder
    motor.move(target);   // apply torque command
    command.run();

    static uint32_t last = 0;
    if (millis() - last >= 500) {
        last = millis();
        Serial.print("[FOC] target=");
        Serial.print(target, 2);
        Serial.print("V  angle=");
        Serial.print(motor.shaft_angle * RAD_TO_DEG, 1);
        Serial.print("deg  vel=");
        Serial.print(motor.shaft_velocity, 2);
        Serial.println(" rad/s");
    }
}
