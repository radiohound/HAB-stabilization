// ============================================================
// motor_test.cpp — Minimal open-loop motor driver test
//
// PURPOSE: Verify the SimpleFOC Mini driver + motor phase wiring
//          WITHOUT the encoder, IMU, PID, or FOC alignment.
//
// Open-loop velocity control spins the motor by sweeping the
// electrical angle directly — no position feedback needed.
//   • Motor spins smoothly  -> driver + phase wiring are GOOD.
//                               Problem is in encoder/FOC.
//   • Motor cogs / vibrates  -> a phase wire (IN1/IN2/IN3) or a
//                               PWM timer pin is wrong.
//   • Motor does nothing     -> check EN pin, power, and that the
//                               driver init succeeded (see serial).
//
// BUILD:   pio run -e motortest -t upload
// MONITOR: pio device monitor -e motortest
//
// SERIAL COMMANDS (type then Enter):
//   T<rad/s>   set target velocity   e.g.  T5   T-5   T0
//   V<volts>   set motor voltage     e.g.  V2   V4
//   (also full SimpleFOC Commander under 'M', e.g. MMV5)
// ============================================================

#include <Arduino.h>
#include <SimpleFOC.h>
#include "config.h"
#include "dfu_jump.h"   // 'D' command -> enter DFU without grounding B0

// 3-PWM driver — same pins as the real firmware (from config.h)
BLDCDriver3PWM driver(DRIVER_PIN_IN1,
                      DRIVER_PIN_IN2,
                      DRIVER_PIN_IN3,
                      DRIVER_PIN_EN);

// BLDC motor — pole pairs only affect commanded vs actual speed in
// open loop; the motor will still spin even if this is wrong.
BLDCMotor motor(MOTOR_POLE_PAIRS);

// Target velocity in rad/s (electrical sweep). Start gentle.
float target_velocity = 5.0f;   // ~48 RPM mechanical at 7 PP

// ── Runtime serial commands ──────────────────────────────────
Commander command(Serial);
void onMotor(char* cmd) { command.motor(&motor, cmd); }

void onTarget(char* cmd) {
    target_velocity = atof(cmd);
    Serial.print("[TEST] target velocity = ");
    Serial.print(target_velocity, 2);
    Serial.println(" rad/s");
}

void onVoltage(char* cmd) {
    float v = atof(cmd);
    motor.voltage_limit = v;
    Serial.print("[TEST] motor voltage limit = ");
    Serial.print(v, 2);
    Serial.println(" V");
}

void onDfu(char* cmd) {
    (void)cmd;
    jumpToBootloader();   // never returns — chip enters ROM DFU
}

void setup() {
    // LED proof-of-life
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0 < 3000)) {}

    delay(800);              // let USB CDC settle so the debug isn't lossy
    Serial.println("============================================");
    Serial.println(" MOTOR TEST — open-loop velocity (no encoder)");
    Serial.println(" >>> BUILD: simplefoc-diag <<<");
    Serial.println("============================================");
    Serial.flush();

    // Print SimpleFOC's internal debug (shows which TIMER/CHANNEL each
    // driver pin got assigned to — that's how we confirm phase C is
    // really on a hardware PWM channel now).
    SimpleFOCDebug::enable(&Serial);

    // ── Driver ────────────────────────────────────────────────
    driver.voltage_power_supply = SUPPLY_VOLTAGE;   // 9.0 V
    driver.voltage_limit        = VOLTAGE_LIMIT;    // 7.0 V
    driver.pwm_frequency        = 20000;            // 20 kHz, inaudible

    Serial.println("[TEST] driver.init()...");
    Serial.flush();
    int drv_ok = driver.init();
    Serial.print("[TEST] driver.init() returned: ");
    Serial.println(drv_ok);
    Serial.flush();
    if (drv_ok != 1) {
        Serial.println("[TEST] FATAL: driver init FAILED — check pins/power.");
        while (true) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(150); }
    }
    motor.linkDriver(&driver);

    // ── Motor — open loop, no sensor, no FOC alignment ────────
    // Keep the applied voltage LOW to start. Motor is 10 ohm, so
    // 2 V -> ~0.2 A per phase. Raise with 'V' once it spins.
    motor.voltage_limit  = 2.0f;          // start conservative
    motor.velocity_limit = 20.0f;         // rad/s cap
    motor.controller     = MotionControlType::velocity_openloop;

    int mot_ok = motor.init();   // NOTE: no initFOC() — open loop needs no alignment
    Serial.print("[TEST] motor.init() returned: ");
    Serial.println(mot_ok);
    Serial.print("[TEST] motor.enabled = ");
    Serial.println(motor.enabled);
    Serial.flush();

    // ── Commander ─────────────────────────────────────────────
    command.add('T', onTarget,  "target velocity rad/s");
    command.add('V', onVoltage, "motor voltage limit V");
    command.add('M', onMotor,   "motor (SimpleFOC commander)");
    command.add('D', onDfu,     "enter DFU bootloader");

    Serial.println("[TEST] Ready. Spinning at 5 rad/s, 2 V.");
    Serial.println("[TEST] Commands:  T<rad/s>   V<volts>   D (enter DFU)");
    Serial.println("[TEST]   e.g.  T10   V4   T-5   T0 (stop)");
}

void loop() {
    // SimpleFOC 2.4.0: loopFOC() is what actually applies the phase
    // voltage (setPhaseVoltage). It handles open-loop internally —
    // computes the electrical angle from the open-loop shaft_angle,
    // no sensor needed. move() only updates the target/angle.
    // (Older SimpleFOC applied voltage inside move(); 2.4.0 does not.)
    motor.loopFOC();
    motor.move(target_velocity);

    // Handle any serial commands
    command.run();

    // ── Diagnostic heartbeat ──────────────────────────────────
    // Prints the duty cycles SimpleFOC is actually writing to the
    // driver. dc_a/b/c are 0..1. If these are nonzero but the IN
    // pins measure 0V, the timer PWM output isn't running. If these
    // are ~0, the control loop isn't commanding any voltage.
    static uint32_t last = 0;
    if (millis() - last >= 1000) {
        last = millis();
        Serial.print("[DIAG] target=");
        Serial.print(target_velocity, 1);
        Serial.print(" Vlim=");
        Serial.print(motor.voltage_limit, 1);
        Serial.print(" en=");
        Serial.print(motor.enabled);
        Serial.print(" | duty a/b/c=");
        Serial.print(driver.dc_a, 3);
        Serial.print("/");
        Serial.print(driver.dc_b, 3);
        Serial.print("/");
        Serial.print(driver.dc_c, 3);
        Serial.print(" | shaft=");
        Serial.println(motor.shaft_angle, 2);
    }
}
