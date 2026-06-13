// ============================================================
// encoder_test.cpp — AS5048A encoder readout, motor OFF
//
// Purpose: verify the magnetic encoder reads a sane angle BEFORE
// we try closed-loop FOC. Motor is never driven — turn the shaft
// by hand and the angle should sweep smoothly 0 -> 360 -> 0.
//
//   • Angle tracks the shaft smoothly      -> encoder is good.
//   • Angle jumps / sticks / reads garbage -> wiring, PWM pin, or
//                                              calibration problem.
//
// AS5048A in PWM mode: PWM out -> pin 11 (PC3), plus 3.3V and GND.
//
// BUILD:   pio run -e enctest -t upload
// MONITOR: pio device monitor -e enctest
//
// SERIAL: 'Z' zero the angle here,  'D' enter DFU
// ============================================================

#include <Arduino.h>
#include <SimpleFOC.h>
#include "config.h"
#include "dfu_jump.h"

// AS5048A magnetic encoder, PWM output mode.
// Same calibration as the real firmware: pulse 2us..903us = 0..360deg.
MagneticSensorPWM encoder(ENCODER_PWM_PIN, 2, 903);

// SimpleFOC's built-in PWM capture ISR (matches main.cpp approach)
void doPWM() { encoder.handlePWM(); }

float zero_offset = 0.0f;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0 < 3000)) {}
    delay(500);

    Serial.println("====================================================");
    Serial.println(" ENCODER TEST — AS5048A readout (motor OFF)");
    Serial.println(" PWM in on pin 11 (PC3). Turn shaft by hand.");
    Serial.println("====================================================");

    encoder.init();
    encoder.enableInterrupt(doPWM);

    delay(200);
    Serial.print("[ENC] initial angle = ");
    Serial.print(encoder.getAngle() * RAD_TO_DEG, 1);
    Serial.println(" deg");
    Serial.println("[ENC] Rotate the shaft — angle should track smoothly.");
    Serial.println("[ENC] 'Z' = zero here,  'D' = enter DFU");
}

void loop() {
    // Must be called continuously so the sensor updates.
    encoder.update();

    static uint32_t last = 0;
    if (millis() - last >= 150) {
        last = millis();

        float mech = encoder.getMechanicalAngle() * RAD_TO_DEG;   // 0..360 within one turn
        float cumulative = encoder.getAngle() * RAD_TO_DEG;        // includes full turns
        float zeroed = cumulative - zero_offset;

        Serial.print("[ENC] mech=");
        Serial.print(mech, 1);
        Serial.print(" deg   total=");
        Serial.print(cumulative, 1);
        Serial.print(" deg   zeroed=");
        Serial.print(zeroed, 1);
        Serial.println(" deg");
    }

    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'Z' || c == 'z') {
            zero_offset = encoder.getAngle() * RAD_TO_DEG;
            Serial.println("[ENC] zeroed.");
        }
        if (c == 'D' || c == 'd') jumpToBootloader();
    }
}
