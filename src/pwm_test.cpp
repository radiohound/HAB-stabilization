// ============================================================
// pwm_test.cpp — BARE-METAL 3-phase test, NO SimpleFOC
//
// Purpose: prove the STM32 can actually generate PWM on the
// driver pins and spin the motor, with zero SimpleFOC involved.
// Uses only Arduino analogWrite() on IN1/IN2/IN3 + EN high.
//
//   • Motor spins / outputs switch  -> the MCU + Mini + motor are
//     all good, and SimpleFOC's driver setup is the problem.
//   • IN pins still 0V here too      -> deeper pin/timer issue.
//
// BUILD:   pio run -e pwmtest -t upload
// MONITOR: pio device monitor -e pwmtest
//
// SERIAL: '+'/'-' = stronger/weaker,  'f'/'s' = faster/slower,
//         '0' = stop (hold),          'D' = enter DFU
// ============================================================

#include <Arduino.h>
#include "config.h"
#include "dfu_jump.h"

const int PH_A = DRIVER_PIN_IN1;   // pin 9  (PB8, TIM4_CH3)
const int PH_B = DRIVER_PIN_IN2;   // pin 10 (PB9, TIM4_CH4)
const int PH_C = DRIVER_PIN_IN3;   // pin 5  (PC7, TIM3_CH2)
const int EN   = DRIVER_PIN_EN;    // pin 6  (PC6) digital enable

float angle     = 0.0f;
float amplitude = 0.40f;   // 0..1 modulation depth (start gentle)
float speed     = 0.012f;  // electrical rad added per loop

const float TWO_PI_3 = 2.0943951f;  // 120 degrees

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0 < 3000)) {}

    Serial.println("====================================================");
    Serial.println(" BARE-METAL PWM TEST — analogWrite, no SimpleFOC");
    Serial.println(" Driving IN1/IN2/IN3 on pins 9 / 10 / 5, EN on 6");
    Serial.println("====================================================");

    pinMode(EN, OUTPUT);
    digitalWrite(EN, HIGH);             // wake the DRV8313

    analogWriteResolution(8);           // duty 0..255
    analogWriteFrequency(20000);        // 20 kHz

    // Prime all three to mid-rail so they're configured as PWM outputs
    analogWrite(PH_A, 128);
    analogWrite(PH_B, 128);
    analogWrite(PH_C, 128);

    Serial.println("[PWM] EN high, 3-phase sine running.");
    Serial.println("[PWM] '+'/'-' strength, 'f'/'s' speed, '0' stop, 'D' DFU");
}

void loop() {
    int da = 128 + (int)(127.0f * amplitude * sin(angle));
    int db = 128 + (int)(127.0f * amplitude * sin(angle - TWO_PI_3));
    int dc = 128 + (int)(127.0f * amplitude * sin(angle + TWO_PI_3));
    analogWrite(PH_A, da);
    analogWrite(PH_B, db);
    analogWrite(PH_C, dc);

    angle += speed;
    if (angle > TWO_PI) angle -= TWO_PI;

    delayMicroseconds(800);

    if (Serial.available()) {
        char ch = Serial.read();
        if (ch == '+') amplitude = min(0.95f, amplitude + 0.1f);
        if (ch == '-') amplitude = max(0.0f,  amplitude - 0.1f);
        if (ch == 'f') speed *= 1.5f;
        if (ch == 's') speed /= 1.5f;
        if (ch == '0') { amplitude = 0.0f; }
        if (ch == 'D' || ch == 'd') jumpToBootloader();
        if (ch != '\r' && ch != '\n') {
            Serial.print("[PWM] amplitude=");
            Serial.print(amplitude, 2);
            Serial.print("  speed=");
            Serial.println(speed, 4);
        }
    }

    static uint32_t last = 0;
    if (millis() - last >= 1000) {
        last = millis();
        Serial.print("[PWM] duty A/B/C = ");
        Serial.print(da); Serial.print(" / ");
        Serial.print(db); Serial.print(" / ");
        Serial.println(dc);
    }
}
