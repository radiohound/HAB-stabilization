#pragma once
// ============================================================
// battery.h — Battery Monitor
// Reads 6× AA pack voltage via voltage divider on A0.
// ============================================================
#include <Arduino.h>
#include "config.h"

// Call once in setup()
void batt_init();

// Read and return battery voltage in volts
// Averages 16 samples to reduce ADC noise
float batt_read_voltage();

// Returns true if voltage is above cutoff threshold
bool batt_is_ok();

// Returns true if voltage is below warning threshold (but above cutoff)
bool batt_is_low();

// ============================================================
// battery.cpp
// ============================================================
#ifdef BATT_IMPL

void batt_init() {
    analogReadResolution(12); // 12-bit ADC on STM32F405
    pinMode(BATT_PIN, INPUT);

    #if DEBUG_LEVEL >= 2
    Serial.print("[BATT] Initial voltage: ");
    Serial.print(batt_read_voltage(), 2);
    Serial.println(" V");
    #endif
}

float batt_read_voltage() {
    // Average 16 samples
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) {
        sum += analogRead(BATT_PIN);
        delayMicroseconds(100);
    }
    float adc_avg = (float)sum / 16.0f;

    // Convert ADC reading to voltage at pin
    float v_pin = (adc_avg / ADC_RESOLUTION) * ADC_REF_VOLTAGE;

    // Scale back through voltage divider
    float v_batt = v_pin * VDIV_RATIO;

    return v_batt;
}

bool batt_is_ok() {
    return batt_read_voltage() > BATT_CUTOFF_VOLTS;
}

bool batt_is_low() {
    float v = batt_read_voltage();
    return (v > BATT_CUTOFF_VOLTS) && (v < BATT_WARN_VOLTS);
}

#endif // BATT_IMPL
