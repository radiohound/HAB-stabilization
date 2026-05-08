#pragma once
// ============================================================
// config.h — HAB Stabilization System Configuration
// All hardware pins and tunable constants live here.
// ============================================================

// ------------------------------------------------------------
// Motor — iPower GBM2804H-100T
// ------------------------------------------------------------
#define MOTOR_POLE_PAIRS    7       // 12N14P = 7 pole pairs
#define MOTOR_PHASE_R       10.0f   // phase resistance, ohms
#define MOTOR_KV            154.0f  // RPM/V

// ------------------------------------------------------------
// SimpleFOC Mini — 3-PWM pins (Feather STM32F405)
// Use Arduino pin numbers (PC_x format hangs pinMode on this core)
// D6 = PC6 (TIM3_CH1)
// D5 = PC7 (TIM3_CH2)
// TX = PB10 (TIM2_CH3) — Arduino pin 1
// D9 = PB8
// ------------------------------------------------------------
#define DRIVER_PIN_IN1      6      // D6  (PC6, TIM3_CH1)
#define DRIVER_PIN_IN2      5      // D5  (PC7, TIM3_CH2)
#define DRIVER_PIN_IN3      12     // D12 (PC2) — moved off TX/PB10 (UART conflict)
#define DRIVER_PIN_EN       9      // D9  (PB8)

// Power supply voltage (6× AA L91 = 9.0V nominal)
#define SUPPLY_VOLTAGE      9.0f
// Motor voltage limit — leaves headroom for current control
#define VOLTAGE_LIMIT       7.0f

// ------------------------------------------------------------
// AS5048A Encoder — PWM output (3-wire: VCC, GND, PWM)
// PWM  → PC3  (D11 on Feather silkscreen)
// ------------------------------------------------------------
#define ENCODER_PWM_PIN     11     // D11 (Arduino pin number, not PC_3)

// ------------------------------------------------------------
// BNO085 IMU — I2C1 via STEMMA QT (PB6=SCL, PB7=SDA)
// Uses Wire (I2C1) — no pin defines needed, hardware default
// ------------------------------------------------------------
#define BNO085_ADDR         0x4A    // default I2C address

// ------------------------------------------------------------
// Battery monitoring — voltage divider on A0 (PA4)
// Divider: 10kΩ top + 3.3kΩ bottom → full scale ~12V
// Adjust VDIV_RATIO to match your actual resistors:
//   ratio = (R_top + R_bottom) / R_bottom
// Default: (10000 + 3300) / 3300 = 4.03
// ------------------------------------------------------------
#define BATT_PIN            A0
#define VDIV_RATIO          4.03f
#define ADC_REF_VOLTAGE     3.3f
#define BATT_ADC_COUNTS     4096.0f     // 12-bit ADC (renamed from ADC_RESOLUTION to avoid
                                        // clash with STM32duino core define of same name)
#define BATT_CUTOFF_VOLTS   7.2f        // 1.2V × 6 cells
#define BATT_WARN_VOLTS     7.8f        // 1.3V × 6 cells

// ------------------------------------------------------------
// UART Telemetry — Serial3 (PB10=TX, PB11=RX)
// NOTE: PB10 is shared with IN3 — see README for mux note.
// If sharing causes issues, use Serial (USB) for telemetry.
// ------------------------------------------------------------
// Use USB serial for telemetry (safest on this pinout)
// Serial  = USB CDC  (debug + telemetry)
// Serial3 = Hardware UART on PB10/PB11 (alternative)
#define TELEM_BAUD          115200
#define TELEM_INTERVAL_MS   30000UL     // 30 seconds

// ------------------------------------------------------------
// PID Controller — Heading outer loop
// Tuning starting point — adjust on bench:
//   Increase Kp until payload oscillates, then halve it.
//   Add Ki slowly to eliminate steady-state error.
//   Add Kd to dampen overshoot.
// ------------------------------------------------------------
#define PID_KP              0.50f
#define PID_KI              0.02f
#define PID_KD              0.08f
#define PID_OUTPUT_LIMIT    VOLTAGE_LIMIT   // clamp torque command
#define PID_RATE_HZ         100             // outer loop rate

// ------------------------------------------------------------
// Heading control
// ------------------------------------------------------------
#define TARGET_HEADING_DEG  0.0f    // degrees, 0 = North
                                    // 0-360, CW positive
// Dead-band — don't correct tiny errors (saves power)
#define HEADING_DEADBAND_DEG  1.0f

// Altitude threshold above which we switch to gyro-dominant
// heading (magnetometer unreliable above ~15km)
// Altitude comes from tracker UART — set 0 to disable switch
#define MAG_CUTOFF_ALT_M    15000

// ------------------------------------------------------------
// Control loop timing
// ------------------------------------------------------------
#define FOC_LOOP_RATE_HZ    1000    // inner FOC loop
#define PID_LOOP_PERIOD_US  (1000000 / PID_RATE_HZ)

// ------------------------------------------------------------
// Debug output verbosity
// 0 = silent, 1 = errors only, 2 = status, 3 = verbose
// ------------------------------------------------------------
#define DEBUG_LEVEL         2
