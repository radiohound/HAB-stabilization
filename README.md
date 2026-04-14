# HAB Payload Stabilization — Arm Assembly Firmware

**K6ATV — April 2026**

---

## Hardware

| Component | Part | Notes |
|---|---|---|
| MCU | Adafruit Feather STM32F405 Express | |
| Driver | SimpleFOC Mini v1.1 | DRV8313, 5A max |
| Motor | iPower GBM2804H-100T | 12N14P, 10Ω, 154Kv |
| Encoder | AS5048A (integrated with motor) | SPI, 6 wires |
| IMU | Adafruit BNO085 #4754 | STEMMA QT |
| Battery | 6× Energizer L91 AA in series | 9V nominal |

---

## Wiring

### Power
```
6× AA L91 (9V) ──► SimpleFOC Mini VIN
                    SimpleFOC Mini GND ──► System GND
                    SimpleFOC Mini 3.3V ──► Feather 3V pin
```

> ⚠️ Never connect 9V to Feather BAT or USB pins.

### SimpleFOC Mini → Feather
```
SimpleFOC IN1  ──► Feather GPIO 6  (PC6,  TIM3_CH1)
SimpleFOC IN2  ──► Feather GPIO 5  (PC7,  TIM3_CH2)
SimpleFOC IN3  ──► Feather TX      (PB10, TIM2_CH3)
SimpleFOC EN   ──► Feather GPIO 9  (PA15)
SimpleFOC GND  ──► Feather GND
```

### AS5048A Encoder (integrated with motor, 6 wires)
```
Encoder VCC   ──► Feather 3V
Encoder GND   ──► Feather GND
Encoder MISO  ──► Feather MISO (PB14)
Encoder MOSI  ──► Feather MOSI (PB15)
Encoder SCK   ──► Feather SCK  (PB13)
Encoder CSN   ──► Feather GPIO (PB0)  ← change in config.h if needed
```

### BNO085 IMU
```
STEMMA QT cable directly to Feather STEMMA QT port
(SDA=PB7, SCL=PB6, 3.3V, GND)
```

### Battery Voltage Monitor
```
9V battery ──► 10kΩ ──► A0 (PA4) ──► 3.3kΩ ──► GND
```
Adjust `VDIV_RATIO` in `config.h` to match your actual resistors.

### PB10 Note
PB10 is used for both SimpleFOC IN3 and Serial3 TX. The firmware
uses USB Serial (Serial) for telemetry, so this conflict is avoided.
If you need hardware UART to the tracker, use a different IN3 pin
and update `DRIVER_PIN_IN3` in config.h.

---

## Software Setup

1. Install [PlatformIO](https://platformio.org/) extension in VS Code
2. Open this folder in VS Code
3. PlatformIO will install dependencies automatically:
   - Simple FOC v2.3.4
   - Adafruit BNO08x v1.2.3
   - Adafruit BusIO v1.14.5
4. Build: `Ctrl+Alt+B`
5. Upload: Put Feather in DFU mode (double-tap reset), then `Ctrl+Alt+U`

---

## First Run — Bench Setup

### 1. Verify encoder
Open serial monitor at 115200 baud. You should see:
```
[ENC] Initial encoder angle: xxx.x deg
```
Manually rotate the motor shaft. The angle should change continuously
without jumps. If it jumps or reads garbage, check SPI wiring and CSN pin.

### 2. Verify IMU
You should see:
```
[IMU] BNO085 initialised OK
[IMU] Mode: Rotation Vector (fused with mag)
```
Rotate the arm assembly. Watch arm_yaw change in the 5-second debug output.

### 3. Verify motor alignment
The motor will make a brief movement during `initFOC()` to find the
electrical zero. This is normal. You should see:
```
[MOTOR] Running alignment... (do not move payload)
[MOTOR] FOC ready
```

### 4. Verify sign convention
**Critical step.** Manually rotate the payload CLOCKWISE (viewed from above).
Watch `payload_hdg` in the debug output. It should INCREASE.

If it decreases, the motor shaft angle sign is inverted. Add this to
`heading_control.h` in the `hc_update()` function:
```cpp
float motor_angle_deg = -(motor_shaft_rad * (180.0f / PI)); // note negation
```

### 5. Test heading control
With everything assembled, type in Serial monitor:
```
T0
```
The payload should rotate to face North (0°) and hold there.
Try `T090`, `T180`, `T270` — payload should move to each heading.

---

## PID Tuning

Starting values in `config.h`:
```
PID_KP = 0.50
PID_KI = 0.02
PID_KD = 0.08
```

**Tuning procedure:**
1. Set `PID_KI = 0`, `PID_KD = 0`
2. Increase `PID_KP` until payload oscillates around target
3. Halve `PID_KP`
4. Slowly increase `PID_KI` until steady-state error is eliminated
5. Add `PID_KD` to dampen overshoot

Tune with the actual payload attached — inertia affects tuning significantly.

---

## Runtime Commands (Serial Monitor)

| Command | Effect |
|---|---|
| `T<degrees>` | Set target heading (0-360) |
| `T0` | Point North |
| `T090` | Point East |
| `T180` | Point South |
| `M` | SimpleFOC motor commander |
| `MV5` | Set motor velocity limit to 5 rad/s |

---

## Telemetry Sentence Format

Output every 30 seconds on USB Serial:
```
$HAB,<uptime_s>,<arm_yaw>,<payload_hdg>,<error>,<motor_turns>,<torque>,<batt_v>,<mag_cal>,<gyro_cal>,<imu_ok>
```

Example:
```
$HAB,120,047.3,049.1,-1.8,0.33,0.24,8.71,3,3,1
```

| Field | Description |
|---|---|
| uptime_s | Seconds since power-on |
| arm_yaw | Arm absolute heading (degrees, 0-360) |
| payload_hdg | Computed payload heading (degrees, 0-360) |
| error | Heading error (degrees, signed) |
| motor_turns | Cumulative motor rotations since power-on |
| torque | Current torque command (volts) |
| batt_v | Battery voltage |
| mag_cal | Magnetometer calibration 0-3 |
| gyro_cal | Gyroscope calibration 0-3 |
| imu_ok | IMU health (1=ok, 0=fault) |

---

## Files

```
platformio.ini      — build configuration
src/config.h        — ALL pins and tuning constants (edit here first)
src/imu.h           — BNO085 driver and heading fusion
src/heading_control.h — PID controller and heading computation
src/battery.h       — voltage divider battery monitor
src/telemetry.h     — serial output
src/main.cpp        — setup() and loop()
```

---

## Known Limitations / Next Steps

- Altitude input for mag cutoff switchover currently uses elapsed time
  as a proxy. Full implementation should parse altitude from tracker
  UART and call `imu_set_gyro_only_mode()` based on actual altitude.

- No current sensing (SimpleFOC Mini doesn't support it).
  Torque control is via voltage — adequate for this application.

- Encoder sign convention must be verified on bench before flight.
  See "Verify sign convention" in bench setup above.

- PID gains require bench tuning with actual payload mass attached.
