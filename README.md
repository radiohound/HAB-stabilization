# HAB Payload Stabilization — Arm Assembly Firmware

**K6ATV — April 2026**
**Status:** working closed-loop heading stabilization, bench-verified June 2026.

Holds the payload's heading by counter-rotating a brushless motor as the
support arm twists. No slip rings, no rotating electrical connections:

```
payload_heading = arm_yaw (BNO085 IMU) + motor_angle (AS5048A encoder)
PID drives the motor to minimise (payload_heading − target)
```

---

## Hardware

| Component | Part | Notes |
|---|---|---|
| MCU | Adafruit Feather STM32F405 Express | |
| Driver | SimpleFOC Mini v1.1 | DRV8313 |
| Motor | iPower GBM2804H-100T | 12N14P → **7 pole pairs**, ~9–10 Ω |
| Encoder | AS5048A (integrated with motor) | run in **PWM output** mode (1 signal wire) |
| IMU | Adafruit BNO085 #4754 | STEMMA QT (I²C) |
| Battery | 6× Energizer L91 AA in series | 9 V nominal |

---

## Wiring

### Power
```
9 V supply ──► SimpleFOC Mini VIN
               SimpleFOC Mini GND ──► System GND
```

> ⚠️ Never connect 9 V to the Feather BAT or USB pins. The Feather is powered
> over USB (bench) or its own regulator; the 9 V only feeds the Mini's VIN.

### SimpleFOC Mini → Feather  *(as-built, matches `config.h`)*
```
SimpleFOC IN1  ──► Feather 9    (PB8, TIM4_CH3)   PWM
SimpleFOC IN2  ──► Feather 10   (PB9, TIM4_CH4)   PWM
SimpleFOC IN3  ──► Feather 5    (PC7, TIM3_CH2)   PWM
SimpleFOC EN   ──► Feather 6    (PC6)             digital enable
SimpleFOC GND  ──► Feather GND  (shared signal ground — REQUIRED)
```

> **Pin-choice rule:** all three IN pins must be on a hardware **timer**
> channel (PWM-capable). `EN` can be any digital pin. The phase order of
> IN1/IN2/IN3 only sets spin direction — any permutation works.
>
> **Gotcha (learned the hard way):** Feather pin **D12 (PC2) has NO timer**
> on the STM32F405 — it physically cannot output PWM. Do not use it for an
> IN pin. Verified PWM-capable pins used above.

### AS5048A Encoder — PWM output mode (3 wires)
```
Encoder VCC ──► Feather 3V
Encoder GND ──► Feather GND
Encoder PWM ──► Feather 11  (PC3)
```
Calibrated pulse range (bench): 2 µs–903 µs = 0°–360° (`MagneticSensorPWM`
args in `main.cpp`).

> The AS5048A is natively a 14-bit **SPI** encoder; PWM is its lower-resolution
> fallback. PWM works but is noisy (≈±5° heading jitter, some motor hum,
> non-repeatable FOC alignment). See *Known Limitations* for the SPI upgrade.

### BNO085 IMU
```
STEMMA QT cable to the Feather STEMMA QT / I²C port
(SDA = PB7, SCL = PB6, 3.3 V, GND).  I²C address 0x4A.
```
The STEMMA QT connector is the suspected weak point for the intermittent
IMU dropouts — a soldered I²C connection is more flight-worthy.

### Battery Voltage Monitor
```
9 V ──► 10kΩ ──► A0 (PA4) ──► 3.3kΩ ──► GND
```
Set `VDIV_RATIO` in `config.h` to match your resistors.
> ⚠️ Currently mis-reads (telemetry battery field is not trustworthy) — the
> divider isn't matched/connected. Left as-is by design; bench mode ignores
> the cutoff so it does not affect operation.

---

## Software Setup

1. Install the [PlatformIO](https://platformio.org/) extension in VS Code.
2. Open this folder. PlatformIO auto-installs dependencies:
   - **Simple FOC v2.4.0**  *(note: 2.4.x, not 2.3.x — see below)*
   - Adafruit BNO08x, Adafruit BusIO
3. Build the real firmware: env **`adafruit_feather_f405`**.
4. Flash — see **Uploading** below.

> **SimpleFOC 2.4.0 note (important if you modify motor code):** in 2.4.0,
> `motor.move()` only computes the set-point — **`motor.loopFOC()` is what
> actually applies the phase voltage**, for open-loop too. The loop must call
> `loopFOC()` *then* `move()`. (Older SimpleFOC applied voltage inside
> `move()`.) `main.cpp` already does this correctly.

### Uploading (DFU)

**Preferred — software trigger (no jumper):**
1. In the serial monitor, type **`D`** + Enter. The board reboots into the
   ROM bootloader on its own.
2. Run the PlatformIO **Upload** for the target env.

**Fallback — hardware:** apply **3.3 V to the B0 pin** (BOOT0 high — *not*
ground), tap **RESET**, remove the jumper. The board enumerates as a DFU
device; run Upload. (Power-on with B0 high is the most reliable trigger.)

---

## Bench Test Environments

Independent test sketches isolate each subsystem. Each builds only its own
file (`build_src_filter` in `platformio.ini`). Invaluable for bring-up:

| Env | What it does |
|---|---|
| `adafruit_feather_f405` | **The real firmware** (`main.cpp`) |
| `pwmtest` | Bare-metal `analogWrite` 3-phase spin — **no SimpleFOC**. Proves driver + wiring can rotate the motor. |
| `motortest` | SimpleFOC **open-loop** velocity (no encoder/IMU). Proves the SimpleFOC driver path. |
| `enctest` | AS5048A **encoder readout**, motor off. Turn the shaft, watch the angle. |
| `foctest` | **Closed-loop FOC** (motor + encoder, torque mode). Validates `initFOC` alignment. |
| `native` | Host-side unit tests for the heading math (`pio test -e native`). |

Each test sketch also supports the `D` DFU command.

---

## First Run — Bench Setup

### 1. Verify the encoder (`enctest`)
Flash `enctest`, open the serial monitor at 115200. Turn the motor shaft by
hand — `mech` should sweep smoothly 0→360 and wrap. Jumps/garbage → check the
PWM wire on pin 11 and the encoder's 3.3 V/GND.

### 2. Verify the motor spins (`pwmtest` then `motortest`)
`pwmtest` should spin the motor with raw PWM (proves hardware). `motortest`
should spin it under SimpleFOC open loop (`T10`, `V4` to drive it).

### 3. Verify closed-loop FOC (`foctest`)
Flash `foctest`, **keep the shaft free** — `initFOC()` twitches the motor to
align. Look for:
```
MOT:sensor dir: CW
MOT:PP check: OK!        (pole pairs = 7 confirmed)
[FOC] initFOC result: SUCCESS
```
Then `T1` should spin it **smoothly**.

### 4. Real firmware + sign convention (`adafruit_feather_f405`)
On boot you should see `[IMU] BNO085 initialised OK`, `[MOTOR] FOC ready`.
**Critical:** rotate the payload **clockwise** (from above) and watch
`payload_hdg` — it should **increase**. If it decreases, negate the motor
angle in `heading_control.h::hc_update()`:
```cpp
float motor_angle_deg = -(motor_shaft_rad * (180.0f / PI)); // negate
```

### 5. Test heading hold
Rotate the support base by hand — the motor should counter-rotate to keep the
payload on target. Use `T<deg>` to command a heading (`T0`, `T090`, …).

---

## PID Tuning

Current bench baseline in `config.h` (light payload):
```
PID_KP = 0.50
PID_KI = 0.02
PID_KD = 0.08      (raw derivative)
HEADING_DEADBAND_DEG = 1.0
```
This held heading best on the bench (~±5° jitter, acceptable).

**Notes from tuning:**
- `Kd = 0` made it **oscillate/swing** (position control of inertia needs
  derivative damping).
- A low-pass-**filtered** Kd removed the noise spikes but added lag and felt
  worse — raw Kd won. If you revisit filtering, use a short time constant.
- The residual jitter is driven by the **noisy PWM encoder**, not the gains.
  SPI mode would let you raise Kd / filter cleanly.

**Re-tune with the actual flight mass** (camera + batteries) — inertia changes
the dynamics. Procedure: zero Ki/Kd, raise Kp to the edge of oscillation,
back off, add Ki for steady-state, add Kd for damping.

---

## Runtime Commands (Serial Monitor, 115200)

| Command | Effect |
|---|---|
| `T<degrees>` | Set target heading (0–360), e.g. `T0` (North), `T090` (East), `T180` (South) |
| `M…` | SimpleFOC motor commander (e.g. `MMV5` = velocity limit 5 rad/s) |
| `D` | Enter DFU bootloader (for flashing without the B0 jumper) |

---

## Telemetry Sentence Format

Output every 30 s on USB Serial:
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
| arm_yaw | Arm absolute heading (deg, 0–360) |
| payload_hdg | Computed payload heading (deg, 0–360) |
| error | Heading error (deg, signed) |
| motor_turns | Cumulative motor rotations since power-on |
| torque | Torque command (volts) |
| batt_v | Battery voltage *(currently unreliable — see Wiring)* |
| mag_cal / gyro_cal | BNO085 calibration 0–3 |
| imu_ok | IMU health (1 = ok, 0 = fault → motor holds still) |

---

## Files

```
platformio.ini          — build configuration (real firmware + test envs)
src/config.h            — ALL pins and tuning constants (edit here first)
src/main.cpp            — setup() and loop() (real firmware)
src/imu.h               — BNO085 driver, heading fusion, I²C bus recovery
src/heading_control.h   — PID controller and heading computation
src/battery.h           — voltage-divider battery monitor
src/telemetry.h         — serial output
src/dfu_jump.h          — software 'D' → ROM bootloader entry
src/motor_test.cpp      — bench: SimpleFOC open-loop (env motortest)
src/pwm_test.cpp        — bench: bare-metal PWM spin (env pwmtest)
src/encoder_test.cpp    — bench: encoder readout (env enctest)
src/foc_test.cpp        — bench: closed-loop FOC (env foctest)
```

---

## Known Limitations / Next Steps

- **Encoder in PWM mode** is the root cause of the ~±5° heading jitter,
  some motor hum, the non-repeatable FOC alignment (so the boot-time
  alignment can't be frozen), and the pole-pair-check warning during
  alignment (the encoder's PWM interrupt contends with the IMU's I²C).
  **Upgrade path:** wire the AS5048A in **SPI** mode (CLK/CS/MISO/MOSI) and
  switch to `MagneticSensorSPI`. This would smooth control, quiet the motor,
  remove the encoder ISR, and allow a frozen (twitch-free) alignment.
  *Deferred — ±5° hold is far better than the uncontrolled ~6 RPM free-spin.*

- **IMU init can be intermittent** (BNO085 occasionally not found on I²C,
  previously needing a power cycle). `imu_init()` now includes I²C
  bus-recovery + retries. Still, prefer a soldered I²C connection over the
  STEMMA QT plug for flight. This is **flight-critical**: if the IMU faults,
  the motor holds still and the payload free-spins.

- **FOC alignment runs live at every power-on** (the motor twitches). Fine
  for flight since you power up on the ground. A frozen alignment was tried
  but the PWM encoder isn't repeatable enough — revisit with SPI.

- **No current sensing** (SimpleFOC Mini limitation). Torque control is via
  voltage — adequate here.

- **Battery telemetry is wrong** (divider not matched/connected). Left as-is;
  bench mode ignores the cutoff.

- **Altitude→mag-cutoff switchover** currently uses elapsed time as a proxy.
  Should parse altitude from the tracker UART and call
  `imu_set_gyro_only_mode()` on real altitude.

- **Re-tune the PID** with the final flight mass before launch.
```
