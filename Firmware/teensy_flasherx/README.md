# Teensy 4.1 Kids Ride-On Controller

A comprehensive motor controller firmware for kids' ride-on vehicles featuring dual control modes (steering wheel/pedal and RC remote), safety features, and OTA firmware updates.

## Table of Contents
- [Features](#features)
- [Hardware Overview](#hardware-overview)
- [Control Modes](#control-modes)
- [Safety Features](#safety-features)
- [Configuration Reference](#configuration-reference)
- [Pin Assignments](#pin-assignments)
- [Serial Commands](#serial-commands)
- [OTA Updates](#ota-updates)
- [Troubleshooting](#troubleshooting)

---

## Features

- **Dual Control Modes**: Car mode (steering wheel + pedal) and RC Remote mode (parental override)
- **Rollover Protection**: Speed-based steering limiting and automatic slowdown for sharp turns
- **Soft Stop**: Gradual motor ramp-down to prevent jarring stops on geared motors
- **Encoder Feedback**: Closed-loop steering control using AS5600 magnetic encoder
- **Current Sensing**: Monitor motor current for all 6 motor channels
- **Real-time Telemetry**: Live monitoring via web serial interface
- **OTA Firmware Updates**: Update firmware wirelessly via ESP32-S3 web interface

---

## Hardware Overview

### Microcontroller
- **Teensy 4.1** - 600MHz ARM Cortex-M7
- 10-bit ADC for analog inputs
- I2C for AS5600 encoder
- Multiple UARTs for ESP32 communication

### Motor Drivers (x3)
- **BTS7960** H-Bridge drivers
- Steering motor: Position control with encoder feedback
- Front drive motor: Speed/direction control
- Rear drive motor: Speed/direction control
- 24V power input, 5V enable pins (via level shifter)

### Sensors
- **Steering Pot**: Analog potentiometer on steering column
- **Throttle Pot**: Analog potentiometer on foot pedal
- **AS5600 Encoder**: 12-bit magnetic encoder on steering motor
- **RC Receiver**: PWM signals from RC transmitter

---

## Control Modes

### Mode 1: Car Mode (Default)
The child controls the vehicle using:
- **Steering wheel** → Controls steering motor position
- **Throttle pedal** → Controls front/rear motor speed
- **Shifter** → FWD/REV/NEUTRAL (required for any movement)

**Note**: Steering is disabled in NEUTRAL for safety.

### Mode 2: RC Remote Mode (Parental Override)
The parent takes full control using an RC transmitter:
- **Steer stick (steer2)** → Controls steering (ignores child's steering wheel)
- **Throttle stick (throttle2)** → Controls speed AND direction (ignores shifter)

#### Activating RC Mode
1. Pull RC throttle stick to **full reverse** (≤-90%)
2. **Hold for 3 seconds** → Mode becomes "ARMED"
3. **Release to neutral** → Mode fully activates
4. Serial output: `[RC] *** RC MODE FULLY ACTIVATED ***`

#### Deactivating RC Mode
- Leave **both sticks at neutral** for **20 seconds**
- Mode automatically returns to Car mode
- Serial output: `[RC] *** RC MODE TIMEOUT ***`

**Important**: The "release to neutral" safety prevents accidentally putting the car in reverse when activating RC mode.

---

## Safety Features

### 1. Rollover Protection
Prevents tipping during high-speed turns.

**Part A - Speed-Based Steering Limit:**
| Speed | Max Steering Allowed |
|-------|---------------------|
| 0-50% | Full (±100%) |
| 50-80% | Progressively limited |
| 80%+ | Only ±15% |

**Part B - Steering-Based Speed Reduction:**
| Steering Angle | Max Speed Allowed |
|----------------|-------------------|
| 0-30% | Full (100%) |
| 30-100% | Progressively limited |
| 100% | Only 50% speed |

**How it works**: When you turn sharply, the car slows down FIRST, then allows the turn. This prevents rollovers.

### 2. Soft Stop
Prevents jarring stops when releasing throttle on geared motors.

- Ramps down at **5% per 50ms** (approximately 1 second to stop)
- Acceleration is still immediate (no delay when pressing throttle)
- Direction changes require ramping to zero first

### 3. Neutral Safety
- Steering motor is disabled when shifter is in NEUTRAL
- Prevents unintended steering movement when stationary

### 4. RC Mode Activation Safety
- Requires 3-second hold at full reverse to arm
- Must return to neutral before fully activating
- Prevents accidental reverse when taking control

### 5. RC Mode Timeout
- Automatically returns control to car after 20 seconds of no RC input
- Prevents runaway if parent drops the controller

---

## Configuration Reference

All configurable values are at the top of `teensy_flasherx.ino`.

### Steering Potentiometer (Car's Steering Wheel)
```cpp
#define STEERING_POT_MIN      105     // Raw ADC at full LEFT
#define STEERING_POT_MAX      708     // Raw ADC at full RIGHT  
#define STEERING_POT_CENTER   407     // Raw ADC at CENTER
#define STEERING_POT_DEADZONE 20      // Ignore values within ±20 of center
#define STEERING_INPUT_SMOOTH 15      // Ignore changes < 15% (jitter filter)
#define STEERING_FILTER_ALPHA 0.3     // Low-pass filter (0.0=max filter, 1.0=no filter)
```

### Steering Motor (Encoder-Based Position Control)
```cpp
#define STEER_ANGLE_CENTER    234     // Center angle in degrees
#define STEER_ANGLE_RANGE     45      // ±45° from center (total 90° range)
#define STEER_ANGLE_DEADBAND  15      // Stop motor when within ±15° of target
#define ENCODER_ANGLE_OFFSET  180     // Offset to calibrate encoder zero
```

### Throttle Potentiometer (Foot Pedal)
```cpp
#define THROTTLE_POT_MIN      249     // Raw ADC at NO throttle
#define THROTTLE_POT_MAX      790     // Raw ADC at FULL throttle
#define THROTTLE_POT_DEADZONE 30      // Ignore small values (prevents creep)
```

### RC Receiver (PWM Signals)
```cpp
#define RC_STEER2_PIN         21      // Steering stick input
#define RC_THROTTLE2_PIN      20      // Throttle stick input
```
**Calibrated PWM ranges:**
- Steering: 1065µs (left) → 1500µs (center) → 2000µs (right)
- Throttle: 1000µs (reverse) → 1500µs (neutral) → 2000µs (forward)

### Soft Stop
```cpp
#define SOFT_STOP_RATE        5       // % decrease per step (lower = smoother)
#define SOFT_STOP_INTERVAL    50      // Milliseconds between steps
```
**Tuning**: For slower/smoother stops, decrease RATE to 3 or 2. For faster stops, increase to 8 or 10.

### Rollover Protection
```cpp
#define ROLLOVER_PROTECTION_ENABLED  true   // Master enable (set false to disable)
#define ROLLOVER_SPEED_THRESHOLD     50     // Speed % where limiting starts
#define ROLLOVER_SPEED_FULL_LIMIT    80     // Speed % where max limit applies
#define ROLLOVER_MIN_STEER_RANGE     15     // Min steering % at high speed
#define ROLLOVER_SLOWDOWN_ANGLE      30     // Steer % where speed reduction starts
#define ROLLOVER_SLOWDOWN_FACTOR     50     // Max speed % at full steering
```

**Tuning Guide:**
| Want to... | Adjust |
|------------|--------|
| Allow more steering at high speed | Increase `ROLLOVER_MIN_STEER_RANGE` |
| Start limiting earlier | Decrease `ROLLOVER_SPEED_THRESHOLD` |
| Slow down more for turns | Decrease `ROLLOVER_SLOWDOWN_FACTOR` |
| Start slowing earlier | Decrease `ROLLOVER_SLOWDOWN_ANGLE` |

### Motor Configuration
```cpp
#define MOTOR_PWM_FREQ        20000   // PWM frequency (Hz)
#define MOTOR_MAX_SPEED       255     // Max PWM value (0-255)
```

---

## Pin Assignments

### Analog Inputs
| Pin | Function |
|-----|----------|
| 17 (A3) | Steering Pot |
| 23 (A9) | Throttle Pot |
| 20 (A6) | RC Throttle2 |
| 21 (A7) | RC Steer2 |
| 24-27, 38-39 | Current Sensors |

### Digital Inputs
| Pin | Function |
|-----|----------|
| 28 | Shifter Reverse |
| 29 | Shifter Forward |
| 30 | Mode Select |
| 31 | E-Brake |

### PWM Outputs (Motor Drivers)
| Pins | Motor |
|------|-------|
| 2, 3 | Steering LPWM/RPWM |
| 6, 9 | Front LPWM/RPWM |
| 12, 13 | Rear LPWM/RPWM |

### Enable Outputs (via Level Shifter)
| Pins | Motor |
|------|-------|
| 4, 5 | Steering L_EN/R_EN |
| 10, 11 | Front L_EN/R_EN |
| 32, 33 | Rear L_EN/R_EN |

### I2C (Encoder)
| Pin | Function |
|-----|----------|
| 18 | SDA |
| 19 | SCL |

### UART (ESP32)
| Pins | Function |
|------|----------|
| 7, 8 | Serial2 (OTA Flash) |
| 14, 15 | Serial3 (Telemetry) |

---

## Serial Commands

Connect via USB Serial at 115200 baud.

| Command | Description |
|---------|-------------|
| `help` | Show all commands |
| `status` | Show system status |
| `version` | Show firmware version |
| `telem` or `t` | Toggle telemetry on/off |
| `header` or `h` | Print telemetry header |
| `verbose` or `v` | Verbose telemetry mode |
| `compact` or `c` | Compact telemetry mode |
| `rate <ms>` | Set telemetry rate (10-1000ms) |
| `rctest` or `rc` | Test RC receiver PWM signals |
| `calenc` or `ce` | Encoder calibration helper |

---

## Telemetry Output

The telemetry displays real-time data in the web serial monitor:

```
|------- RAW INPUTS ------|--- PROCESSED (%) ---|-- PWM OUTPUTS --|--- CURRENT ---|SOURCE|-- SYS --|
| Steer Throt Str2  Thr2  Encdr | Steer Throt Str2  Thr2  | StL  StR  FrL  FrR  ReL  ReR| ...      |      | Loop  T |
```

**Columns:**
- **RAW INPUTS**: Steering pot, Throttle pot, RC steer, RC throttle, Encoder angle
- **PROCESSED**: Converted to -100% to +100% or 0-100%
- **PWM OUTPUTS**: Current PWM values for all 6 motor channels
- **CURRENT**: Motor current in mA (if sensors calibrated)
- **SOURCE**: "Car" or "Remot" (which input is active)
- **SYS**: Loop time (µs) and CPU temperature

---

## OTA Updates

Firmware can be updated wirelessly via the ESP32-S3 web interface.

### How It Works
1. ESP32 hosts a web page with file upload
2. User uploads `.hex` file from Arduino IDE
3. ESP32 sends firmware to Teensy via Serial2
4. Teensy writes to flash buffer and reboots

### To Generate .hex File
In Arduino IDE: Sketch → Export Compiled Binary

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Motors don't move | Check EN pins going HIGH, verify shifter not in NEUTRAL |
| Steering oscillates | Increase `STEER_ANGLE_DEADBAND`, check encoder wiring |
| Steering jitters | Increase `STEERING_INPUT_SMOOTH`, decrease `STEERING_FILTER_ALPHA` |
| RC mode won't activate | Hold throttle full reverse 3+ seconds, then release to neutral |
| Car tips over in turns | Decrease `ROLLOVER_MIN_STEER_RANGE` and `ROLLOVER_SLOWDOWN_FACTOR` |
| Stops too abruptly | Decrease `SOFT_STOP_RATE` to 2 or 3 |
| RC reading 0 | Use `rctest` command to check PWM signals, verify wiring |

---

## Version History

- **1.5.3** - Rollover protection, soft stop, RC mode improvements
- **1.5.x** - RC receiver PWM support, source display
- **1.4.x** - Encoder-based steering control
- **1.3.x** - Initial motor control implementation

---

## License

This project is open source. See LICENSE file for details.
