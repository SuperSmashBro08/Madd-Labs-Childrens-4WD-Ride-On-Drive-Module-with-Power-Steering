# Kids Custom Power Wheels Project - Setup Instructions

## Table of Contents
1. [Required Parts & Tools](#required-parts--tools)
2. [Firmware Setup](#firmware-setup)
3. [Hardware Assembly](#hardware-assembly)
4. [Wiring Guide](#wiring-guide)
5. [First Power-Up](#first-power-up)

---

## Required Parts & Tools

### Electronics
| Qty | Part | Description |
|-----|------|-------------|
| 1 | Teensy 4.1 | Main microcontroller |
| 1 | ESP32-S3 DevKitC-1 (N8R8) | WiFi/Web server controller from Espressif |
| 3 | BTS7960 Motor Driver Modules | High-current H-bridge motor drivers (Steering, Front, Rear) |
| 1 | Pololu D24V50F5 | 5V 5A Step-down voltage regulator |
| 1 | AS5600 Encoder Module | Magnetic rotary encoder for steering feedback |
| 1 | FlySky FS-i6X | RC controller with receiver |
| 1 | Custom PCB | Main control board |

### Connectors & Hardware
| Qty | Part | Description |
|-----|------|-------------|
| 4 | XT60E-M | Panel mount male connectors |
| 6 | XT60 Female | Female bullet connectors |
| 8 | JST-VH 3.96mm 3-pin Female | For BTS7960 signal connections |
| 4 | JST-VH 3.96mm 3-pin Male | Mating connectors |
| 2 | JST-VH 3.96mm 4-pin Female | For additional connections |
| 1 | JST-VH 3.96mm 4-pin Male | Mating connector |
| 2 | JST-VH 3.96mm 5-pin Female | For receiver connection |
| 1 | JST-VH 3.96mm 5-pin Male | Mating connector |
| 1 | 30-40A Blade Fuse | Main power protection |
| 6 | M3x20mm Standoffs | PCB mounting (or 3D printed) |
| - | M3 Screws | Various lengths for assembly |
| - | M2.5 Screws | For XT60E-M connector mounting |
| - | Spade Connectors | For shifter connections |
| - | Dupont Connector Assortment | For misc connections |
| - | Heat Shrink Assortment | Wire insulation |

### Wire
| Qty | Part | Description |
|-----|------|-------------|
| - | 16 AWG Red Wire | Power positive connections |
| - | 16 AWG Black Wire | Power ground connections |

### Controls
| Qty | Part | Description |
|-----|------|-------------|
| 1 | Foot Pedal Potentiometer | Throttle control (Amazon) |
| 1 | Waterproof Potentiometer | Steering wheel feedback |
| 1-2 | 3-Way Rocker Switch | For shifter (if not using existing) |

### Tools Required
- Soldering station
- Wire cutters/strippers
- Crimping tools (for JST and Dupont connectors)
- Screwdrivers
- Multimeter (recommended)

---

## Firmware Setup

### Step 1: Configure WiFi Credentials

1. Navigate to the `Firmware/esp32_flasher/` folder
2. Open the `secrets.h` file
3. Add your WiFi network credentials:

```cpp
#define WIFI_SSID "YourWiFiNetworkName"
#define WIFI_PASSWORD "YourWiFiPassword"
```

4. Save the file

### Step 2: Flash the ESP32-S3

1. Connect the ESP32-S3 DevKitC-1 to your computer via USB
2. Open the Arduino IDE (or PlatformIO)
3. Select the correct board: **ESP32-S3 Dev Module**
4. Select the correct COM port
5. Open `Firmware/esp32_flasher/esp32_flasher.ino`
6. Click **Upload** to flash the firmware
7. **Important:** Open the Serial Monitor and set the baud rate to **115200**
8. The ESP32 will display its IP address once connected to your WiFi network
9. **Write down this IP address** - you will need it to access the web server

### Step 3: Flash the Teensy 4.1

1. Connect the Teensy 4.1 to your computer via USB
2. Open the Arduino IDE with Teensyduino installed
3. Select board: **Teensy 4.1**
4. Select the correct COM port
5. Open `Firmware/teensy_flasherx/teensy_flasherx.ino`
6. Click **Upload** to flash the firmware
7. Wait for the upload to complete

### Step 4: Access the Web Server

1. Open a web browser on a device connected to the same WiFi network
2. Type the IP address from Step 2 into the browser address bar
3. The web interface should load, allowing you to configure and control the system

---

## Hardware Assembly

### PCB Mounting

1. Install the 6x M3x20mm standoffs (or 3D printed standoffs) to mount the custom PCB in the enclosure
2. Secure with M3 screws

### Install the Fuse

1. Insert the **30-40A blade fuse** into the fuse slot on the custom PCB
2. Ensure it is fully seated

### Install the Voltage Regulator

1. Mount the Pololu D24V50F5 step-down regulator to the designated location on the PCB
2. This converts the 24-27V input to 5V for the control electronics

### Install Microcontrollers

1. Plug the **Teensy 4.1** into its designated socket on the custom PCB
2. Plug the **ESP32-S3 DevKitC-1** into its designated socket on the custom PCB
3. Ensure both are properly oriented and fully seated

### Mount XT60E-M Connectors

1. Install the 4x XT60E-M panel mount connectors in the enclosure
2. Secure with M2.5 screws

---

## Wiring Guide

### Power Input

1. Connect **24V to 27V** power source to the main power input
2. Use 16 AWG wire for all power connections
3. **Red = Positive (+)**
4. **Black = Negative/Ground (-)**

### BTS7960 Motor Driver Wiring

You have **3x BTS7960 modules** controlling:
- **Steering Motor**
- **Front Drive Motor**
- **Rear Drive Motor**

#### Power Connections (for each BTS7960):
| BTS7960 Pin | Connection |
|-------------|------------|
| B+ | 24-27V Positive (from PCB power out) |
| B- | Ground (from PCB power out) |
| M+ | Motor Positive |
| M- | Motor Negative |

Use **16 AWG red and black wire** for all power in/out connections to the BTS7960 modules.

#### Signal Connections (8-pin JST):

Each BTS7960 module has an 8-pin signal connection to the custom PCB.

**The pins are clearly labeled on both the BTS7960 module and the custom PCB - simply match the labels:**

| Pin | Signal |
|-----|--------|
| RPWM | Right PWM |
| LPWM | Left PWM |
| R_EN | Right Enable |
| L_EN | Left Enable |
| R_IS | Right Current Sense |
| L_IS | Left Current Sense |
| VCC | 5V Logic |
| GND | Ground |

Use the 3-pin JST-VH connectors to connect from the custom PCB to each BTS7960 module.

### FlySky Receiver Wiring

Connect the FlySky receiver to the custom PCB using the JST connector:

| Receiver Pin | PCB Connection |
|--------------|----------------|
| 5V (VCC) | 5V |
| GND | GND |
| CH1 (Steering) | Steering Signal Pin |
| CH2 (Throttle) | Throttle Signal Pin |

### AS5600 Encoder Wiring

Connect the AS5600 magnetic encoder module for steering position feedback according to the PCB pinout.

### Shifter Wiring

**If your Power Wheels has an existing shifter:**
- Connect the shifter contacts to the PCB using spade connectors

**If you need to build a shifter:**
- Use 2x 3-way rocker switches
- Wire to provide Forward/Neutral/Reverse selection

### Throttle Pedal

Connect the foot pedal potentiometer:
- VCC → 5V
- GND → Ground  
- Signal → Throttle ADC input

### Steering Wheel Potentiometer

Connect the waterproof potentiometer on the steering wheel:
- VCC → 5V
- GND → Ground
- Signal → Steering ADC input

---

## First Power-Up

### Pre-Power Checklist

- [ ] All wiring double-checked
- [ ] Fuse installed (30-40A)
- [ ] Both Teensy and ESP32 are flashed and installed
- [ ] BTS7960 modules connected (power and signal)
- [ ] FlySky receiver connected
- [ ] Motors connected to BTS7960 outputs
- [ ] No loose wires or short circuits

### Power On Sequence

1. Ensure the FlySky transmitter is **OFF**
2. Apply **24-27V** power to the system
3. Wait for the ESP32 to boot and connect to WiFi
4. Open Serial Monitor at **115200 baud** to verify IP address (if needed)
5. Access the web interface via the IP address in your browser
6. Turn on the FlySky transmitter
7. Verify communication in the web interface
8. Test each function at low power before full operation

### Troubleshooting

| Issue | Solution |
|-------|----------|
| No IP address displayed | Check WiFi credentials in `secrets.h` |
| Web server not loading | Ensure device is on same network, check IP address |
| Motors not responding | Check BTS7960 wiring, verify 8-pin signal connections |
| No RC signal | Verify FlySky receiver wiring, check binding |
| System not powering on | Check fuse, verify voltage (24-27V) |

---

## Safety Warnings

⚠️ **CAUTION:** This system operates at high currents. Always:
- Use appropriately rated wire (16 AWG minimum for power)
- Install the fuse before first power-up
- Double-check polarity before connecting power
- Supervise children at all times during operation
- Test in a safe, open area

---

## Support

For questions, issues, or contributions, please open an issue on the GitHub repository.

**Happy Building!** 🚗⚡
