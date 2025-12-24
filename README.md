# Kids Ride-On Framework

[![Open Source](https://img.shields.io/badge/Open%20Source-Yes-brightgreen)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-ESP32--S3%20%2B%20Teensy%204.1-blue)]()
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

An open-source framework for building smart, connected kids' ride-on vehicles with OTA (Over-The-Air) firmware updates, web-based control, and modular peripheral support.

---

## 🎯 Project Overview

This project provides a complete hardware and software framework for upgrading kids' electric ride-on toys with:

- **Dual-MCU Architecture**: ESP32-S3 for WiFi/Web + Teensy 4.1 for real-time control
- **OTA Firmware Updates**: Update both MCUs wirelessly via web browser
- **Web Dashboard**: Monitor and control the vehicle from any device
- **Modular Design**: Easy to add motors, LEDs, sensors, and more
- **Custom PCB**: Purpose-built circuit board for clean integration

---

## 📁 Project Structure

```
Kids-Ride-On-Framework/
├── esp32_flasher/           # ESP32-S3 firmware (WiFi, Web Server, OTA)
│   ├── esp32_flasher.ino    # Main ESP32 code
│   ├── secrets.h.example    # Template for WiFi credentials
│   └── README.md            # ESP32-specific documentation
│
├── teensy_flasherx/         # Teensy 4.1 firmware (Motor control, sensors)
│   ├── teensy_flasherx.ino  # Main application code (YOUR CODE HERE)
│   ├── FlasherXOTA.h/cpp    # OTA update handler (don't modify)
│   ├── FXUtil.h/cpp         # Intel HEX parser (don't modify)
│   ├── FlashTxx.h/c         # Flash primitives (don't modify)
│   └── README.md            # Teensy-specific documentation
│
├── hardware/                # Hardware design files
│   └── PCB/
│       └── Kids Ride On 2.1/
│           ├── Kicad files/ # KiCad schematic and PCB
│           ├── bom/         # Bill of materials
│           ├── 3d/          # 3D models
│           └── Production/  # Gerber files for manufacturing
│
├── Backup/                  # Backup of original firmware
├── readme files/            # Additional documentation
├── .gitignore               # Git ignore rules
└── README.md                # This file
```

---

## 🚀 Getting Started

### Prerequisites

- [Arduino IDE 2.x](https://www.arduino.cc/en/software) or [PlatformIO](https://platformio.org/)
- [Teensyduino](https://www.pjrc.com/teensy/teensyduino.html) add-on for Arduino IDE
- ESP32 Arduino Core (install via Board Manager)

### Hardware Required

- ESP32-S3 DevKitC (or compatible)
- Teensy 4.1
- Custom PCB (see `hardware/PCB/`) or breadboard setup
- 12V/24V power supply (depending on your motors)
- Motors, sensors, LEDs as needed

### Quick Start

1. **Clone the repository**
   ```bash
   git clone https://github.com/yourusername/kids-ride-on-framework.git
   cd kids-ride-on-framework
   ```

2. **Configure WiFi credentials**
   ```bash
   cd esp32_flasher
   cp secrets.h.example secrets.h
   # Edit secrets.h with your WiFi SSID and password
   ```

3. **Upload ESP32 firmware**
   - Open `esp32_flasher/esp32_flasher.ino` in Arduino IDE
   - Select board: ESP32-S3 Dev Module
   - Upload via USB
   - Open Serial Monitor to see the assigned IP address

4. **Upload Teensy firmware**
   - Open `teensy_flasherx/teensy_flasherx.ino` in Arduino IDE
   - Select board: Teensy 4.1
   - Upload via USB (first time only - then use OTA!)

5. **Access the Web Dashboard**
   - Open your browser to the IP address shown in Serial Monitor
   - You can now upload firmware updates wirelessly!

---

## ⚙️ Configuration

### Sensor Calibration (IMPORTANT!)

Every vehicle will have different sensor ranges. Open `teensy_flasherx/teensy_flasherx.ino` and find the **USER CONFIGURATION SECTION** near the top:

```cpp
//******************************************************************************
//                    USER CONFIGURATION SECTION
//******************************************************************************

// ----- STEERING 1 (Primary/Front) -----
#define STEERING1_PIN         A0      // Analog input pin
#define STEERING1_MIN         0       // Raw ADC at full LEFT
#define STEERING1_MAX         1023    // Raw ADC at full RIGHT
#define STEERING1_CENTER      512     // Raw ADC at CENTER

// ----- THROTTLE 1 (Primary) -----
#define THROTTLE1_PIN         A2      // Analog input pin
#define THROTTLE1_MIN         100     // Raw ADC at NO throttle
#define THROTTLE1_MAX         900     // Raw ADC at FULL throttle

// ... more configuration options
```

**How to calibrate:**
1. Upload the firmware with default values
2. Open Serial Monitor
3. Move your steering/throttle and note the raw ADC values
4. Update the MIN/MAX/CENTER values in the code
5. Re-upload (via OTA!)

### WiFi Configuration

For security, WiFi credentials are stored in `secrets.h` which is **not committed to git**.

1. Copy the example file:
   ```bash
   cp esp32_flasher/secrets.h.example esp32_flasher/secrets.h
   ```

2. Edit `secrets.h` with your credentials:
   ```cpp
   #define WIFI_SSID     "YourWiFiName"
   #define WIFI_PASSWORD "YourWiFiPassword"
   ```

---

## 🔧 Hardware

### PCB Design

The custom PCB connects the ESP32-S3 and Teensy 4.1 with proper level shifting and power regulation. Design files are in `hardware/PCB/Kids Ride On 2.1/`.

### Complete Pinout (V2.1 PCB)

#### Teensy 4.1 Pinout

**Analog Inputs (0–3.3V)**
| Pin | Function | Description |
|-----|----------|-------------|
| 17 (A3) | S1_WIPER | Steering Potentiometer |
| 23 (A9) | T1_WIPER | Throttle Potentiometer |
| 16 (A2) | AS5600_OUT | Magnetic Encoder Analog |
| 24 (A10) | STEER_I_L | Steering Motor Current Left |
| 25 (A11) | STEER_I_R | Steering Motor Current Right |
| 26 (A12) | FRONT_I_L | Front Drive Current Left |
| 27 (A13) | FRONT_I_R | Front Drive Current Right |
| 38 (A14) | REAR_I_L | Rear Drive Current Left |
| 39 (A15) | REAR_I_R | Rear Drive Current Right |

**Digital Inputs (3.3V Logic)**
| Pin | Function | Description |
|-----|----------|-------------|
| 28 | SHIFT_FWD | Shifter Forward |
| 29 | SHIFT_REV | Shifter Reverse |
| 30 | MODE_SEL | Mode Select |
| 31 | E_BRAKE | E-Brake Input |

**PWM Outputs (3.3V)**
| Pin | Function | Description |
|-----|----------|-------------|
| 2 | STEER_LPWM | Steering Left PWM |
| 3 | STEER_RPWM | Steering Right PWM |
| 6 | FRONT_LPWM | Front Drive Left PWM |
| 9 | FRONT_RPWM | Front Drive Right PWM |
| 12 | REAR_LPWM | Rear Drive Left PWM |
| 13 | REAR_RPWM | Rear Drive Right PWM |

**Motor Driver Enable (3.3V → 5V via level shifter)**
| Pin | Function | Description |
|-----|----------|-------------|
| 4 | STEER_L_EN | Steering Left Enable |
| 5 | STEER_R_EN | Steering Right Enable |
| 10 | FRONT_L_EN | Front Drive Left Enable |
| 11 | FRONT_R_EN | Front Drive Right Enable |
| 32 | REAR_L_EN | Rear Drive Left Enable |
| 33 | REAR_R_EN | Rear Drive Right Enable |

**I2C (AS5600 Encoder)**
| Pin | Function |
|-----|----------|
| 18 | SDA |
| 19 | SCL |

**UART to ESP32-S3**
| Teensy Pin | ESP32 Pin | Function |
|------------|-----------|----------|
| 7 (RX2) | GPIO43 (TX) | OTA Flash Data |
| 8 (TX2) | GPIO44 (RX) | OTA Flash Data |
| 14 (TX3) | GPIO18 (RX) | Debug Serial |
| 15 (RX3) | GPIO17 (TX) | Debug Serial |

#### ESP32-S3 Pinout

| GPIO | Function | Description |
|------|----------|-------------|
| 43 | UART TX | TX to Teensy RX2 (Pin 7) |
| 44 | UART RX | RX from Teensy TX2 (Pin 8) |
| 17 | UART TX | TX to Teensy RX3 (Pin 15) |
| 18 | UART RX | RX from Teensy TX3 (Pin 14) |
| 38 | RGB_LED | Status LED / Fiber Optic |

### Bill of Materials

See `hardware/PCB/Kids Ride On 2.1/bom/` for the complete parts list.

---

## 🌐 Web Interface

The ESP32 hosts a web server with:

- **System Status**: Connection status for both MCUs
- **Teensy OTA**: Upload `.hex` files to flash Teensy wirelessly
- **ESP32 OTA**: Upload `.bin` files to update ESP32 wirelessly
- **Serial Monitor**: Real-time serial output from both MCUs

Access via: `http://<ESP32-IP-ADDRESS>/`

---

## 📡 OTA Updates

### Updating Teensy

1. Compile your Teensy code in Arduino IDE
2. Find the `.hex` file in the build folder
3. Open the web dashboard
4. Drag & drop the `.hex` file onto "Flash Teensy 4.1"
5. Wait for completion - Teensy will reboot automatically

### Updating ESP32

1. Export compiled binary from Arduino IDE (Sketch → Export Compiled Binary)
2. Open the web dashboard
3. Drag & drop the `.bin` file onto "ESP32 OTA Update"
4. Wait for completion - ESP32 will reboot automatically

---

## 🛠️ Development

### Adding New Features

All your custom code goes in `teensy_flasherx/teensy_flasherx.ino`:

```cpp
void initPeripherals() {
    // Initialize your motors, LEDs, sensors here
}

void updatePeripherals() {
    // Your main control loop - runs every iteration
    // Read sensors, update motors, etc.
}
```

The FlasherX OTA code is encapsulated and doesn't need modification.

### File Overview

| File | Modify? | Purpose |
|------|---------|---------|
| `teensy_flasherx.ino` | ✅ YES | Your application code |
| `FlasherXOTA.h/cpp` | ❌ No | OTA update handler |
| `FXUtil.h/cpp` | ❌ No | Intel HEX parser |
| `FlashTxx.h/c` | ❌ No | Low-level flash access |
| `esp32_flasher.ino` | ⚠️ Rarely | Web server & OTA |

---

## 📋 Roadmap

- [x] OTA firmware updates for Teensy 4.1
- [x] OTA firmware updates for ESP32
- [x] Web-based dashboard
- [x] Serial monitor via WebSocket
- [ ] Motor control with acceleration ramping
- [ ] LED strip animations (NeoPixel/WS2812)
- [ ] Steering servo control
- [ ] Speed limiting / parental controls
- [ ] Bluetooth gamepad support
- [ ] Battery monitoring
- [ ] GPS tracking (optional)
- [ ] Mobile app

---

## 🤝 Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

---

## 📄 License

This project is open source and available under the [MIT License](LICENSE).

---

## ⚠️ Safety Warning

This project involves modifying children's toys with higher-powered electronics. Please:

- Always supervise children when using modified vehicles
- Install appropriate speed limiters
- Use proper fusing and circuit protection
- Test thoroughly in a safe environment
- Follow local regulations for motorized vehicles

---

## 🙏 Acknowledgments

- [FlasherX](https://github.com/joepasquariello/FlasherX) by Joe Pasquariello - Teensy OTA library
- [PJRC](https://www.pjrc.com/) - Teensy development
- [Espressif](https://www.espressif.com/) - ESP32 platform

---

**Happy Building! 🚗💨**
