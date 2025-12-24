# ESP32-S3 Teensy 4.1 FlasherX Web Interface

A web-based firmware upload system for programming Teensy 4.1 via an ESP32-S3 using the FlasherX protocol.

## Features

- **Teensy 4.1 Flashing**: Upload .hex/.bin firmware files via FlasherX protocol
- **ESP32 OTA Updates**: Update the ESP32 firmware over-the-air
- **Web Interface**: Modern responsive UI at http://192.168.1.29
- **Upload Progress Bar**: Real-time progress indication
- **Error Reporting**: Clear error messages for troubleshooting
- **Serial Monitor**: Live serial output from both ESP32 and Teensy via WebSocket

## Hardware Requirements

- ESP32-S3 DevKit
- Teensy 4.1
- Custom PCB with pre-wired UART connections

## Custom PCB Wiring

### Flashing UART (ESP32 UART0 ↔ Teensy Serial2)
Used for firmware upload via FlasherX protocol.

| ESP32-S3 Pin | Teensy 4.1 Pin | Function |
|--------------|----------------|----------|
| GPIO 43 (TX0) | RX2 (Pin 7) | ESP32 TX → Teensy RX |
| GPIO 44 (RX0) | TX2 (Pin 8) | ESP32 RX ← Teensy TX |

### Debug UART (ESP32 UART1 ↔ Teensy Serial3)
Used for serial monitoring in web interface.

| ESP32-S3 Pin | Teensy 4.1 Pin | Function |
|--------------|----------------|----------|
| GPIO 17 | RX3 (Pin 15) | ESP32 TX → Teensy RX |
| GPIO 18 | TX3 (Pin 14) | ESP32 RX ← Teensy TX |

### Control Pins
| ESP32-S3 Pin | Teensy 4.1 Pin | Function |
|--------------|----------------|----------|
| GPIO 4 | Reset | Reset control |
| GPIO 5 | Program | Program mode |
| GND | GND | Ground |

## Configuration

Edit the following in `esp32_flasher.ino`:

```cpp
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
```

## Arduino IDE Setup

### Board Manager
1. Add ESP32 board URL: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
2. Install "ESP32 by Espressif Systems"

### Library Dependencies
Install via Library Manager:
- **ArduinoJson** by Benoit Blanchon (v6.x)
- **WebSockets** by Markus Sattler

### Board Settings
- Board: "ESP32S3 Dev Module"
- USB Mode: "USB-OTG (TinyUSB)"
- Partition Scheme: "Default 4MB with spiffs"
- Upload Speed: 921600

## Usage

1. Upload this sketch to ESP32-S3
2. Open Serial Monitor (115200 baud) to see connection status
3. Navigate to http://192.168.1.29 in your browser
4. Use the web interface to:
   - Upload Teensy firmware (.hex or .bin files)
   - Update ESP32 firmware via OTA (.bin files)
   - Monitor serial output from both devices
   - Reset Teensy remotely

## FlasherX Protocol

The Teensy must have the FlasherX receiver sketch loaded. The protocol uses:
- **Flash Serial (Serial2)**: Firmware data transfer
- **Debug Serial (Serial3)**: Status messages to web monitor

Protocol commands:
- Sync: Send 0x55 bytes, receive ACK (0x06)
- Block: Send address + data + checksum, receive ACK
- Finish: Send finish command, receive ACK

## Troubleshooting

| Issue | Solution |
|-------|----------|
| WiFi won't connect | Check SSID/password, verify 2.4GHz network |
| FlasherX sync fails | Verify wiring, check Teensy has receiver sketch |
| Upload progress stuck | Check serial connection speed (115200 baud) |
| WebSocket disconnects | Refresh page, check network stability |

## LED Indicators (Web UI)

- 🟢 Green: Connected/Success
- 🔴 Red: Disconnected/Error
- 🔵 Blue: In Progress

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Web interface |
| `/status` | GET | System status JSON |
| `/teensy/upload` | POST | Upload Teensy firmware |
| `/teensy/reset` | POST | Reset Teensy |
| `/esp/ota` | POST | ESP32 OTA update |

WebSocket on port 81 for real-time serial monitoring.
