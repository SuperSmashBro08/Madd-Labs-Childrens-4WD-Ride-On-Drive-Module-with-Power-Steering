# Teensy 4.1 FlasherX Receiver

This sketch runs on the Teensy 4.1 to receive firmware updates from the ESP32-S3 FlasherX web interface.

## Prerequisites

### Install FlasherX Library
1. Download from: https://github.com/joepasquariello/FlasherX
2. In Arduino IDE: Sketch → Include Library → Add .ZIP Library
3. Select the downloaded FlasherX-master.zip

## Hardware Setup

Connect Teensy 4.1 to ESP32-S3 (Custom PCB Configuration):

### Flashing UART (Firmware Upload)
| Teensy Pin | ESP32-S3 Pin | Function |
|------------|--------------|----------|
| TX2 (Pin 8) | GPIO 44 (RX0) | Teensy TX → ESP32 RX |
| RX2 (Pin 7) | GPIO 43 (TX0) | Teensy RX ← ESP32 TX |

### Debug UART (Serial Monitor)
| Teensy Pin | ESP32-S3 Pin | Function |
|------------|--------------|----------|
| TX3 (Pin 14) | GPIO 18 | Teensy TX → ESP32 RX |
| RX3 (Pin 15) | GPIO 17 | Teensy RX ← ESP32 TX |

### Control Pins
| Teensy Pin | ESP32-S3 Pin | Function |
|------------|--------------|----------|
| Program | GPIO 5 | Program mode |
| Reset | GPIO 4 | Reset control |
| GND | GND | Ground |

## Arduino IDE Settings

- Board: "Teensy 4.1"
- USB Type: "Serial"
- CPU Speed: "600 MHz"
- Port: Select Teensy port

## How It Works

1. ESP32 toggles Program pin to enter bootloader mode
2. ESP32 sends sync bytes (0x55 x 3)
3. Teensy responds with ACK (0x06)
4. ESP32 sends firmware blocks with address/data/checksum
5. Teensy writes blocks to flash buffer
6. ESP32 sends finish command
7. Teensy applies firmware and reboots

## Protocol Details

### Sync Sequence
- ESP32 sends: 0x55 0x55 0x55
- Teensy responds: 0x06 (ACK) or 0x15 (NAK)

### Block Write (CMD 0x01)
| Byte(s) | Description |
|---------|-------------|
| 1 | Command (0x01) |
| 4 | Address (big-endian) |
| 2 | Length (big-endian) |
| N | Data bytes |
| 1 | Checksum (sum of data bytes) |

### Finish (CMD 0x02)
| Byte(s) | Description |
|---------|-------------|
| 1 | Command (0x02) |

## Testing

1. Upload this sketch to Teensy 4.1
2. Open Serial Monitor (115200 baud)
3. Verify "FlasherX library detected" message
4. Test with ESP32 web interface

## Troubleshooting

| Issue | Solution |
|-------|----------|
| "FlasherX not installed" | Download and install the library |
| Sync fails | Check wiring, especially TX/RX cross-connection |
| Checksum errors | Reduce baud rate or check for noise |
| Flash write fails | Ensure Teensy has enough free flash |

## Notes

- The FlasherX library allows Teensy to reprogram itself
- Firmware is first written to a buffer, then copied to flash
- If update fails mid-way, Teensy keeps running current firmware
- Always test with non-critical firmware first
