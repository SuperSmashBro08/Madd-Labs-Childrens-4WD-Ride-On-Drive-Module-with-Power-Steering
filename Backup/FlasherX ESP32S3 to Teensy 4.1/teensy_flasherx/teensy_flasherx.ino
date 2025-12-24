/*
 * Teensy 4.1 FlasherX Receiver
 * 
 * Receives Intel HEX firmware via Serial2 from ESP32 and programs flash.
 * Uses FlasherX library for self-programming.
 */

#include <Arduino.h>
extern "C" {
  #include "FlashTxx.h"
}
#include "FXUtil.h"

// Version Information
#define TEENSY_FIRMWARE_VERSION "1.2.1"
#define TEENSY_BUILD_DATE __DATE__ " " __TIME__

// Custom PCB Serial Configuration
#define FLASH_SERIAL Serial2   // Pin 7 RX, Pin 8 TX -> ESP32 GPIO43/44
#define DEBUG_SERIAL Serial3   // Pin 15 RX, Pin 14 TX -> ESP32 GPIO17/18
#define USB_SERIAL   Serial    // USB for Arduino IDE

// Protocol constants
#define CMD_PING     0x03
#define CMD_VERSION  0x04
#define CMD_UPDATE   0x05
#define PROTO_ACK    0x06
#define PROTO_NAK    0x15

// FlasherX buffer
uint32_t buffer_addr = 0;
uint32_t buffer_size = 0;

void setup() {
    USB_SERIAL.begin(115200);
    delay(1000);
    
    USB_SERIAL.println("\n=== Teensy 4.1 FlasherX Receiver ===");
    USB_SERIAL.printf("Firmware Version: %s\n", TEENSY_FIRMWARE_VERSION);
    USB_SERIAL.printf("Build Date: %s\n", TEENSY_BUILD_DATE);
    USB_SERIAL.println("Flash Serial2: Pin 7(RX) / Pin 8(TX) -> ESP32");
    USB_SERIAL.println("Debug Serial3: Pin 15(RX) / Pin 14(TX) -> ESP32");
    
    FLASH_SERIAL.begin(115200);
    DEBUG_SERIAL.begin(115200);
    
    DEBUG_SERIAL.printf("Teensy v%s started\n", TEENSY_FIRMWARE_VERSION);
    
    // Initialize flash buffer
    int bufferType = firmware_buffer_init(&buffer_addr, &buffer_size);
    if (bufferType != NO_BUFFER_TYPE) {
        USB_SERIAL.printf("Flash buffer: type=%d addr=0x%08lX, size=%lu bytes\n", bufferType, buffer_addr, buffer_size);
        DEBUG_SERIAL.println("FlasherX ready");
    } else {
        USB_SERIAL.println("ERROR: Flash buffer init failed!");
        USB_SERIAL.printf("  buffer_addr=0x%08lX, buffer_size=%lu\n", buffer_addr, buffer_size);
        DEBUG_SERIAL.println("ERROR: Flash init failed");
    }
    
    USB_SERIAL.println("Ready - send 'update' to start firmware update");
    DEBUG_SERIAL.println("Teensy ready");
}

void loop() {
    // Check for commands from ESP32 on FLASH_SERIAL
    if (FLASH_SERIAL.available()) {
        uint8_t cmd = FLASH_SERIAL.read();
        handleCommand(cmd);
    }
    
    // Check for commands from USB serial (for testing)
    if (USB_SERIAL.available()) {
        String input = USB_SERIAL.readStringUntil('\n');
        input.trim();
        
        if (input == "update") {
            startUpdate();
        } else if (input == "version") {
            USB_SERIAL.printf("Version: %s\n", TEENSY_FIRMWARE_VERSION);
            USB_SERIAL.printf("Build: %s\n", TEENSY_BUILD_DATE);
        }
    }
    
    // Heartbeat
    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat > 10000) {
        USB_SERIAL.println("Teensy running...");
        DEBUG_SERIAL.println("heartbeat");
        lastHeartbeat = millis();
    }
}

void handleCommand(uint8_t cmd) {
    switch (cmd) {
        case CMD_PING:
            FLASH_SERIAL.write(PROTO_ACK);
            USB_SERIAL.println("Ping received");
            DEBUG_SERIAL.println("Ping OK");
            break;
            
        case CMD_VERSION:
            FLASH_SERIAL.print("VER:");
            FLASH_SERIAL.println(TEENSY_FIRMWARE_VERSION);
            USB_SERIAL.println("Version requested");
            DEBUG_SERIAL.println("Version sent");
            break;
            
        case CMD_UPDATE:
            FLASH_SERIAL.write(PROTO_ACK);
            USB_SERIAL.println("Update command received");
            DEBUG_SERIAL.println("Starting update...");
            startUpdate();
            break;
            
        default:
            // Ignore other bytes (could be stray data or hex content)
            break;
    }
}

void startUpdate() {
    USB_SERIAL.println("\n=== Starting Firmware Update ===");
    USB_SERIAL.println("Send Intel HEX file via Serial2...");
    DEBUG_SERIAL.println("Waiting for HEX...");
    
    // Re-init buffer - returns 0 for failure, 1 or 2 for success
    int bufferType = firmware_buffer_init(&buffer_addr, &buffer_size);
    if (bufferType == NO_BUFFER_TYPE) {
        USB_SERIAL.println("ERROR: Buffer init failed");
        USB_SERIAL.printf("  buffer_addr=0x%08lX, buffer_size=%lu\n", buffer_addr, buffer_size);
        DEBUG_SERIAL.println("ERROR: Buffer fail");
        FLASH_SERIAL.write(PROTO_NAK);
        return;
    }
    
    USB_SERIAL.printf("Buffer ready: type=%d addr=0x%08lX size=%lu\n", bufferType, buffer_addr, buffer_size);
    
    // Use FlasherX update_firmware function
    // It reads hex lines from FLASH_SERIAL and writes to DEBUG_SERIAL/USB_SERIAL for output
    update_firmware(&FLASH_SERIAL, &USB_SERIAL, buffer_addr, buffer_size);
    
    // If we get here, update failed or was aborted
    USB_SERIAL.println("Update function returned - check for errors above");
    DEBUG_SERIAL.println("Update ended");
}
