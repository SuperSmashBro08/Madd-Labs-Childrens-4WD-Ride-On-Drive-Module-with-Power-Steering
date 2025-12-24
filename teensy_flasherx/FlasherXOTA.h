//******************************************************************************
// FlasherXOTA.h -- OTA Firmware Update Handler for Teensy 4.1
//******************************************************************************
// Encapsulates all FlasherX OTA functionality for clean separation from
// main application code. Handles ESP32 communication protocol and firmware
// update process.
//******************************************************************************
#ifndef FLASHERXOTA_H_
#define FLASHERXOTA_H_

#include <Arduino.h>

//******************************************************************************
// Protocol Constants
//******************************************************************************
#define CMD_PING     0x03
#define CMD_VERSION  0x04
#define CMD_UPDATE   0x05
#define PROTO_ACK    0x06
#define PROTO_NAK    0x15

//******************************************************************************
// FlasherXOTA Class
//******************************************************************************
class FlasherXOTA {
public:
    FlasherXOTA();
    
    // Initialize with serial streams
    // flashSerial: Serial port connected to ESP32 for firmware transfer (e.g., Serial2)
    // debugSerial: Serial port for debug output to ESP32 (e.g., Serial3)
    // usbSerial:   USB Serial for local debugging (e.g., Serial)
    void begin(Stream* flashSerial, Stream* debugSerial, Stream* usbSerial,
               const char* firmwareVersion, const char* buildDate);
    
    // Call this in loop() to check for OTA commands from ESP32
    void checkForCommands();
    
    // Check if OTA system is ready
    bool isReady() const { return _bufferReady; }
    
    // Get buffer info
    uint32_t getBufferAddress() const { return _bufferAddr; }
    uint32_t getBufferSize() const { return _bufferSize; }
    
    // Manually trigger firmware update (e.g., from USB command)
    void startUpdate();
    
    // Print status information
    void printStatus();

private:
    Stream* _flashSerial;
    Stream* _debugSerial;
    Stream* _usbSerial;
    
    const char* _firmwareVersion;
    const char* _buildDate;
    
    uint32_t _bufferAddr;
    uint32_t _bufferSize;
    bool _bufferReady;
    
    void handleCommand(uint8_t cmd);
    bool initBuffer();
};

// Global instance for easy access
extern FlasherXOTA OTA;

#endif
