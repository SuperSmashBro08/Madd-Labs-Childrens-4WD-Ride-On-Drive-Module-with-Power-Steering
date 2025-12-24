//******************************************************************************
// FlasherXOTA.cpp -- OTA Firmware Update Handler for Teensy 4.1
//******************************************************************************
// Encapsulates all FlasherX OTA functionality for clean separation from
// main application code.
//******************************************************************************
#include "FlasherXOTA.h"
extern "C" {
  #include "FlashTxx.h"
}
#include "FXUtil.h"

// Global instance
FlasherXOTA OTA;

//******************************************************************************
// Constructor
//******************************************************************************
FlasherXOTA::FlasherXOTA() 
    : _flashSerial(nullptr)
    , _debugSerial(nullptr)
    , _usbSerial(nullptr)
    , _firmwareVersion("0.0.0")
    , _buildDate("Unknown")
    , _bufferAddr(0)
    , _bufferSize(0)
    , _bufferReady(false)
{
}

//******************************************************************************
// begin() - Initialize OTA system
//******************************************************************************
void FlasherXOTA::begin(Stream* flashSerial, Stream* debugSerial, Stream* usbSerial,
                        const char* firmwareVersion, const char* buildDate)
{
    _flashSerial = flashSerial;
    _debugSerial = debugSerial;
    _usbSerial = usbSerial;
    _firmwareVersion = firmwareVersion;
    _buildDate = buildDate;
    
    // Initialize flash buffer
    _bufferReady = initBuffer();
    
    if (_usbSerial) {
        _usbSerial->println("\n=== FlasherX OTA System Initialized ===");
        if (_bufferReady) {
            _usbSerial->printf("Flash buffer ready: addr=0x%08lX, size=%lu bytes\n", 
                              _bufferAddr, _bufferSize);
        } else {
            _usbSerial->println("WARNING: Flash buffer init failed!");
        }
    }
    
    if (_debugSerial) {
        _debugSerial->println(_bufferReady ? "FlasherX ready" : "FlasherX init failed");
    }
}

//******************************************************************************
// initBuffer() - Initialize flash buffer
//******************************************************************************
bool FlasherXOTA::initBuffer()
{
    int bufferType = firmware_buffer_init(&_bufferAddr, &_bufferSize);
    return (bufferType != NO_BUFFER_TYPE);
}

//******************************************************************************
// checkForCommands() - Check for OTA commands from ESP32
//******************************************************************************
void FlasherXOTA::checkForCommands()
{
    if (_flashSerial && _flashSerial->available()) {
        uint8_t cmd = _flashSerial->read();
        handleCommand(cmd);
    }
}

//******************************************************************************
// handleCommand() - Process OTA command
//******************************************************************************
void FlasherXOTA::handleCommand(uint8_t cmd)
{
    static bool firstPing = true;
    static bool firstVersion = true;
    
    switch (cmd) {
        case CMD_PING:
            if (_flashSerial) _flashSerial->write(PROTO_ACK);
            if (firstPing) {
                if (_usbSerial) _usbSerial->println("[OTA] ESP32 connected");
                firstPing = false;
            }
            break;
            
        case CMD_VERSION:
            if (_flashSerial) {
                _flashSerial->print("VER:");
                _flashSerial->println(_firmwareVersion);
            }
            if (firstVersion) {
                if (_usbSerial) _usbSerial->println("[OTA] Version sent to ESP32");
                firstVersion = false;
            }
            break;
            
        case CMD_UPDATE:
            if (_flashSerial) _flashSerial->write(PROTO_ACK);
            if (_usbSerial) _usbSerial->println("[OTA] Update command received");
            if (_debugSerial) _debugSerial->println("Starting update...");
            startUpdate();
            break;
            
        default:
            // Ignore other bytes (could be stray data or hex content)
            break;
    }
}

//******************************************************************************
// startUpdate() - Begin firmware update process
//******************************************************************************
void FlasherXOTA::startUpdate()
{
    if (_usbSerial) {
        _usbSerial->println("\n=== Starting Firmware Update ===");
        _usbSerial->println("Send Intel HEX file via Flash Serial...");
    }
    if (_debugSerial) {
        _debugSerial->println("Waiting for HEX...");
    }
    
    // Re-init buffer
    if (!initBuffer()) {
        if (_usbSerial) {
            _usbSerial->println("ERROR: Buffer init failed");
            _usbSerial->printf("  buffer_addr=0x%08lX, buffer_size=%lu\n", 
                              _bufferAddr, _bufferSize);
        }
        if (_debugSerial) _debugSerial->println("ERROR: Buffer fail");
        if (_flashSerial) _flashSerial->write(PROTO_NAK);
        return;
    }
    
    if (_usbSerial) {
        _usbSerial->printf("Buffer ready: addr=0x%08lX size=%lu\n", 
                          _bufferAddr, _bufferSize);
    }
    
    // Use FlasherX update_firmware function
    update_firmware(_flashSerial, _usbSerial, _bufferAddr, _bufferSize);
    
    // If we get here, update failed or was aborted
    if (_usbSerial) {
        _usbSerial->println("Update function returned - check for errors above");
    }
    if (_debugSerial) {
        _debugSerial->println("Update ended");
    }
}

//******************************************************************************
// printStatus() - Print OTA system status
//******************************************************************************
void FlasherXOTA::printStatus()
{
    if (_usbSerial) {
        _usbSerial->println("\n=== FlasherX OTA Status ===");
        _usbSerial->printf("Firmware Version: %s\n", _firmwareVersion);
        _usbSerial->printf("Build Date: %s\n", _buildDate);
        _usbSerial->printf("Buffer Ready: %s\n", _bufferReady ? "Yes" : "No");
        if (_bufferReady) {
            _usbSerial->printf("Buffer Address: 0x%08lX\n", _bufferAddr);
            _usbSerial->printf("Buffer Size: %lu bytes\n", _bufferSize);
        }
    }
}
