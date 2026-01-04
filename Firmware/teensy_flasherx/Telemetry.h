//******************************************************************************
// Telemetry.h - High-Speed Serial Monitoring for Kids Ride-On
//******************************************************************************
// Provides fixed-width formatted output for real-time monitoring via ESP32.
// Output is designed to align perfectly for easy reading.
//******************************************************************************
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>

//******************************************************************************
// Telemetry Configuration
//******************************************************************************
#define TELEMETRY_BAUD        115200    // Match ESP32 debug serial
#define TELEMETRY_INTERVAL_MS 50        // Update rate (50ms = 20Hz)
#define TELEMETRY_HEADER_INTERVAL 20    // Print header every N updates

//******************************************************************************
// Telemetry Data Structure
//******************************************************************************
struct TelemetryData {
    // Inputs - Throttle1 (steering wheel + pedal)
    int16_t steeringRaw;      // Raw steering pot value (0-1023)
    int16_t throttleRaw;      // Raw throttle pot value (0-1023)
    uint16_t encoderAngle;    // AS5600 angle in degrees (0-359)
    
    // Inputs - RC (steer2 + throttle2)
    int16_t steer2Raw;        // Raw RC steering value (0-1023)
    int16_t throttle2Raw;     // Raw RC throttle value (0-1023)
    
    // Processed inputs (-100 to +100 or 0-100)
    int8_t steeringPercent;   // -100 (left) to +100 (right)
    int8_t throttlePercent;   // 0 to 100
    int8_t steer2Percent;     // -100 (left) to +100 (right) - RC
    int8_t throttle2Percent;  // 0 to 100 - RC
    
    // Digital inputs
    bool shifterFwd;          // Forward gear
    bool shifterRev;          // Reverse gear
    bool modeSelect;          // Mode switch state
    bool eBrake;              // E-brake engaged
    
    // Control source
    uint8_t controlSource;    // 1=Car (steering wheel), 2=Remote (RC receiver)
    
    // Motor outputs (0-255 PWM)
    uint8_t steerPwmL;        // Steering left PWM
    uint8_t steerPwmR;        // Steering right PWM
    uint8_t frontPwmL;        // Front drive left PWM
    uint8_t frontPwmR;        // Front drive right PWM
    uint8_t rearPwmL;         // Rear drive left PWM
    uint8_t rearPwmR;         // Rear drive right PWM
    
    // Current sensing (mA)
    uint16_t steerCurrentL;   // Steering motor current left
    uint16_t steerCurrentR;   // Steering motor current right
    uint16_t frontCurrentL;   // Front drive current left
    uint16_t frontCurrentR;   // Front drive current right
    uint16_t rearCurrentL;    // Rear drive current left
    uint16_t rearCurrentR;    // Rear drive current right
    
    // System
    uint32_t loopTimeUs;      // Loop execution time in microseconds
    uint8_t cpuTemp;          // CPU temperature (Teensy 4.1)
};

//******************************************************************************
// Telemetry Class
//******************************************************************************
class Telemetry {
public:
    Telemetry();
    
    // Initialize telemetry system
    void begin(Stream* telemSerial, Stream* debugSerial = nullptr);
    
    // Call this every loop - handles timing internally
    void update();
    
    // Enable/disable telemetry output
    void enable(bool enabled) { _enabled = enabled; }
    bool isEnabled() const { return _enabled; }
    
    // Set update interval (ms)
    void setInterval(uint16_t intervalMs) { _intervalMs = intervalMs; }
    
    // Access data for modification
    TelemetryData& data() { return _data; }
    
    // Force send now (bypasses interval)
    void sendNow();
    
    // Print column header
    void printHeader();
    
    // Compact mode (single line) vs verbose mode
    void setCompactMode(bool compact) { _compactMode = compact; }

private:
    Stream* _telemSerial;
    Stream* _debugSerial;
    TelemetryData _data;
    
    bool _enabled;
    bool _compactMode;
    uint16_t _intervalMs;
    uint32_t _lastSendTime;
    uint16_t _updateCount;
    uint16_t _headerInterval;
    
    void sendCompact();
    void sendVerbose();
    
    // Helper for fixed-width integer formatting
    void printInt(int32_t value, uint8_t width);
    void printUInt(uint32_t value, uint8_t width);
};

// Global telemetry instance
extern Telemetry Telem;

#endif
