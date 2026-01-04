//******************************************************************************
// Telemetry.cpp - High-Speed Serial Monitoring for Kids Ride-On
//******************************************************************************
#include "Telemetry.h"

// Global instance
Telemetry Telem;

//******************************************************************************
// Constructor
//******************************************************************************
Telemetry::Telemetry()
    : _telemSerial(nullptr)
    , _debugSerial(nullptr)
    , _enabled(true)
    , _compactMode(true)
    , _intervalMs(TELEMETRY_INTERVAL_MS)
    , _lastSendTime(0)
    , _updateCount(0)
    , _headerInterval(TELEMETRY_HEADER_INTERVAL)
{
    // Initialize data structure to zeros
    memset(&_data, 0, sizeof(_data));
}

//******************************************************************************
// begin() - Initialize telemetry
//******************************************************************************
void Telemetry::begin(Stream* telemSerial, Stream* debugSerial) {
    _telemSerial = telemSerial;
    _debugSerial = debugSerial;
    _lastSendTime = millis();
    _updateCount = 0;
    
    if (_telemSerial) {
        _telemSerial->println();
        _telemSerial->println("=== Telemetry Started ===");
        printHeader();
    }
}

//******************************************************************************
// update() - Call every loop, sends at configured interval
//******************************************************************************
void Telemetry::update() {
    if (!_enabled || !_telemSerial) return;
    
    uint32_t now = millis();
    if (now - _lastSendTime >= _intervalMs) {
        _lastSendTime = now;
        
        // Print header periodically for readability
        if (_updateCount % _headerInterval == 0) {
            printHeader();
        }
        _updateCount++;
        
        if (_compactMode) {
            sendCompact();
        } else {
            sendVerbose();
        }
    }
}

//******************************************************************************
// sendNow() - Force immediate send
//******************************************************************************
void Telemetry::sendNow() {
    if (!_telemSerial) return;
    
    if (_compactMode) {
        sendCompact();
    } else {
        sendVerbose();
    }
}

//******************************************************************************
// printHeader() - Print column headers
//******************************************************************************
void Telemetry::printHeader() {
    if (!_telemSerial) return;
    
    if (_compactMode) {
        // Fixed-width header matching data columns
        // Each column has specific width for alignment
        _telemSerial->println();
        _telemSerial->println("|------- RAW INPUTS ------|--- PROCESSED (%) ---|-- PWM OUTPUTS --|---------- CURRENT (mA) ---------|SOURCE|-- SYS --|");
        _telemSerial->println("| Steer Throt Str2  Thr2  Encdr | Steer Throt Str2  Thr2  | StL  StR  FrL  FrR  ReL  ReR| StL   StR   FrL   FrR   ReL   ReR|      | Loop  T |");
        _telemSerial->println("|---------------------------|--------------------------|------------------------------|----------------------------------|------|---------|");
    }
}

//******************************************************************************
// sendCompact() - Single line fixed-width output
//******************************************************************************
void Telemetry::sendCompact() {
    // Format: Fixed width columns for perfect alignment
    // Steer(5) Throt(5) Steer2(5) Throt2(5) Encdr(5) | Steer%(5) Throt%(5) Steer2%(5) Throt2%(5) | PWM x6 | Current x6 | Loop Temp
    
    _telemSerial->print("| ");
    
    // Raw inputs (5 chars each, right-aligned)
    printInt(_data.steeringRaw, 5);
    printInt(_data.throttleRaw, 5);
    printInt(_data.steer2Raw, 5);
    printInt(_data.throttle2Raw, 5);
    printInt(_data.encoderAngle, 5);
    _telemSerial->print(" | ");
    
    // Processed (-100 to +100, 5 chars with sign)
    printInt(_data.steeringPercent, 5);
    _telemSerial->print("% ");
    printInt(_data.throttlePercent, 5);
    _telemSerial->print("% ");
    printInt(_data.steer2Percent, 5);
    _telemSerial->print("% ");
    printInt(_data.throttle2Percent, 5);
    _telemSerial->print("% | ");
    
    // PWM outputs (4 chars each, 0-255)
    printUInt(_data.steerPwmL, 4);
    _telemSerial->print(" ");
    printUInt(_data.steerPwmR, 4);
    _telemSerial->print(" ");
    printUInt(_data.frontPwmL, 4);
    _telemSerial->print(" ");
    printUInt(_data.frontPwmR, 4);
    _telemSerial->print(" ");
    printUInt(_data.rearPwmL, 4);
    _telemSerial->print(" ");
    printUInt(_data.rearPwmR, 4);
    _telemSerial->print(" | ");
    
    // Current (5 chars each, mA)
    printUInt(_data.steerCurrentL, 5);
    _telemSerial->print(" ");
    printUInt(_data.steerCurrentR, 5);
    _telemSerial->print(" ");
    printUInt(_data.frontCurrentL, 5);
    _telemSerial->print(" ");
    printUInt(_data.frontCurrentR, 5);
    _telemSerial->print(" ");
    printUInt(_data.rearCurrentL, 5);
    _telemSerial->print(" ");
    printUInt(_data.rearCurrentR, 5);
    _telemSerial->print(" | ");
    
    // Control Source
    _telemSerial->print(" ");
    if (_data.controlSource == 1) {
        _telemSerial->print("Car  ");
    } else if (_data.controlSource == 2) {
        _telemSerial->print("Remot");
    } else {
        _telemSerial->print("????");
    }
    _telemSerial->print(" | ");
    
    // System (loop time in us, temp)
    printUInt(_data.loopTimeUs, 5);
    _telemSerial->print(" ");
    printUInt(_data.cpuTemp, 2);
    _telemSerial->println(" |");
}

//******************************************************************************
// sendVerbose() - Multi-line detailed output
//******************************************************************************
void Telemetry::sendVerbose() {
    _telemSerial->println("----------------------------------------");
    
    _telemSerial->print("INPUTS:  Steer=");
    printInt(_data.steeringRaw, 5);
    _telemSerial->print("  Throttle=");
    printInt(_data.throttleRaw, 5);
    _telemSerial->print("  Encoder=");
    printInt(_data.encoderAngle, 5);
    _telemSerial->println();
    
    _telemSerial->print("CONTROL: Steer=");
    printInt(_data.steeringPercent, 4);
    _telemSerial->print("%  Throttle=");
    printInt(_data.throttlePercent, 3);
    _telemSerial->print("%  Gear=");
    if (_data.shifterFwd) _telemSerial->print("FWD");
    else if (_data.shifterRev) _telemSerial->print("REV");
    else _telemSerial->print("N  ");
    _telemSerial->print("  Mode=");
    _telemSerial->print(_data.modeSelect ? "ON " : "OFF");
    _telemSerial->print("  Brake=");
    _telemSerial->println(_data.eBrake ? "ON" : "OFF");
    
    _telemSerial->print("MOTORS:  SteerL=");
    printUInt(_data.steerPwmL, 3);
    _telemSerial->print("  SteerR=");
    printUInt(_data.steerPwmR, 3);
    _telemSerial->print("  FrontL=");
    printUInt(_data.frontPwmL, 3);
    _telemSerial->print("  FrontR=");
    printUInt(_data.frontPwmR, 3);
    _telemSerial->print("  RearL=");
    printUInt(_data.rearPwmL, 3);
    _telemSerial->print("  RearR=");
    printUInt(_data.rearPwmR, 3);
    _telemSerial->println();
    
    _telemSerial->print("CURRENT: SteerL=");
    printUInt(_data.steerCurrentL, 5);
    _telemSerial->print("mA  SteerR=");
    printUInt(_data.steerCurrentR, 5);
    _telemSerial->print("mA  FrontL=");
    printUInt(_data.frontCurrentL, 5);
    _telemSerial->print("mA  FrontR=");
    printUInt(_data.frontCurrentR, 5);
    _telemSerial->println("mA");
    
    _telemSerial->print("         RearL=");
    printUInt(_data.rearCurrentL, 5);
    _telemSerial->print("mA  RearR=");
    printUInt(_data.rearCurrentR, 5);
    _telemSerial->println("mA");
    
    _telemSerial->print("SYSTEM:  LoopTime=");
    printUInt(_data.loopTimeUs, 5);
    _telemSerial->print("us  CPUTemp=");
    printUInt(_data.cpuTemp, 2);
    _telemSerial->println("C");
}

//******************************************************************************
// printInt() - Print signed integer with fixed width, right-aligned
//******************************************************************************
void Telemetry::printInt(int32_t value, uint8_t width) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%*ld", width, value);
    _telemSerial->print(buf);
}

//******************************************************************************
// printUInt() - Print unsigned integer with fixed width, right-aligned
//******************************************************************************
void Telemetry::printUInt(uint32_t value, uint8_t width) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%*lu", width, value);
    _telemSerial->print(buf);
}
