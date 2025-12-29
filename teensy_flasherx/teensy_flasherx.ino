/*******************************************************************************
 * Teensy 4.1 Kids Ride-On Controller
 * 
 * Main application code for controlling ride-on vehicle peripherals.
 * OTA firmware updates handled by FlasherXOTA module.
 * 
 * Serial Configuration (Custom PCB):
 *   Serial  (USB)    - Arduino IDE / Debug Console
 *   Serial2 (7/8)    - ESP32 Flash Serial (OTA updates)
 *   Serial3 (15/14)  - ESP32 Debug Serial / Telemetry
 ******************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include "FlasherXOTA.h"
#include "Telemetry.h"

//******************************************************************************
// Version Information
//******************************************************************************
#define FIRMWARE_VERSION "1.3.8"
#define BUILD_DATE       __DATE__ " " __TIME__

//******************************************************************************
//                    USER CONFIGURATION SECTION
//******************************************************************************
// Calibrate these values to match YOUR specific sensors and hardware.
// Use the serial monitor to read raw values, then enter your min/max here.
//******************************************************************************

// ----- STEERING POT (S1_WIPER) -----
#define STEERING_POT_PIN      17      // Pin 17 (A3) - Steering potentiometer
#define STEERING_POT_MIN      105     // Raw ADC value at full LEFT
#define STEERING_POT_MAX      708     // Raw ADC value at full RIGHT
#define STEERING_POT_CENTER   407     // Raw ADC value at CENTER (neutral) = (105+708)/2
#define STEERING_POT_DEADZONE 20      // Ignore values within +/- this of center

// ----- THROTTLE POT (T1_WIPER) -----
#define THROTTLE_POT_PIN      23      // Pin 23 (A9) - Throttle potentiometer
#define THROTTLE_POT_MIN      249     // Raw ADC value at NO throttle (foot off)
#define THROTTLE_POT_MAX      790     // Raw ADC value at FULL throttle
#define THROTTLE_POT_DEADZONE 30      // Ignore small values (prevents creep)

// ----- AS5600 MAGNETIC ENCODER (I2C ONLY) -----
#define ENCODER_I2C_ADDR      0x36    // Default AS5600 I2C address
#define ENCODER_CPR           4096    // Counts per revolution (12-bit = 4096)

// ----- CURRENT SENSING -----
// All current sense pins output 0-3.3V proportional to motor current
#define STEER_CURRENT_L_PIN   24      // Pin 24 (A10) - Steering motor current Left
#define STEER_CURRENT_R_PIN   25      // Pin 25 (A11) - Steering motor current Right
#define FRONT_CURRENT_L_PIN   26      // Pin 26 (A12) - Front drive current Left
#define FRONT_CURRENT_R_PIN   27      // Pin 27 (A13) - Front drive current Right
#define REAR_CURRENT_L_PIN    38      // Pin 38 (A14) - Rear drive current Left
#define REAR_CURRENT_R_PIN    39      // Pin 39 (A15) - Rear drive current Right
#define CURRENT_SENSE_SCALE   1.0     // mV per Amp (adjust for your sensor)

// ----- MOTOR CONFIGURATION -----
#define MOTOR_PWM_FREQ        20000   // PWM frequency in Hz
#define MOTOR_MAX_SPEED       255     // Maximum PWM value (0-255)
#define MOTOR_RAMP_RATE       5       // Speed change per loop (acceleration limit)

//******************************************************************************
//                    END USER CONFIGURATION
//******************************************************************************

//******************************************************************************
// Pin Definitions (Fixed by V2.1 PCB design - DO NOT CHANGE)
//******************************************************************************

// ===== ANALOG INPUTS (0-3.3V) =====
#define PIN_STEERING_POT      17      // A3  - Steering Pot (S1_WIPER)
#define PIN_THROTTLE_POT      23      // A9  - Throttle Pot (T1_WIPER)
#define PIN_AS5600_ANALOG     16      // A2  - AS5600 Analog Out
#define PIN_STEER_CURRENT_L   24      // A10 - Steer Motor Current L
#define PIN_STEER_CURRENT_R   25      // A11 - Steer Motor Current R
#define PIN_FRONT_CURRENT_L   26      // A12 - Front Drive Current L
#define PIN_FRONT_CURRENT_R   27      // A13 - Front Drive Current R
#define PIN_REAR_CURRENT_L    38      // A14 - Rear Drive Current L
#define PIN_REAR_CURRENT_R    39      // A15 - Rear Drive Current R

// ===== DIGITAL INPUTS (3.3V Logic) =====
#define PIN_SHIFTER_FWD       29      // Shifter Forward (swapped)
#define PIN_SHIFTER_REV       28      // Shifter Reverse (swapped)
#define PIN_MODE_SELECT       30      // Mode Select
#define PIN_EBRAKE            31      // E-Brake Input

// ===== PWM OUTPUTS (3.3V) =====
#define PIN_STEER_LPWM        2       // Steering LPWM
#define PIN_STEER_RPWM        3       // Steering RPWM
#define PIN_FRONT_LPWM        6       // Front Drive LPWM
#define PIN_FRONT_RPWM        9       // Front Drive RPWM
#define PIN_REAR_LPWM         12      // Rear Drive LPWM
#define PIN_REAR_RPWM         13      // Rear Drive RPWM

// ===== MOTOR DRIVER ENABLE OUTPUTS (3.3V -> 5V via level shifter) =====
#define PIN_STEER_L_EN        4       // Steering L_EN
#define PIN_STEER_R_EN        5       // Steering R_EN
#define PIN_FRONT_L_EN        10      // Front Drive L_EN
#define PIN_FRONT_R_EN        11      // Front Drive R_EN
#define PIN_REAR_L_EN         32      // Rear Drive L_EN
#define PIN_REAR_R_EN         33      // Rear Drive R_EN

// ===== I2C (AS5600 Encoder) =====
#define PIN_I2C_SDA           18      // I2C Data
#define PIN_I2C_SCL           19      // I2C Clock

// ===== UART TO ESP32-S3 =====
// Serial2: OTA Firmware Updates
#define PIN_ESP_RX2           7       // Teensy RX2 <- ESP32 GPIO43 (TX)
#define PIN_ESP_TX2           8       // Teensy TX2 -> ESP32 GPIO44 (RX)
// Serial3: Debug/Monitoring
#define PIN_ESP_RX3           15      // Teensy RX3 <- ESP32 GPIO17 (TX)
#define PIN_ESP_TX3           14      // Teensy TX3 -> ESP32 GPIO18 (RX)

//******************************************************************************
// Configuration
//******************************************************************************
#define HEARTBEAT_INTERVAL 10000  // ms
#define LOOP_TIME_SMOOTHING 0.1   // EMA smoothing factor for loop time

//******************************************************************************
// Global Variables
//******************************************************************************
uint32_t loopStartTime = 0;
uint32_t smoothedLoopTime = 0;

//******************************************************************************
// setup()
//******************************************************************************
void setup() {
    // Initialize USB Serial for debugging
    Serial.begin(115200);
    delay(1000);
    
    // Initialize communication serials
    Serial2.begin(115200);  // ESP32 Flash Serial
    Serial3.begin(115200);  // ESP32 Debug/Telemetry Serial
    
    // Print startup banner
    Serial.println("\n========================================");
    Serial.println("   Kids Ride-On Controller - Teensy 4.1");
    Serial.println("========================================");
    Serial.printf("Firmware: %s\n", FIRMWARE_VERSION);
    Serial.printf("Build:    %s\n", BUILD_DATE);
    Serial.println();
    
    // Initialize OTA update system
    OTA.begin(&Serial2, &Serial3, &Serial, FIRMWARE_VERSION, BUILD_DATE);
    
    // Initialize I2C for AS5600 encoder
    Wire.begin();
    Wire.setClock(400000);  // 400 kHz I2C speed
    Serial.println("I2C Initialized for AS5600 encoder (400 kHz)");
    
    // Scan I2C bus for devices
    delay(100);
    scanI2CBus();
    delay(100);
    
    // Initialize telemetry (sends to ESP32 via Serial3)
    Telem.begin(&Serial3, &Serial);
    Telem.setInterval(100);  // 100ms = 10Hz update rate
    Telem.setCompactMode(true);
    
    // Initialize peripherals
    initPeripherals();
    
    // Notify ESP32 we're ready
    Serial3.printf("Teensy v%s ready\n", FIRMWARE_VERSION);
    
    Serial.println("\nCommands: 'update', 'version', 'status', 'telem', 'header'");
    Serial.println("Ready.\n");
}

//******************************************************************************
// setSteeringMotor() - Control steering motor based on steering input
// steeringPercent: -100 to +100 (-100=full left, +100=full right)
// calibrationMode: if true, use reduced speed for calibration
//******************************************************************************
void setSteeringMotor(int steeringPercent, bool calibrationMode = true) {
    // Calibration mode: slow speed (20/255 ≈ 8%) for precise control
    // Normal mode: full speed (255)
    int maxSpeed = calibrationMode ? 20 : MOTOR_MAX_SPEED;
    
    // Convert percentage to PWM value
    // steeringPercent ranges from -100 to +100
    int pwmValue = (abs(steeringPercent) * maxSpeed) / 100;
    
    // Clamp to max
    if (pwmValue > maxSpeed) pwmValue = maxSpeed;
    
    // Enable both steering motor drivers
    digitalWrite(PIN_STEER_L_EN, HIGH);
    digitalWrite(PIN_STEER_R_EN, HIGH);
    
    if (steeringPercent > 5) {
        // Steer RIGHT: drive right PWM, left PWM off
        analogWrite(PIN_STEER_RPWM, pwmValue);
        analogWrite(PIN_STEER_LPWM, 0);
        Serial.printf("PWM RIGHT: steerPct=%d, pin3=%d\n", steeringPercent, pwmValue);
    } else if (steeringPercent < -5) {
        // Steer LEFT: drive left PWM, right PWM off
        analogWrite(PIN_STEER_LPWM, pwmValue);
        analogWrite(PIN_STEER_RPWM, 0);
        Serial.printf("PWM LEFT: steerPct=%d, pin2=%d\n", steeringPercent, pwmValue);
    } else {
        // CENTERED: all PWM off
        analogWrite(PIN_STEER_LPWM, 0);
        analogWrite(PIN_STEER_RPWM, 0);
    }
}

//******************************************************************************
// loop()
//******************************************************************************
void loop() {
    loopStartTime = micros();
    
    // Check for OTA update commands from ESP32
    OTA.checkForCommands();
    
    // Handle USB serial commands
    handleSerialCommands();
    
    // Read all inputs and update telemetry data
    readInputs();
    // Print shifter status every loop
    TelemetryData& td = Telem.data();
    Serial.printf("Shifter:   %s\n", td.shifterFwd ? "FWD" : (td.shifterRev ? "REV" : "NEUTRAL"));
    // Run peripheral control logic
    updatePeripherals();
    
    // Calculate loop time and update telemetry
    uint32_t loopTime = micros() - loopStartTime;
    smoothedLoopTime = (LOOP_TIME_SMOOTHING * loopTime) + ((1.0 - LOOP_TIME_SMOOTHING) * smoothedLoopTime);
    Telem.data().loopTimeUs = smoothedLoopTime;
    Telem.data().cpuTemp = tempmonGetTemp();  // Teensy 4.1 internal temp
    
    // Send telemetry (respects interval setting)
    Telem.update();
    
    // Heartbeat (for ESP32 connection monitoring)
    heartbeat();
}

//******************************************************************************
// scanI2CBus() - Scan for I2C devices on the bus
//******************************************************************************
void scanI2CBus() {
    Serial.println("\n=== I2C Bus Scan ===");
    delay(500);  // Give time to see the header
    int found = 0;
    
    for (byte addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        byte error = Wire.endTransmission();
        
        if (error == 0) {
            Serial.printf("  [0x%02X] Device found\n", addr);
            delay(100);  // Slow down output so it's readable
            found++;
            
            // If this is the AS5600 address, probe it
            if (addr == ENCODER_I2C_ADDR) {
                delay(200);
                probeAS5600();
                delay(200);
            }
        }
    }
    
    delay(300);
    if (found == 0) {
        Serial.println("  NO DEVICES FOUND on I2C bus!");
        Serial.println("  CHECK: SDA/SCL connections, pull-ups, power supply");
    } else {
        Serial.printf("  Total: %d device(s)\n", found);
    }
    Serial.println();
    delay(500);
}

//******************************************************************************
// probeAS5600() - Read AS5600 registers and display raw values
//******************************************************************************
void probeAS5600() {
    Serial.println("\n--- AS5600 Register Probe ---");
    
    // Read status register (0x02)
    Wire.beginTransmission(ENCODER_I2C_ADDR);
    Wire.write(0x02);
    Wire.endTransmission();
    Wire.requestFrom(ENCODER_I2C_ADDR, 1);
    
    if (Wire.available()) {
        uint8_t status = Wire.read();
        Serial.printf("  Status Reg (0x02): 0x%02X\n", status);
        Serial.printf("    MagnetDetected: %s\n", (status & 0x20) ? "NO" : "YES");
        Serial.printf("    MagnetTooWeak:  %s\n", (status & 0x10) ? "YES" : "NO");
        Serial.printf("    MagnetTooStrong:%s\n", (status & 0x08) ? "YES" : "NO");
    } else {
        Serial.println("  ERROR: Cannot read Status register");
        return;
    }
    
    // Read angle registers (0x0E = MSB, 0x0F = LSB)
    Wire.beginTransmission(ENCODER_I2C_ADDR);
    Wire.write(0x0E);
    Wire.endTransmission();
    Wire.requestFrom(ENCODER_I2C_ADDR, 2);
    
    if (Wire.available() >= 2) {
        uint16_t rawAngle = (Wire.read() << 8) | Wire.read();
        uint16_t angleDeg = (rawAngle * 360) / 4096;
        Serial.printf("  Angle Raw (0x0E/0x0F): %4d counts = %3d degrees\n", rawAngle, angleDeg);
    } else {
        Serial.println("  ERROR: Cannot read Angle registers");
    }
    
    Serial.println();
}

//******************************************************************************
// readAS5600Angle() - Read AS5600 angle via I2C
//******************************************************************************
void readAS5600Angle(TelemetryData& td) {
    // AS5600 stores angle in registers 0x0E (MSB) and 0x0F (LSB)
    // Full 12-bit angle value = (MSB << 8) | LSB
    // 4096 counts = 360 degrees, so angle_degrees = (counts / 4096) * 360
    
    static bool i2cErrorReported = false;
    
    Wire.beginTransmission(ENCODER_I2C_ADDR);
    Wire.write(0x0E);  // Angle register MSB
    byte error = Wire.endTransmission();
    
    if (error != 0) {
        if (!i2cErrorReported) {
            Serial.printf("I2C Error on transmission: %d (will not repeat)\n", error);
            i2cErrorReported = true;
        }
        td.encoderAngle = 0;
        return;
    }
    
    Wire.requestFrom(ENCODER_I2C_ADDR, 2);
    if (Wire.available() >= 2) {
        uint16_t rawAngle = (Wire.read() << 8) | Wire.read();
        // Convert 12-bit count (0-4095) to degrees (0-359)
        td.encoderAngle = (rawAngle * 360) / 4096;
        i2cErrorReported = false;  // Reset flag when communication works
    } else {
        if (!i2cErrorReported) {
            Serial.printf("I2C Error: No data from AS5600 (available: %d bytes)\n", Wire.available());
            i2cErrorReported = true;
        }
        td.encoderAngle = 0;  // Error reading
    }
}

// ...existing code...
void readInputs() {
    TelemetryData& td = Telem.data();
    
    // Read analog inputs
    td.steeringRaw = analogRead(PIN_STEERING_POT);
    td.throttleRaw = analogRead(PIN_THROTTLE_POT);
    
    // Read AS5600 angle via I2C
    readAS5600Angle(td);
    
    // Read current sensors
    td.steerCurrentL = analogRead(PIN_STEER_CURRENT_L);  // TODO: Convert to mA
    td.steerCurrentR = analogRead(PIN_STEER_CURRENT_R);
    td.frontCurrentL = analogRead(PIN_FRONT_CURRENT_L);
    td.frontCurrentR = analogRead(PIN_FRONT_CURRENT_R);
    td.rearCurrentL = analogRead(PIN_REAR_CURRENT_L);
    td.rearCurrentR = analogRead(PIN_REAR_CURRENT_R);
    
    // Read digital inputs (active low with pullup)
    // Shifter: ground pulls pin LOW when activated, both HIGH = neutral
    int fwdPin = digitalRead(PIN_SHIFTER_FWD);  // LOW=0 when activated, HIGH=1 when not
    int revPin = digitalRead(PIN_SHIFTER_REV);  // LOW=0 when activated, HIGH=1 when not
    td.shifterFwd = (fwdPin == LOW);   // Forward: FWD pin is pulled LOW
    td.shifterRev = (revPin == LOW);   // Reverse: REV pin is pulled LOW
    // Note: Neutral = both pins HIGH (shifterFwd==false AND shifterRev==false)
    
    // Other digital inputs (inverted logic for non-shifter)
    td.modeSelect = !digitalRead(PIN_MODE_SELECT);
    td.eBrake = !digitalRead(PIN_EBRAKE);
    
    // Calculate processed values
    // Steering: map raw to -100 to +100 with deadzone
    int steerCentered = td.steeringRaw - STEERING_POT_CENTER;
    if (abs(steerCentered) < STEERING_POT_DEADZONE) {
        td.steeringPercent = 0;
    } else if (steerCentered > 0) {
        td.steeringPercent = map(steerCentered, STEERING_POT_DEADZONE, 
                                  STEERING_POT_MAX - STEERING_POT_CENTER, 0, 100);
    } else {
        td.steeringPercent = map(steerCentered, STEERING_POT_MIN - STEERING_POT_CENTER,
                                  -STEERING_POT_DEADZONE, -100, 0);
    }
    td.steeringPercent = constrain(td.steeringPercent, -100, 100);
    
    // Throttle: map raw to 0-100 with deadzone
    if (td.throttleRaw < THROTTLE_POT_MIN + THROTTLE_POT_DEADZONE) {
        td.throttlePercent = 0;
    } else {
        td.throttlePercent = map(td.throttleRaw, 
                                  THROTTLE_POT_MIN + THROTTLE_POT_DEADZONE,
                                  THROTTLE_POT_MAX, 0, 100);
    }
    td.throttlePercent = constrain(td.throttlePercent, 0, 100);
}

//******************************************************************************
// initPeripherals() - Initialize all ride-on peripherals
//******************************************************************************
void initPeripherals() {
    // Configure analog inputs (default on Teensy, but explicit is good)
    analogReadResolution(10);  // 10-bit ADC (0-1023)
    
    // Configure digital inputs with internal pullups
    pinMode(PIN_SHIFTER_FWD, INPUT_PULLUP);
    pinMode(PIN_SHIFTER_REV, INPUT_PULLUP);
    pinMode(PIN_MODE_SELECT, INPUT_PULLUP);
    pinMode(PIN_EBRAKE, INPUT_PULLUP);
    
    // Configure PWM outputs
    pinMode(PIN_STEER_LPWM, OUTPUT);
    pinMode(PIN_STEER_RPWM, OUTPUT);
    pinMode(PIN_FRONT_LPWM, OUTPUT);
    pinMode(PIN_FRONT_RPWM, OUTPUT);
    pinMode(PIN_REAR_LPWM, OUTPUT);
    pinMode(PIN_REAR_RPWM, OUTPUT);
    
    // Configure motor enable outputs (active high)
    pinMode(PIN_STEER_L_EN, OUTPUT);
    pinMode(PIN_STEER_R_EN, OUTPUT);
    pinMode(PIN_FRONT_L_EN, OUTPUT);
    pinMode(PIN_FRONT_R_EN, OUTPUT);
    pinMode(PIN_REAR_L_EN, OUTPUT);
    pinMode(PIN_REAR_R_EN, OUTPUT);
    
    // Set PWM frequency for motor outputs
    analogWriteFrequency(PIN_STEER_LPWM, MOTOR_PWM_FREQ);
    analogWriteFrequency(PIN_STEER_RPWM, MOTOR_PWM_FREQ);
    analogWriteFrequency(PIN_FRONT_LPWM, MOTOR_PWM_FREQ);
    analogWriteFrequency(PIN_FRONT_RPWM, MOTOR_PWM_FREQ);
    analogWriteFrequency(PIN_REAR_LPWM, MOTOR_PWM_FREQ);
    analogWriteFrequency(PIN_REAR_RPWM, MOTOR_PWM_FREQ);
    
    // Start with all outputs off
    analogWrite(PIN_STEER_LPWM, 0);
    analogWrite(PIN_STEER_RPWM, 0);
    analogWrite(PIN_FRONT_LPWM, 0);
    analogWrite(PIN_FRONT_RPWM, 0);
    analogWrite(PIN_REAR_LPWM, 0);
    analogWrite(PIN_REAR_RPWM, 0);
    
    digitalWrite(PIN_STEER_L_EN, LOW);
    digitalWrite(PIN_STEER_R_EN, LOW);
    digitalWrite(PIN_FRONT_L_EN, LOW);
    digitalWrite(PIN_FRONT_R_EN, LOW);
    digitalWrite(PIN_REAR_L_EN, LOW);
    digitalWrite(PIN_REAR_R_EN, LOW);
    
    Serial.println("Peripherals initialized");
    Serial.printf("  PWM frequency: %d Hz\n", MOTOR_PWM_FREQ);
}

//******************************************************************************
// updatePeripherals() - Main peripheral control loop
//******************************************************************************
void updatePeripherals() {
    TelemetryData& td = Telem.data();
    
    // CALIBRATION MODE: Drive steering motor based on steering input
    // User rotates steering wheel and watches encoder angle output
    // This helps find the center position for alignment
    setSteeringMotor(td.steeringPercent, true);  // true = calibration mode (slow speed)
}

//******************************************************************************
// handleSerialCommands() - Process USB serial commands
//******************************************************************************
void handleSerialCommands() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        input.toLowerCase();
        
        if (input == "update") {
            OTA.startUpdate();
        } 
        else if (input == "version") {
            Serial.printf("Version: %s\n", FIRMWARE_VERSION);
            Serial.printf("Build:   %s\n", BUILD_DATE);
        }
        else if (input == "status") {
            OTA.printStatus();
            printPeripheralStatus();
        }
        else if (input == "telem" || input == "t") {
            Telem.enable(!Telem.isEnabled());
            Serial.printf("Telemetry: %s\n", Telem.isEnabled() ? "ON" : "OFF");
        }
        else if (input == "header" || input == "h") {
            Telem.printHeader();
        }
        else if (input == "verbose" || input == "v") {
            Telem.setCompactMode(false);
            Serial.println("Telemetry: Verbose mode");
        }
        else if (input == "compact" || input == "c") {
            Telem.setCompactMode(true);
            Serial.println("Telemetry: Compact mode");
        }
        else if (input.startsWith("rate ")) {
            int rate = input.substring(5).toInt();
            if (rate >= 10 && rate <= 1000) {
                Telem.setInterval(rate);
                Serial.printf("Telemetry rate: %d ms (%.1f Hz)\n", rate, 1000.0/rate);
            } else {
                Serial.println("Rate must be 10-1000 ms");
            }
        }
        else if (input == "help") {
            printHelp();
        }
        else if (input.length() > 0) {
            Serial.printf("Unknown command: '%s' (type 'help')\n", input.c_str());
        }
    }
}

//******************************************************************************
// printPeripheralStatus() - Print status of all peripherals
//******************************************************************************
void printPeripheralStatus() {
    TelemetryData& td = Telem.data();
    
    Serial.println("\n=== Peripheral Status ===");
    Serial.println("--- Inputs ---");
    Serial.printf("  Steering:  %4d raw  %4d%%\n", td.steeringRaw, td.steeringPercent);
    Serial.printf("  Throttle:  %4d raw  %4d%%\n", td.throttleRaw, td.throttlePercent);
    Serial.printf("  Encoder:   %3d deg\n", td.encoderAngle);
    Serial.printf("  Shifter:   %s\n", td.shifterFwd ? "FWD" : (td.shifterRev ? "REV" : "NEUTRAL"));
    Serial.printf("  Mode:      %s\n", td.modeSelect ? "ON" : "OFF");
    Serial.printf("  E-Brake:   %s\n", td.eBrake ? "ENGAGED" : "OFF");
    
    Serial.println("--- Outputs ---");
    Serial.printf("  Steer PWM: L=%3d  R=%3d\n", td.steerPwmL, td.steerPwmR);
    Serial.printf("  Front PWM: L=%3d  R=%3d\n", td.frontPwmL, td.frontPwmR);
    Serial.printf("  Rear PWM:  L=%3d  R=%3d\n", td.rearPwmL, td.rearPwmR);
    
    Serial.println("--- Current (mA) ---");
    Serial.printf("  Steer:     L=%5d  R=%5d\n", td.steerCurrentL, td.steerCurrentR);
    Serial.printf("  Front:     L=%5d  R=%5d\n", td.frontCurrentL, td.frontCurrentR);
    Serial.printf("  Rear:      L=%5d  R=%5d\n", td.rearCurrentL, td.rearCurrentR);
    
    Serial.println("--- System ---");
    Serial.printf("  Loop time: %lu us\n", td.loopTimeUs);
    Serial.printf("  CPU temp:  %d C\n", td.cpuTemp);
    Serial.printf("  Telemetry: %s\n", Telem.isEnabled() ? "ON" : "OFF");
}

//******************************************************************************
// printHelp() - Print available commands
//******************************************************************************
void printHelp() {
    Serial.println("\n=== Available Commands ===");
    Serial.println("  update      - Start OTA firmware update");
    Serial.println("  version     - Show firmware version");
    Serial.println("  status      - Show system status");
    Serial.println("  help        - Show this help");
    Serial.println();
    Serial.println("=== Telemetry Commands ===");
    Serial.println("  telem (t)   - Toggle telemetry on/off");
    Serial.println("  header (h)  - Print telemetry header");
    Serial.println("  compact (c) - Single-line telemetry mode");
    Serial.println("  verbose (v) - Multi-line telemetry mode");
    Serial.println("  rate <ms>   - Set update rate (10-1000 ms)");
}

//******************************************************************************
// heartbeat() - Periodic status output
//******************************************************************************
void heartbeat() {
    static unsigned long lastHeartbeat = 0;
    
    if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
        Serial.println("Teensy running...");
        Serial3.println("heartbeat");
        lastHeartbeat = millis();
    }
}
