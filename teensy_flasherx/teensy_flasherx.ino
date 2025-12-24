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
#include "FlasherXOTA.h"
#include "Telemetry.h"

//******************************************************************************
// Version Information
//******************************************************************************
#define FIRMWARE_VERSION "1.3.2"
#define BUILD_DATE       __DATE__ " " __TIME__

//******************************************************************************
//                    USER CONFIGURATION SECTION
//******************************************************************************
// Calibrate these values to match YOUR specific sensors and hardware.
// Use the serial monitor to read raw values, then enter your min/max here.
//******************************************************************************

// ----- STEERING POT (S1_WIPER) -----
#define STEERING_POT_PIN      17      // Pin 17 (A3) - Steering potentiometer
#define STEERING_POT_MIN      0       // Raw ADC value at full LEFT
#define STEERING_POT_MAX      1023    // Raw ADC value at full RIGHT
#define STEERING_POT_CENTER   512     // Raw ADC value at CENTER (neutral)
#define STEERING_POT_DEADZONE 20      // Ignore values within +/- this of center

// ----- THROTTLE POT (T1_WIPER) -----
#define THROTTLE_POT_PIN      23      // Pin 23 (A9) - Throttle potentiometer
#define THROTTLE_POT_MIN      100     // Raw ADC value at NO throttle (foot off)
#define THROTTLE_POT_MAX      900     // Raw ADC value at FULL throttle
#define THROTTLE_POT_DEADZONE 30      // Ignore small values (prevents creep)

// ----- AS5600 MAGNETIC ENCODER -----
// Can be read via I2C (Pin 18/19) OR analog output (Pin 16)
#define ENCODER_USE_I2C       false   // true = I2C, false = Analog
#define ENCODER_I2C_ADDR      0x36    // Default AS5600 I2C address
#define ENCODER_ANALOG_PIN    16      // Pin 16 (A2) - AS5600 analog output
#define ENCODER_MIN           0       // Encoder counts/ADC at minimum
#define ENCODER_MAX           4095    // Encoder counts at maximum (12-bit I2C)
#define ENCODER_CPR           4096    // Counts per revolution
#define ENCODER_ANALOG_MIN    0       // Raw ADC at 0 degrees (analog mode)
#define ENCODER_ANALOG_MAX    1023    // Raw ADC at 360 degrees (analog mode)

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
#define PIN_SHIFTER_FWD       28      // Shifter Forward
#define PIN_SHIFTER_REV       29      // Shifter Reverse
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
// readInputs() - Read all sensor inputs and update telemetry
//******************************************************************************
void readInputs() {
    TelemetryData& td = Telem.data();
    
    // Read analog inputs
    td.steeringRaw = analogRead(PIN_STEERING_POT);
    td.throttleRaw = analogRead(PIN_THROTTLE_POT);
    td.encoderRaw = analogRead(PIN_AS5600_ANALOG);
    
    // Read current sensors
    td.steerCurrentL = analogRead(PIN_STEER_CURRENT_L);  // TODO: Convert to mA
    td.steerCurrentR = analogRead(PIN_STEER_CURRENT_R);
    td.frontCurrentL = analogRead(PIN_FRONT_CURRENT_L);
    td.frontCurrentR = analogRead(PIN_FRONT_CURRENT_R);
    td.rearCurrentL = analogRead(PIN_REAR_CURRENT_L);
    td.rearCurrentR = analogRead(PIN_REAR_CURRENT_R);
    
    // Read digital inputs (active low with pullup, so invert)
    td.shifterFwd = !digitalRead(PIN_SHIFTER_FWD);
    td.shifterRev = !digitalRead(PIN_SHIFTER_REV);
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
    // TODO: Add your peripheral control logic here
    // This runs every loop iteration
    // Examples:
    // - Read joystick/controller input
    // - Update motor speeds
    // - Update LED animations
    // - Check safety sensors
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
    Serial.printf("  Encoder:   %4d raw\n", td.encoderRaw);
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
