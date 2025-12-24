/*******************************************************************************
 * Teensy 4.1 Kids Ride-On Controller
 * 
 * Main application code for controlling ride-on vehicle peripherals.
 * OTA firmware updates handled by FlasherXOTA module.
 * 
 * Serial Configuration (Custom PCB):
 *   Serial  (USB)    - Arduino IDE / Debug Console
 *   Serial2 (7/8)    - ESP32 Flash Serial (OTA updates)
 *   Serial3 (15/14)  - ESP32 Debug Serial
 ******************************************************************************/

#include <Arduino.h>
#include "FlasherXOTA.h"

//******************************************************************************
// Version Information
//******************************************************************************
#define FIRMWARE_VERSION "1.2.4"
#define BUILD_DATE       __DATE__ " " __TIME__

//******************************************************************************
//                    USER CONFIGURATION SECTION
//******************************************************************************
// Calibrate these values to match YOUR specific sensors and hardware.
// Use the serial monitor to read raw values, then enter your min/max here.
//******************************************************************************

// ----- STEERING 1 (Primary/Front) -----
// Potentiometer or encoder connected to steering mechanism
#define STEERING1_PIN         A0      // Analog input pin
#define STEERING1_MIN         0       // Raw ADC value at full LEFT
#define STEERING1_MAX         1023    // Raw ADC value at full RIGHT
#define STEERING1_CENTER      512     // Raw ADC value at CENTER (neutral)
#define STEERING1_DEADZONE    20      // Ignore values within +/- this of center

// ----- STEERING 2 (Secondary/Rear, if applicable) -----
#define STEERING2_PIN         A1      // Analog input pin
#define STEERING2_MIN         0       // Raw ADC value at full LEFT
#define STEERING2_MAX         1023    // Raw ADC value at full RIGHT
#define STEERING2_CENTER      512     // Raw ADC value at CENTER (neutral)
#define STEERING2_DEADZONE    20      // Ignore values within +/- this of center

// ----- THROTTLE 1 (Primary) -----
// Pedal or lever controlling main drive
#define THROTTLE1_PIN         A2      // Analog input pin
#define THROTTLE1_MIN         100     // Raw ADC value at NO throttle (foot off)
#define THROTTLE1_MAX         900     // Raw ADC value at FULL throttle
#define THROTTLE1_DEADZONE    30      // Ignore small values (prevents creep)

// ----- THROTTLE 2 (Secondary/Reverse, if applicable) -----
#define THROTTLE2_PIN         A3      // Analog input pin
#define THROTTLE2_MIN         100     // Raw ADC value at NO throttle
#define THROTTLE2_MAX         900     // Raw ADC value at FULL throttle
#define THROTTLE2_DEADZONE    30      // Ignore small values

// ----- MAGNETIC ENCODER (Wheel Speed/Position) -----
// AS5600 or similar magnetic rotary encoder
// Can be read via I2C OR analog output (OUT pin)
#define ENCODER_USE_I2C       true    // true = I2C, false = Analog
#define ENCODER_I2C_ADDR      0x36    // Default AS5600 I2C address
#define ENCODER_ANALOG_PIN    A4      // Analog input if using OUT pin
#define ENCODER_MIN           0       // Encoder counts/ADC at minimum
#define ENCODER_MAX           4095    // Encoder counts at maximum (12-bit I2C)
                                      // or 0-1023 for 10-bit analog, 0-4095 for 12-bit ADC
#define ENCODER_CPR           4096    // Counts per revolution
#define ENCODER_ANALOG_MIN    0       // Raw ADC at 0 degrees (analog mode)
#define ENCODER_ANALOG_MAX    1023    // Raw ADC at 360 degrees (analog mode)

// ----- MOTOR CONFIGURATION -----
#define MOTOR_PWM_FREQ        20000   // PWM frequency in Hz
#define MOTOR_MAX_SPEED       255     // Maximum PWM value (0-255)
#define MOTOR_RAMP_RATE       5       // Speed change per loop (acceleration limit)

//******************************************************************************
//                    END USER CONFIGURATION
//******************************************************************************

//******************************************************************************
// Pin Definitions (Fixed by PCB design)
//******************************************************************************
// Serial pins are fixed by hardware:
// Serial2: Pin 7 (RX), Pin 8 (TX) -> ESP32 GPIO43/44 (OTA Flash)
// Serial3: Pin 15 (RX), Pin 14 (TX) -> ESP32 GPIO17/18 (Debug)

// Motor control pins (adjust to match your PCB)
// #define MOTOR1_PWM_PIN    2
// #define MOTOR1_DIR_PIN    3
// #define MOTOR2_PWM_PIN    4
// #define MOTOR2_DIR_PIN    5

// LED/Lighting pins
// #define LED_HEADLIGHT_PIN 6
// #define LED_TAILLIGHT_PIN 9
// #define LED_STRIP_PIN     10

//******************************************************************************
// Configuration
//******************************************************************************
#define HEARTBEAT_INTERVAL 10000  // ms

//******************************************************************************
// setup()
//******************************************************************************
void setup() {
    // Initialize USB Serial for debugging
    Serial.begin(115200);
    delay(1000);
    
    // Initialize communication serials
    Serial2.begin(115200);  // ESP32 Flash Serial
    Serial3.begin(115200);  // ESP32 Debug Serial
    
    // Print startup banner
    Serial.println("\n========================================");
    Serial.println("   Kids Ride-On Controller - Teensy 4.1");
    Serial.println("========================================");
    Serial.printf("Firmware: %s\n", FIRMWARE_VERSION);
    Serial.printf("Build:    %s\n", BUILD_DATE);
    Serial.println();
    
    // Initialize OTA update system
    OTA.begin(&Serial2, &Serial3, &Serial, FIRMWARE_VERSION, BUILD_DATE);
    
    // Initialize peripherals
    initPeripherals();
    
    // Notify ESP32 we're ready
    Serial3.printf("Teensy v%s ready\n", FIRMWARE_VERSION);
    
    Serial.println("\nCommands: 'update', 'version', 'status'");
    Serial.println("Ready.\n");
}

//******************************************************************************
// loop()
//******************************************************************************
void loop() {
    // Check for OTA update commands from ESP32
    OTA.checkForCommands();
    
    // Handle USB serial commands
    handleSerialCommands();
    
    // Run peripheral control logic
    updatePeripherals();
    
    // Heartbeat
    heartbeat();
}

//******************************************************************************
// initPeripherals() - Initialize all ride-on peripherals
//******************************************************************************
void initPeripherals() {
    // TODO: Add your peripheral initialization here
    // Examples:
    // - Motor controllers
    // - LED strips
    // - Audio system
    // - Sensors
    
    Serial.println("Peripherals initialized");
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
    Serial.println("\n=== Peripheral Status ===");
    // TODO: Add your peripheral status here
    Serial.println("(No peripherals configured yet)");
}

//******************************************************************************
// printHelp() - Print available commands
//******************************************************************************
void printHelp() {
    Serial.println("\n=== Available Commands ===");
    Serial.println("  update  - Start OTA firmware update");
    Serial.println("  version - Show firmware version");
    Serial.println("  status  - Show system status");
    Serial.println("  help    - Show this help");
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
