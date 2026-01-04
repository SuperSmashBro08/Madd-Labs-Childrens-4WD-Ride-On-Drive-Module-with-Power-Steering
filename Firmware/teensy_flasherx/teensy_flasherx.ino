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
#define FIRMWARE_VERSION "1.5.8"
#define BUILD_DATE       __DATE__ " " __TIME__

//******************************************************************************
//                    USER CONFIGURATION SECTION
//******************************************************************************
// Calibrate these values to match YOUR specific sensors and hardware.
// Use the serial monitor to read raw values, then enter your min/max here.
//******************************************************************************

// ----- STEERING POT (S1_WIPER) -----
// CALIBRATE THESE VALUES: Use serial monitor to read raw ADC values at each position
#define STEERING_POT_PIN      17      // Pin 17 (A3) - Steering potentiometer
#define STEERING_POT_MIN      105     // Raw ADC value at full LEFT (calibrate to your hardware)
#define STEERING_POT_MAX      708     // Raw ADC value at full RIGHT (calibrate to your hardware)
#define STEERING_POT_CENTER   407     // Raw ADC value at CENTER = (MIN+MAX)/2
#define STEERING_POT_DEADZONE 20      // Ignore values within +/- this of center
#define STEERING_INPUT_SMOOTH 15      // Ignore steering changes < this % to filter jitter
#define STEERING_FILTER_ALPHA 0.3     // Low-pass filter: 0.0-1.0 (lower = more filtering)

// ----- STEERING MOTOR LIMITS (Encoder Angle in Degrees) -----
// After applying ENCODER_ANGLE_OFFSET, center should display 180°
// This keeps steering range (135°-225°) away from the 0°/360° wraparound
#define STEER_ANGLE_CENTER    180     // Always 180 when offset is calibrated correctly
#define STEER_ANGLE_RANGE     45      // ± degrees from center (adjust for your steering linkage)
#define STEER_ANGLE_MIN       (STEER_ANGLE_CENTER - STEER_ANGLE_RANGE)  // Calculated: full left (135°)
#define STEER_ANGLE_MAX       (STEER_ANGLE_CENTER + STEER_ANGLE_RANGE)  // Calculated: full right (225°)
#define STEER_MOTOR_SPEED     100     // PWM speed for steering (0-255)
#define STEER_ANGLE_DEADBAND  15      // Stop motor when within ± this many degrees of target

// ----- THROTTLE POT (T1_WIPER) -----
#define THROTTLE_POT_PIN      23      // Pin 23 (A9) - Throttle potentiometer
#define THROTTLE_POT_MIN      249     // Raw ADC value at NO throttle (foot off)
#define THROTTLE_POT_MAX      790     // Raw ADC value at FULL throttle
#define THROTTLE_POT_DEADZONE 30      // Ignore small values (prevents creep)

// ----- AS5600 MAGNETIC ENCODER (I2C ONLY) -----
#define ENCODER_I2C_ADDR      0x36    // Default AS5600 I2C address
#define ENCODER_CPR           4096    // Counts per revolution (12-bit = 4096)
//
// HOW TO CALIBRATE ENCODER_ANGLE_OFFSET:
// 1. Set ENCODER_ANGLE_OFFSET to 0 temporarily
// 2. Upload firmware and open serial monitor
// 3. Steer wheels to STRAIGHT (center) position
// 4. Read the raw encoder angle displayed (e.g., 233°)
// 5. Calculate offset: ENCODER_ANGLE_OFFSET = (180 - raw_angle + 360) % 360
//    Example: If raw reads 233°, offset = (180 - 233 + 360) % 360 = 307
// 6. Set ENCODER_ANGLE_OFFSET to your calculated value
// 7. After upload, center should now display 180°
//
#define ENCODER_ANGLE_OFFSET  307     // Offset to make center read 180° (raw 233° + 307 = 540 % 360 = 180°)

// ----- CURRENT SENSING -----
// All current sense pins output 0-3.3V proportional to motor current
#define STEER_CURRENT_L_PIN   24      // Pin 24 (A10) - Steering motor current Left
#define STEER_CURRENT_R_PIN   25      // Pin 25 (A11) - Steering motor current Right
#define FRONT_CURRENT_L_PIN   26      // Pin 26 (A12) - Front drive current Left
#define FRONT_CURRENT_R_PIN   27      // Pin 27 (A13) - Front drive current Right
#define REAR_CURRENT_L_PIN    38      // Pin 38 (A14) - Rear drive current Left
#define REAR_CURRENT_R_PIN    39      // Pin 39 (A15) - Rear drive current Right
#define CURRENT_SENSE_SCALE   1.0     // mV per Amp (adjust for your sensor)

// ----- RC RECEIVER INPUTS (0-3.3V) -----
#define RC_STEER2_PIN         21      // Pin 21 (A7) - RC Steering (steer2)
#define RC_THROTTLE2_PIN      20      // Pin 20 (A6) - RC Throttle (throttle2)

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

// Soft stop configuration for drive motors
#define SOFT_STOP_RATE       5      // Percentage to decrease per update cycle
#define SOFT_STOP_INTERVAL   50     // Milliseconds between ramp-down steps

// Rollover protection - limits steering at high speed and slows down for sharp turns
#define ROLLOVER_PROTECTION_ENABLED  true   // Master enable for rollover protection
#define ROLLOVER_SPEED_THRESHOLD     50     // Above this speed %, start limiting steering
#define ROLLOVER_SPEED_FULL_LIMIT    80     // At this speed %, max steering limit applies
#define ROLLOVER_MIN_STEER_RANGE     15     // Minimum steering range % at high speed (15 = ±15°)
#define ROLLOVER_SLOWDOWN_ANGLE      30     // Steer angle % where speed reduction begins
#define ROLLOVER_SLOWDOWN_FACTOR     50     // Max speed % when at full steering angle

//******************************************************************************
// Global Variables
//******************************************************************************
uint32_t loopStartTime = 0;
uint32_t smoothedLoopTime = 0;

// RC Mode tracking
bool rcModeActive = false;           // Is RC mode currently active?
bool rcModePending = false;          // RC mode armed, waiting for neutral to fully activate
uint32_t throttleHoldStartTime = 0; // When did throttle go above threshold?

// Soft stop tracking for drive motors
int currentDriveSpeed = 0;           // Current actual motor speed (0-100)
int currentDriveDirection = 0;       // Current direction (0=OFF, 1=FWD, 2=REV)
uint32_t lastSoftStopTime = 0;       // Last time we did a ramp-down step

// Rollover protection state
int limitedSteeringPercent = 0;      // Steering % after speed-based limiting
int rolloverSpeedLimit = 100;        // Current speed limit due to steering angle
bool throttleHoldingHigh = false;   // Is throttle currently held high?
uint32_t lastRcInputTime = 0;       // Last time we got RC input (steer2 or throttle2)

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
    } else if (steeringPercent < -5) {
        // Steer LEFT: drive left PWM, right PWM off
        analogWrite(PIN_STEER_LPWM, pwmValue);
        analogWrite(PIN_STEER_RPWM, 0);
    } else {
        // CENTERED: all PWM off
        analogWrite(PIN_STEER_LPWM, 0);
        analogWrite(PIN_STEER_RPWM, 0);
    }
}

//******************************************************************************
// driveSteeringMotor() - Drive steering motor in a direction at given speed
// direction: 1=LEFT, 2=RIGHT
// throttlePercent: 0-100 (speed)
//******************************************************************************
void driveSteeringMotor(int direction, int throttlePercent) {
    // Convert throttle percentage to PWM (0-255)
    int pwmValue = map(throttlePercent, 0, 100, 0, MOTOR_MAX_SPEED);
    pwmValue = constrain(pwmValue, 0, MOTOR_MAX_SPEED);
    
    TelemetryData& td = Telem.data();
    
    // Enable both steering motor drivers
    digitalWrite(PIN_STEER_L_EN, HIGH);
    digitalWrite(PIN_STEER_R_EN, HIGH);
    
    if (direction == 1) {
        // LEFT: drive left PWM, right PWM off
        analogWrite(PIN_STEER_LPWM, pwmValue);
        analogWrite(PIN_STEER_RPWM, 0);
        td.steerPwmL = pwmValue;
        td.steerPwmR = 0;
    } else if (direction == 2) {
        // RIGHT: drive right PWM, left PWM off
        analogWrite(PIN_STEER_LPWM, 0);
        analogWrite(PIN_STEER_RPWM, pwmValue);
        td.steerPwmL = 0;
        td.steerPwmR = pwmValue;
    } else {
        // OFF
        allMotorsOff();
    }
}

//******************************************************************************
// driveFrontMotor() - Drive front motor in a direction at given speed
// direction: 1=FORWARD, 2=REVERSE
// throttlePercent: 0-100 (speed)
//******************************************************************************
void driveFrontMotor(int direction, int throttlePercent) {
    // Convert throttle percentage to PWM (0-255)
    int pwmValue = map(throttlePercent, 0, 100, 0, MOTOR_MAX_SPEED);
    pwmValue = constrain(pwmValue, 0, MOTOR_MAX_SPEED);
    
    TelemetryData& td = Telem.data();
    
    // Enable both front motor drivers
    digitalWrite(PIN_FRONT_L_EN, HIGH);
    digitalWrite(PIN_FRONT_R_EN, HIGH);
    
    if (direction == 1) {
        // FORWARD: drive left PWM, right PWM off
        analogWrite(PIN_FRONT_LPWM, pwmValue);
        analogWrite(PIN_FRONT_RPWM, 0);
        td.frontPwmL = pwmValue;
        td.frontPwmR = 0;
    } else if (direction == 2) {
        // REVERSE: drive right PWM, left PWM off
        analogWrite(PIN_FRONT_LPWM, 0);
        analogWrite(PIN_FRONT_RPWM, pwmValue);
        td.frontPwmL = 0;
        td.frontPwmR = pwmValue;
    } else {
        // OFF
        digitalWrite(PIN_FRONT_L_EN, LOW);
        digitalWrite(PIN_FRONT_R_EN, LOW);
        analogWrite(PIN_FRONT_LPWM, 0);
        analogWrite(PIN_FRONT_RPWM, 0);
        td.frontPwmL = 0;
        td.frontPwmR = 0;
    }
}

//******************************************************************************
// driveRearMotor() - Drive rear motor in a direction at given speed
// direction: 1=FORWARD, 2=REVERSE
// throttlePercent: 0-100 (speed)
//******************************************************************************
void driveRearMotor(int direction, int throttlePercent) {
    // Convert throttle percentage to PWM (0-255)
    int pwmValue = map(throttlePercent, 0, 100, 0, MOTOR_MAX_SPEED);
    pwmValue = constrain(pwmValue, 0, MOTOR_MAX_SPEED);
    
    TelemetryData& td = Telem.data();
    
    // Enable both rear motor drivers
    digitalWrite(PIN_REAR_L_EN, HIGH);
    digitalWrite(PIN_REAR_R_EN, HIGH);
    
    if (direction == 1) {
        // FORWARD: drive left PWM, right PWM off
        analogWrite(PIN_REAR_LPWM, pwmValue);
        analogWrite(PIN_REAR_RPWM, 0);
        td.rearPwmL = pwmValue;
        td.rearPwmR = 0;
    } else if (direction == 2) {
        // REVERSE: drive right PWM, left PWM off
        analogWrite(PIN_REAR_LPWM, 0);
        analogWrite(PIN_REAR_RPWM, pwmValue);
        td.rearPwmL = 0;
        td.rearPwmR = pwmValue;
    } else {
        // OFF
        digitalWrite(PIN_REAR_L_EN, LOW);
        digitalWrite(PIN_REAR_R_EN, LOW);
        analogWrite(PIN_REAR_LPWM, 0);
        analogWrite(PIN_REAR_RPWM, 0);
        td.rearPwmL = 0;
        td.rearPwmR = 0;
    }
}

//******************************************************************************
// applySoftStop() - Gradually ramp motor speed to target with soft stop
// Prevents jarring stops when throttle is released on geared motors
// targetDirection: 0=OFF, 1=FORWARD, 2=REVERSE
// targetSpeed: 0-100 (target percentage)
//******************************************************************************
void applySoftStop(int targetDirection, int targetSpeed) {
    uint32_t now = millis();
    
    // If direction changed (including to/from neutral), we need special handling
    if (targetDirection != currentDriveDirection) {
        if (targetDirection == 0 || currentDriveDirection == 0) {
            // Going to/from neutral - ramp down current speed first
            if (currentDriveSpeed > 0) {
                // Ramp down at soft stop rate
                if (now - lastSoftStopTime >= SOFT_STOP_INTERVAL) {
                    currentDriveSpeed -= SOFT_STOP_RATE;
                    if (currentDriveSpeed < 0) currentDriveSpeed = 0;
                    lastSoftStopTime = now;
                }
                // Drive at current ramping-down speed
                driveFrontMotor(currentDriveDirection, currentDriveSpeed);
                driveRearMotor(currentDriveDirection, currentDriveSpeed);
                return;
            } else {
                // Speed is 0, can change direction now
                currentDriveDirection = targetDirection;
            }
        } else {
            // Changing from FWD to REV or vice versa - must stop first
            if (currentDriveSpeed > 0) {
                // Ramp down first
                if (now - lastSoftStopTime >= SOFT_STOP_INTERVAL) {
                    currentDriveSpeed -= SOFT_STOP_RATE;
                    if (currentDriveSpeed < 0) currentDriveSpeed = 0;
                    lastSoftStopTime = now;
                }
                driveFrontMotor(currentDriveDirection, currentDriveSpeed);
                driveRearMotor(currentDriveDirection, currentDriveSpeed);
                return;
            } else {
                // Stopped, can change direction
                currentDriveDirection = targetDirection;
            }
        }
    }
    
    // Same direction - adjust speed towards target
    if (currentDriveSpeed < targetSpeed) {
        // Accelerating - can be immediate (no soft start needed)
        currentDriveSpeed = targetSpeed;
        lastSoftStopTime = now;
    } else if (currentDriveSpeed > targetSpeed) {
        // Decelerating - use soft stop
        if (now - lastSoftStopTime >= SOFT_STOP_INTERVAL) {
            currentDriveSpeed -= SOFT_STOP_RATE;
            if (currentDriveSpeed < targetSpeed) currentDriveSpeed = targetSpeed;
            lastSoftStopTime = now;
        }
    }
    
    // Drive motors at current speed
    if (currentDriveSpeed == 0) {
        driveFrontMotor(0, 0);
        driveRearMotor(0, 0);
    } else {
        driveFrontMotor(currentDriveDirection, currentDriveSpeed);
        driveRearMotor(currentDriveDirection, currentDriveSpeed);
    }
}

//******************************************************************************
// calculateRolloverProtection() - Limit steering at high speed, slow for sharp turns
// Returns: adjusted steering percentage based on current speed
// Also sets rolloverSpeedLimit for speed reduction during sharp turns
//******************************************************************************
int calculateRolloverProtection(int steeringInput, int currentSpeed) {
    if (!ROLLOVER_PROTECTION_ENABLED) {
        rolloverSpeedLimit = 100;
        return steeringInput;
    }
    
    int limitedSteering = steeringInput;
    
    // PART 1: Limit steering angle at high speeds
    // At speeds above threshold, reduce max steering angle proportionally
    if (currentSpeed > ROLLOVER_SPEED_THRESHOLD) {
        // Calculate how much to limit steering (0-100%)
        // At ROLLOVER_SPEED_THRESHOLD: no limit (100% steering allowed)
        // At ROLLOVER_SPEED_FULL_LIMIT: max limit (ROLLOVER_MIN_STEER_RANGE % allowed)
        int speedRange = ROLLOVER_SPEED_FULL_LIMIT - ROLLOVER_SPEED_THRESHOLD;
        int speedAboveThreshold = currentSpeed - ROLLOVER_SPEED_THRESHOLD;
        
        // Calculate allowed steering range (100% down to MIN_STEER_RANGE)
        int allowedRange;
        if (speedAboveThreshold >= speedRange) {
            allowedRange = ROLLOVER_MIN_STEER_RANGE;
        } else {
            // Linear interpolation between 100% and MIN_STEER_RANGE
            allowedRange = 100 - ((100 - ROLLOVER_MIN_STEER_RANGE) * speedAboveThreshold / speedRange);
        }
        
        // Limit steering to allowed range
        limitedSteering = constrain(steeringInput, -allowedRange, allowedRange);
    }
    
    // PART 2: Reduce speed when steering angle is high
    // This slows down the car before allowing a sharp turn
    int absSteer = abs(limitedSteering);
    if (absSteer > ROLLOVER_SLOWDOWN_ANGLE) {
        // Calculate speed limit based on steering angle
        // At ROLLOVER_SLOWDOWN_ANGLE: 100% speed allowed
        // At 100% steering: ROLLOVER_SLOWDOWN_FACTOR % speed max
        int steerRange = 100 - ROLLOVER_SLOWDOWN_ANGLE;
        int steerAboveThreshold = absSteer - ROLLOVER_SLOWDOWN_ANGLE;
        
        // Linear reduction from 100% to SLOWDOWN_FACTOR
        rolloverSpeedLimit = 100 - ((100 - ROLLOVER_SLOWDOWN_FACTOR) * steerAboveThreshold / steerRange);
        rolloverSpeedLimit = constrain(rolloverSpeedLimit, ROLLOVER_SLOWDOWN_FACTOR, 100);
    } else {
        rolloverSpeedLimit = 100;  // No speed limit for small steering angles
    }
    
    return limitedSteering;
}

//******************************************************************************
// allMotorsOff() - Turn off all motor PWM outputs
//******************************************************************************
void allMotorsOff() {
    TelemetryData& td = Telem.data();
    
    // Disable all enable pins - CRITICAL for backdrive prevention
    digitalWrite(PIN_STEER_L_EN, LOW);
    digitalWrite(PIN_STEER_R_EN, LOW);
    digitalWrite(PIN_FRONT_L_EN, LOW);
    digitalWrite(PIN_FRONT_R_EN, LOW);
    digitalWrite(PIN_REAR_L_EN, LOW);
    digitalWrite(PIN_REAR_R_EN, LOW);
    
    // Set all PWM to 0
    analogWrite(PIN_STEER_LPWM, 0);
    analogWrite(PIN_STEER_RPWM, 0);
    analogWrite(PIN_FRONT_LPWM, 0);
    analogWrite(PIN_FRONT_RPWM, 0);
    analogWrite(PIN_REAR_LPWM, 0);
    analogWrite(PIN_REAR_RPWM, 0);
    
    // Update telemetry
    td.steerPwmL = 0;
    td.steerPwmR = 0;
    td.frontPwmL = 0;
    td.frontPwmR = 0;
    td.rearPwmL = 0;
    td.rearPwmR = 0;
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
    
    // Debug output every 1 second
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug >= 1000) {
        lastDebug = millis();
        
        TelemetryData& td = Telem.data();
        int targetAngle = STEER_ANGLE_CENTER + (td.steeringPercent * STEER_ANGLE_RANGE) / 100;
        targetAngle = constrain(targetAngle, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
        
        int angleError = targetAngle - td.encoderAngle;
        if (angleError > 180) angleError -= 360;
        else if (angleError < -180) angleError += 360;
        
        Serial.printf("Steer1: %3d%% | Current: %3d° | Target: %3d° | Error: %4d° | PWM: L=%3d R=%3d\n",
                     td.steeringPercent, td.encoderAngle, targetAngle, angleError,
                     td.steerPwmL, td.steerPwmR);
        Serial.printf("Throttle1: %3d%% | Steer2: %3d%% (raw=%4d) | Throttle2: %3d%% (raw=%4d) | RC Mode: %s\n",
                     td.throttlePercent, td.steer2Percent, td.steer2Raw, td.throttle2Percent, td.throttle2Raw,
                     rcModeActive ? "ACTIVE" : "inactive");
    }
    
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
    static uint16_t lastValidAngle = 180;  // Remember last good reading (default to center)
    
    Wire.beginTransmission(ENCODER_I2C_ADDR);
    Wire.write(0x0E);  // Angle register MSB
    byte error = Wire.endTransmission();
    
    if (error != 0) {
        if (!i2cErrorReported) {
            Serial.printf("I2C Error on transmission: %d (will not repeat)\n", error);
            i2cErrorReported = true;
        }
        // Use last valid angle if I2C fails
        td.encoderAngle = lastValidAngle;
        return;
    }
    
    Wire.requestFrom(ENCODER_I2C_ADDR, 2);
    if (Wire.available() >= 2) {
        uint16_t rawAngle = (Wire.read() << 8) | Wire.read();
        // Convert 12-bit count (0-4095) to degrees (0-359)
        uint16_t angleDeg = (rawAngle * 360) / 4096;
        // Apply offset so neutral steering = 180°
        td.encoderAngle = (angleDeg + ENCODER_ANGLE_OFFSET) % 360;
        lastValidAngle = td.encoderAngle;
        i2cErrorReported = false;  // Reset flag when communication works
    } else {
        if (!i2cErrorReported) {
            Serial.printf("I2C Error: No data from AS5600 (available: %d bytes)\n", Wire.available());
            i2cErrorReported = true;
        }
        // Use last valid angle if I2C fails
        td.encoderAngle = lastValidAngle;
    }
}

// ...existing code...
void readInputs() {
    TelemetryData& td = Telem.data();
    
    // Read analog inputs
    td.steeringRaw = analogRead(PIN_STEERING_POT);
    td.throttleRaw = analogRead(PIN_THROTTLE_POT);
    
    // Read RC inputs (PWM servo signals - measure pulse width in microseconds)
    // Typical RC PWM: 1000µs = min, 1500µs = center, 2000µs = max
    td.steer2Raw = pulseIn(RC_STEER2_PIN, HIGH, 25000);      // 25ms timeout
    td.throttle2Raw = pulseIn(RC_THROTTLE2_PIN, HIGH, 25000); // 25ms timeout
    
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
    
    // Low-pass filter the raw steering value to reduce jitter
    static int filteredSteeringRaw = STEERING_POT_CENTER;
    filteredSteeringRaw = (int)(STEERING_FILTER_ALPHA * td.steeringRaw + 
                                 (1.0 - STEERING_FILTER_ALPHA) * filteredSteeringRaw);
    
    int steerCentered = filteredSteeringRaw - STEERING_POT_CENTER;
    int newSteeringPercent = 0;
    
    if (abs(steerCentered) < STEERING_POT_DEADZONE) {
        newSteeringPercent = 0;
    } else if (steerCentered > 0) {
        newSteeringPercent = map(steerCentered, STEERING_POT_DEADZONE, 
                                  STEERING_POT_MAX - STEERING_POT_CENTER, 0, 100);
    } else {
        newSteeringPercent = map(steerCentered, STEERING_POT_MIN - STEERING_POT_CENTER,
                                  -STEERING_POT_DEADZONE, -100, 0);
    }
    newSteeringPercent = constrain(newSteeringPercent, -100, 100);
    
    // Apply hysteresis filter to reduce jitter
    // Only update steering if it changes by more than STEERING_INPUT_SMOOTH
    if (abs(newSteeringPercent - td.steeringPercent) >= STEERING_INPUT_SMOOTH) {
        td.steeringPercent = newSteeringPercent;
    }
    // Otherwise keep the old value (hysteresis)
    
    // Throttle: map raw to 0-100 with deadzone
    if (td.throttleRaw < THROTTLE_POT_MIN + THROTTLE_POT_DEADZONE) {
        td.throttlePercent = 0;
    } else {
        td.throttlePercent = map(td.throttleRaw, 
                                  THROTTLE_POT_MIN + THROTTLE_POT_DEADZONE,
                                  THROTTLE_POT_MAX, 0, 100);
    }
    td.throttlePercent = constrain(td.throttlePercent, 0, 100);
    
    // RC Steering (steer2): PWM microseconds to -100 to +100
    // Calibrated: 1065µs=left, 1500µs=center, 2000µs=right
    if (td.steer2Raw == 0) {
        // No signal or timeout
        td.steer2Percent = 0;
    } else if (td.steer2Raw <= 1065) {
        // Full left
        td.steer2Percent = -100;
    } else if (td.steer2Raw >= 2000) {
        // Full right
        td.steer2Percent = 100;
    } else if (td.steer2Raw < 1500) {
        // Left side: map 1065-1500 to -100 to 0
        td.steer2Percent = map(td.steer2Raw, 1065, 1500, -100, 0);
    } else {
        // Right side: map 1500-2000 to 0 to 100
        td.steer2Percent = map(td.steer2Raw, 1500, 2000, 0, 100);
    }
    td.steer2Percent = constrain(td.steer2Percent, -100, 100);
    
    // RC Throttle (throttle2): PWM microseconds to -100 to +100 (bidirectional)
    // Calibrated: 1000µs=reverse, 1500µs=neutral, 2000µs=forward
    if (td.throttle2Raw == 0) {
        // No signal or timeout
        td.throttle2Percent = 0;
    } else if (td.throttle2Raw <= 1000) {
        // Full reverse
        td.throttle2Percent = -100;
    } else if (td.throttle2Raw >= 2000) {
        // Full forward
        td.throttle2Percent = 100;
    } else if (td.throttle2Raw < 1500) {
        // Reverse side: map 1000-1500 to -100 to 0
        td.throttle2Percent = map(td.throttle2Raw, 1000, 1500, -100, 0);
    } else {
        // Forward side: map 1500-2000 to 0 to 100
        td.throttle2Percent = map(td.throttle2Raw, 1500, 2000, 0, 100);
    }
    td.throttle2Percent = constrain(td.throttle2Percent, -100, 100);
}

//******************************************************************************
// initPeripherals() - Initialize all ride-on peripherals
//******************************************************************************
void initPeripherals() {
    // Configure analog inputs (default on Teensy, but explicit is good)
    analogReadResolution(10);  // 10-bit ADC (0-1023)
    
    // Configure analog pins for RC receiver
    pinMode(RC_STEER2_PIN, INPUT);
    pinMode(RC_THROTTLE2_PIN, INPUT);
    
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
    
    // RC MODE ACTIVATION DETECTION
    // Hold RC throttle (throttle2) full REVERSE for 3 seconds to arm RC mode
    // Then must return to neutral before RC mode fully activates (safety feature)
    // Deactivation only happens via 20-second neutral timeout (not by holding reverse again)
    if (!rcModeActive && td.throttle2Percent <= -90) {
        // RC throttle is held down (full reverse) - only check when not already active
        if (!throttleHoldingHigh) {
            // Just went down - start timer
            throttleHoldingHigh = true;
            throttleHoldStartTime = millis();
            Serial.println("[RC] RC Throttle held DOWN - counting down to arm RC mode...");
        } else {
            // RC throttle has been held down
            uint32_t holdTime = millis() - throttleHoldStartTime;
            if (holdTime >= 3000 && holdTime < 3100) {  // ~3 second activation window
                // Arming - set pending, wait for neutral
                rcModePending = true;
                Serial.println("[RC] *** RC MODE ARMED - Release throttle to neutral to activate ***");
            }
        }
    } else {
        // RC throttle is not held down anymore (or RC mode is already active)
        if (throttleHoldingHigh) {
            Serial.printf("[RC] RC Throttle released after %lu ms\n", millis() - throttleHoldStartTime);
            throttleHoldingHigh = false;
        }
        
        // Check if RC mode is pending and throttle is now at neutral
        if (rcModePending && abs(td.throttle2Percent) <= 10) {
            // Throttle returned to neutral - fully activate RC mode
            rcModeActive = true;
            rcModePending = false;
            lastRcInputTime = millis();
            Serial.println("[RC] *** RC MODE FULLY ACTIVATED - Throttle at neutral ***");
        }
    }
    
    // RC MODE TIMEOUT: If no RC input (steer2 or throttle2) for 20 seconds, return control to car
    if (rcModeActive) {
        // Check if there's any RC input (steer2 or throttle2 not neutral)
        bool hasRcInput = (abs(td.steer2Percent) > 5) || (abs(td.throttle2Percent) > 5);
        
        if (hasRcInput) {
            // RC is being used - reset timeout
            lastRcInputTime = millis();
        }
        
        uint32_t timeSinceLastRcInput = millis() - lastRcInputTime;
        if (timeSinceLastRcInput > 20000) {  // 20 seconds
            rcModeActive = false;
            Serial.println("[RC] *** RC MODE TIMEOUT - No RC input for 20 seconds, control returned to car ***");
        }
    }
    
    // Set control source for telemetry display
    td.controlSource = rcModeActive ? 2 : 1;  // 1=Car, 2=Remote
    
    // CONTROL LOGIC
    if (rcModeActive) {
        // RC MODE: Use steer2 and throttle2 (from RC receiver)
        updateRCControls(td);
    } else {
        // THROTTLE1 MODE: Use steering wheel and throttle pedal
        updateThrottle1Controls(td);
    }
}

//******************************************************************************
// updateThrottle1Controls() - Control using steering wheel + throttle pedal
//******************************************************************************
void updateThrottle1Controls(TelemetryData& td) {
    // THROTTLE1 MODE: Control front and rear motors with throttle
    // Shifter controls direction: FWD=Forward, REV=Reverse, NEUTRAL=Off
    // Rollover protection limits steering at high speed and slows for sharp turns
    
    // Apply rollover protection - limits steering based on speed, sets rolloverSpeedLimit
    limitedSteeringPercent = calculateRolloverProtection(td.steeringPercent, td.throttlePercent);
    
    // Calculate speed with rollover limit applied
    int actualSpeed = (td.throttlePercent * rolloverSpeedLimit) / 100;
    
    // DRIVE MOTORS: Shifter for direction, throttle for speed
    if (td.shifterFwd) {
        // Forward
        driveFrontMotor(1, actualSpeed);
        driveRearMotor(1, actualSpeed);
    } else if (td.shifterRev) {
        // Reverse
        driveFrontMotor(2, actualSpeed);
        driveRearMotor(2, actualSpeed);
    } else {
        // Neutral - stop motors
        driveFrontMotor(0, 0);
        driveRearMotor(0, 0);
    }
    
    // STEERING: Use steering input to position motor (only when shifter engaged)
    updateSteeringControls(td);
}

//******************************************************************************
// updateRCControls() - Control using RC remote (steer2/throttle2)
//******************************************************************************
void updateRCControls(TelemetryData& td) {
    // RC MODE: Full remote control - ignores shifter completely
    // throttle2: -100 (reverse) to 0 (stop) to +100 (forward)
    // steer2: -100 (left) to 0 (center) to +100 (right)
    // Rollover protection limits steering at high speed and slows for sharp turns
    
    // Apply rollover protection - limits steering based on speed, sets rolloverSpeedLimit
    limitedSteeringPercent = calculateRolloverProtection(td.steer2Percent, abs(td.throttle2Percent));
    
    // Calculate speed with rollover limit applied
    int actualSpeed = (abs(td.throttle2Percent) * rolloverSpeedLimit) / 100;
    
    // DRIVE MOTORS: throttle2 controls both direction AND speed
    if (abs(td.throttle2Percent) < 5) {
        // Throttle near neutral - stop motors
        driveFrontMotor(0, 0);
        driveRearMotor(0, 0);
    } else if (td.throttle2Percent > 0) {
        // Forward
        driveFrontMotor(1, actualSpeed);
        driveRearMotor(1, actualSpeed);
    } else {
        // Reverse
        driveFrontMotor(2, actualSpeed);
        driveRearMotor(2, actualSpeed);
    }
    
    // STEERING: Use steer2 for angle control (always enabled in RC mode)
    updateRCSteeringControls(td);
}

//******************************************************************************
// updateRCSteeringControls() - Steering control for RC mode (ignores shifter)
//******************************************************************************
void updateRCSteeringControls(TelemetryData& td) {
    // RC steering - always enabled regardless of shifter position
    // steer2: -100 (left) to 0 (center) to +100 (right)
    
    // Convert steer2 percentage to target angle
    // Maps -100% to STEER_ANGLE_MIN, 0% to STEER_ANGLE_CENTER, +100% to STEER_ANGLE_MAX
    int targetAngle = STEER_ANGLE_CENTER + (td.steer2Percent * STEER_ANGLE_RANGE) / 100;
    targetAngle = constrain(targetAngle, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
    
    // Get current encoder angle
    int currentAngle = td.encoderAngle;
    
    // Calculate error handling 360° wraparound
    int angleError = targetAngle - currentAngle;
    // Normalize to -180 to +180 range for shortest path
    if (angleError > 180) {
        angleError -= 360;
    } else if (angleError < -180) {
        angleError += 360;
    }
    
    // Determine motor direction based on error
    static bool wasMovingRC = false;
    // Check if at target (with deadband) OR if error is near wraparound ambiguity
    if (abs(angleError) <= STEER_ANGLE_DEADBAND || abs(abs(angleError) - 180) < 5) {
        // Within deadband OR at wraparound ambiguity - stop motor
        if (wasMovingRC) {
            wasMovingRC = false;
        }
        digitalWrite(PIN_STEER_L_EN, LOW);
        digitalWrite(PIN_STEER_R_EN, LOW);
        analogWrite(PIN_STEER_LPWM, 0);
        analogWrite(PIN_STEER_RPWM, 0);
        td.steerPwmL = 0;
        td.steerPwmR = 0;
    } else if (angleError > 0) {
        // Target is clockwise - turn RIGHT
        wasMovingRC = true;
        driveSteeringMotor(2, 100);  // direction=2 (RIGHT), speed=100%
    } else {
        // Target is counter-clockwise - turn LEFT
        wasMovingRC = true;
        driveSteeringMotor(1, 100);  // direction=1 (LEFT), speed=100%
    }
}

//******************************************************************************
// updateSteeringControls() - Handle steering motor control
//******************************************************************************
void updateSteeringControls(TelemetryData& td) {
    // Safety check: Only allow steering if shifter is engaged (FWD or REV)
    // In NEUTRAL, disable steering motor completely
    if (!td.shifterFwd && !td.shifterRev) {
        // Shifter in NEUTRAL - disable steering
        digitalWrite(PIN_STEER_L_EN, LOW);
        digitalWrite(PIN_STEER_R_EN, LOW);
        analogWrite(PIN_STEER_LPWM, 0);
        analogWrite(PIN_STEER_RPWM, 0);
        td.steerPwmL = 0;
        td.steerPwmR = 0;
        return;
    }
    
    // STEERING POT MODE: Use steering input to position motor
    // Encoder feedback drives motor to reach target position
    
    // Convert steering percentage to target angle
    // Maps -100% to STEER_ANGLE_MIN, 0% to STEER_ANGLE_CENTER, +100% to STEER_ANGLE_MAX
    int targetAngle = STEER_ANGLE_CENTER + (td.steeringPercent * STEER_ANGLE_RANGE) / 100;
    targetAngle = constrain(targetAngle, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
    
    // Get current encoder angle
    int currentAngle = td.encoderAngle;
    
    // Calculate error handling 360° wraparound
    int angleError = targetAngle - currentAngle;
    // Normalize to -180 to +180 range for shortest path
    if (angleError > 180) {
        angleError -= 360;
    } else if (angleError < -180) {
        angleError += 360;
    }
    
    // Determine motor direction based on error
    static bool wasMoving = false;
    // Check if at target (with deadband) OR if error is near wraparound ambiguity
    if (abs(angleError) <= STEER_ANGLE_DEADBAND || abs(abs(angleError) - 180) < 5) {
        // Within deadband OR at wraparound ambiguity - stop motor
        if (wasMoving) {
            Serial.printf(">>> STEER MOTOR OFF - Target reached! Error: %d°\n", angleError);
            wasMoving = false;
        }
        // Only disable steering EN pins, keep front/rear running
        digitalWrite(PIN_STEER_L_EN, LOW);
        digitalWrite(PIN_STEER_R_EN, LOW);
        analogWrite(PIN_STEER_LPWM, 0);
        analogWrite(PIN_STEER_RPWM, 0);
        td.steerPwmL = 0;
        td.steerPwmR = 0;
    } else if (angleError > 0) {
        // Target is clockwise - turn RIGHT
        if (!wasMoving) {
            Serial.println(">>> STEER MOTOR ON - Turning RIGHT");
            wasMoving = true;
        }
        driveSteeringMotor(2, 100);  // direction=2 (RIGHT), speed=100%
    } else {
        // Target is counter-clockwise - turn LEFT
        if (!wasMoving) {
            Serial.println(">>> STEER MOTOR ON - Turning LEFT");
            wasMoving = true;
        }
        driveSteeringMotor(1, 100);  // direction=1 (LEFT), speed=100%
    }
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
        else if (input == "rctest" || input == "rc") {
            // Test RC receiver inputs - read pins directly
            Serial.println("\n=== RC Receiver PWM Diagnostic Test ===");
            Serial.println("Reading RC PWM signals 10 times...\n");
            Serial.println("Time(ms) | Pin20(Steer2)µs | Pin21(Throt2)µs |");
            Serial.println("---------|-----------------|-----------------|");
            
            for (int i = 0; i < 10; i++) {
                int rc_steer = pulseIn(RC_STEER2_PIN, HIGH, 25000);
                int rc_throttle = pulseIn(RC_THROTTLE2_PIN, HIGH, 25000);
                Serial.printf("%7lu | %6d µs        | %6d µs        |\n", 
                    millis(), rc_steer, rc_throttle);
                delay(500);
            }
            Serial.println("\nTypical RC PWM ranges:");
            Serial.println("  Steering: 1000µs (left) - 1500µs (center) - 2000µs (right)");
            Serial.println("  Throttle: 1000µs (idle) - 1500µs (center) - 2000µs (full)");
            Serial.println("If values are 0, check connections and pin numbers.");
            Serial.println();
        }
        else if (input == "calenc" || input == "ce") {
            // Calibrate encoder: show raw angle and offset needed
            Serial.println("\n=== Encoder Calibration ===");
            Serial.println("Steer wheel to CENTER (neutral) position");
            delay(500);
            
            // Read raw angle
            Wire.beginTransmission(ENCODER_I2C_ADDR);
            Wire.write(0x0E);
            Wire.endTransmission();
            Wire.requestFrom(ENCODER_I2C_ADDR, 2);
            
            if (Wire.available() >= 2) {
                uint16_t rawAngle = (Wire.read() << 8) | Wire.read();
                uint16_t angleDeg = (rawAngle * 360) / 4096;
                Serial.printf("Raw angle at CENTER: %d counts = %d degrees\n", rawAngle, angleDeg);
                Serial.printf("To center at 180°, set ENCODER_ANGLE_OFFSET to: %d\n", (360 + 180 - angleDeg) % 360);
                Serial.printf("  (This is the offset needed to make current %d° → 180°)\n", angleDeg);
            } else {
                Serial.println("ERROR: Cannot read encoder");
            }
            Serial.println();
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
    
    // Calculate target angle for display
    int targetAngle = STEER_ANGLE_CENTER + (td.steeringPercent * STEER_ANGLE_RANGE) / 100;
    targetAngle = constrain(targetAngle, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
    
    int angleError = targetAngle - td.encoderAngle;
    if (angleError > 180) angleError -= 360;
    else if (angleError < -180) angleError += 360;
    
    Serial.println("\n=== Peripheral Status ===");
    Serial.println("--- Inputs ---");
    Serial.printf("  Steering:  %4d raw  %4d%%\n", td.steeringRaw, td.steeringPercent);
    Serial.printf("  Throttle:  %4d raw  %4d%%\n", td.throttleRaw, td.throttlePercent);
    Serial.printf("  Encoder:   %3d deg  (current)\n", td.encoderAngle);
    Serial.printf("  Target:    %3d deg\n", targetAngle);
    Serial.printf("  Error:     %4d deg\n", angleError);
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
    Serial.println("=== Calibration ===");
    Serial.println("  calenc (ce) - Calibrate encoder offset for 180° center");
    Serial.println("  rctest (rc) - Test RC receiver inputs (read pins 20/21)");
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
