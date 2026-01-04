/*******************************************************************************
 * Teensy 4.1 Motor Driver Test - V2.1 PCB
 * 
 * Tests all 3 motor drivers (Steering, Front, Rear) together
 * Throttle pot = motor speed (0-100%)
 * Shifter = direction (FWD / REV / NEUTRAL)
 * 
 * Pure Teensy code - NO OTA, NO I2C, NO Telemetry
 ******************************************************************************/

#include <Arduino.h>

//====== PIN DEFINITIONS ======

// Throttle input
#define PIN_THROTTLE 23

// Shifter inputs (active LOW)
#define PIN_FWD 29
#define PIN_REV 28

// Steering motor
#define PIN_STEER_LPWM 2
#define PIN_STEER_RPWM 3
#define PIN_STEER_L_EN 4
#define PIN_STEER_R_EN 5

// Front motor
#define PIN_FRONT_LPWM 6
#define PIN_FRONT_RPWM 9
#define PIN_FRONT_L_EN 10
#define PIN_FRONT_R_EN 11

// Rear motor
#define PIN_REAR_LPWM 12
#define PIN_REAR_RPWM 13
#define PIN_REAR_L_EN 32
#define PIN_REAR_R_EN 33

//====== THROTTLE CALIBRATION ======
#define THROTTLE_MIN 249
#define THROTTLE_MAX 790
#define THROTTLE_DEADZONE 30

//====== SETUP ======
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n========================================");
    Serial.println("  MOTOR DRIVER TEST - Teensy 4.1");
    Serial.println("========================================");
    Serial.println("3 Motors: Steering, Front, Rear");
    Serial.println("Throttle = Speed (0-100%)");
    Serial.println("Shifter = Direction (FWD/REV/NEUTRAL)");
    Serial.println();
    
    // Configure shifter inputs
    pinMode(PIN_FWD, INPUT_PULLUP);
    pinMode(PIN_REV, INPUT_PULLUP);
    
    // Configure all enable pins
    pinMode(PIN_STEER_L_EN, OUTPUT);
    pinMode(PIN_STEER_R_EN, OUTPUT);
    pinMode(PIN_FRONT_L_EN, OUTPUT);
    pinMode(PIN_FRONT_R_EN, OUTPUT);
    pinMode(PIN_REAR_L_EN, OUTPUT);
    pinMode(PIN_REAR_R_EN, OUTPUT);
    
    // Configure all PWM pins
    pinMode(PIN_STEER_LPWM, OUTPUT);
    pinMode(PIN_STEER_RPWM, OUTPUT);
    pinMode(PIN_FRONT_LPWM, OUTPUT);
    pinMode(PIN_FRONT_RPWM, OUTPUT);
    pinMode(PIN_REAR_LPWM, OUTPUT);
    pinMode(PIN_REAR_RPWM, OUTPUT);
    
    // Set PWM frequency
    analogWriteFrequency(PIN_STEER_LPWM, 20000);
    analogWriteFrequency(PIN_STEER_RPWM, 20000);
    analogWriteFrequency(PIN_FRONT_LPWM, 20000);
    analogWriteFrequency(PIN_FRONT_RPWM, 20000);
    analogWriteFrequency(PIN_REAR_LPWM, 20000);
    analogWriteFrequency(PIN_REAR_RPWM, 20000);
    
    // All motors off initially
    allMotorsOff();
    
    Serial.println("Ready. Move throttle and shifter to test...\n");
}

//====== MAIN LOOP ======
void loop() {
    // Read throttle
    int throttleRaw = analogRead(PIN_THROTTLE);
    int throttlePercent = 0;
    
    if (throttleRaw > THROTTLE_MIN + THROTTLE_DEADZONE) {
        throttlePercent = map(throttleRaw, 
                              THROTTLE_MIN + THROTTLE_DEADZONE,
                              THROTTLE_MAX, 
                              0, 100);
        throttlePercent = constrain(throttlePercent, 0, 100);
    }
    
    // Convert to PWM (0-255)
    int pwmValue = map(throttlePercent, 0, 100, 0, 255);
    
    // Read shifter (active LOW)
    bool isFwd = (digitalRead(PIN_FWD) == LOW);
    bool isRev = (digitalRead(PIN_REV) == LOW);
    
    // Determine gear
    int gear = 0;  // 0=NEUTRAL, 1=FORWARD, 2=REVERSE
    if (isFwd) gear = 1;
    else if (isRev) gear = 2;
    
    // Apply motor control
    driveAllMotors(gear, pwmValue);
    
    // Print every 200ms
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 200) {
        lastPrint = millis();
        
        const char* gearStr = (gear == 1) ? "FWD" : (gear == 2) ? "REV" : "NEU";
        Serial.printf("Throttle: %3d%% (raw=%3d) | PWM: %3d | Gear: %s | FWD_pin: %d | REV_pin: %d | EN_pins: L_EN=%d R_EN=%d\n",
                     throttlePercent, throttleRaw, pwmValue, gearStr,
                     digitalRead(PIN_FWD), digitalRead(PIN_REV),
                     digitalRead(PIN_STEER_L_EN), digitalRead(PIN_STEER_R_EN));
    }
}

//====== DRIVE ALL MOTORS ======
void driveAllMotors(int gear, int pwmValue) {
    // Enable all drivers
    digitalWrite(PIN_STEER_L_EN, HIGH);
    digitalWrite(PIN_STEER_R_EN, HIGH);
    digitalWrite(PIN_FRONT_L_EN, HIGH);
    digitalWrite(PIN_FRONT_R_EN, HIGH);
    digitalWrite(PIN_REAR_L_EN, HIGH);
    digitalWrite(PIN_REAR_R_EN, HIGH);
    
    if (pwmValue == 0) {
        // No throttle - all off
        allMotorsOff();
    } 
    else if (gear == 1) {
        // FORWARD - apply to L_PWM pins
        analogWrite(PIN_STEER_LPWM, pwmValue);
        analogWrite(PIN_STEER_RPWM, 0);
        analogWrite(PIN_FRONT_LPWM, pwmValue);
        analogWrite(PIN_FRONT_RPWM, 0);
        analogWrite(PIN_REAR_LPWM, pwmValue);
        analogWrite(PIN_REAR_RPWM, 0);
    } 
    else if (gear == 2) {
        // REVERSE - apply to R_PWM pins
        analogWrite(PIN_STEER_LPWM, 0);
        analogWrite(PIN_STEER_RPWM, pwmValue);
        analogWrite(PIN_FRONT_LPWM, 0);
        analogWrite(PIN_FRONT_RPWM, pwmValue);
        analogWrite(PIN_REAR_LPWM, 0);
        analogWrite(PIN_REAR_RPWM, pwmValue);
    } 
    else {
        // NEUTRAL - all off
        allMotorsOff();
    }
}

//====== ALL MOTORS OFF ======
void allMotorsOff() {
    analogWrite(PIN_STEER_LPWM, 0);
    analogWrite(PIN_STEER_RPWM, 0);
    analogWrite(PIN_FRONT_LPWM, 0);
    analogWrite(PIN_FRONT_RPWM, 0);
    analogWrite(PIN_REAR_LPWM, 0);
    analogWrite(PIN_REAR_RPWM, 0);
}
