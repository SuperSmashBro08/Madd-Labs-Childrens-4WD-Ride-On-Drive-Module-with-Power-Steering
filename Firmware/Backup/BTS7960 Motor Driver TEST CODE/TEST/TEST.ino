// ===== FRONT & STEER MOTOR DRIVERS: FWD/REV + Pedal on Teensy 4.1 =====
// BTS7960-style drivers

// -------- FRONT motor driver pins --------
const int PIN_R_EN  = 11;
const int PIN_L_EN  = 10;
const int PIN_R_PWM = 9;   // FRONT, one direction
const int PIN_L_PWM = 6;   // FRONT, other direction

// -------- STEER motor driver pins --------
const int PIN_R2_EN  = 5;  // STEER R_EN (through level shifter)
const int PIN_L2_EN  = 4;  // STEER L_EN (through level shifter)
const int PIN_R2_PWM = 3;   // STEER, one direction (reverse)
const int PIN_L2_PWM = 2;   // STEER, other direction (forward)

// -------- Inputs --------
const int PIN_PEDAL = 23;   // A13 foot pedal
const int PIN_FWD   = 28;   // Shifter FORWARD contact (to GND when forward)
const int PIN_REV   = 29;   // Shifter REVERSE contact (to GND when reverse)

// -------- Pedal calibration --------
// 100 is your "off" point.
// We'll use a small deadband above it and map up to ~900.
const int PEDAL_RAW_MIN   = 100;  // raw at "off"
const int PEDAL_DEADBAND  = 30;    // small deadband above min
const int PEDAL_RAW_MAX   = 900;  // raw at full pedal

// Map pedal raw reading to 0..255 PWM
int computePedalPWM(int raw)
{
  // Below off+deadband → force 0
  if (raw <= PEDAL_RAW_MIN + PEDAL_DEADBAND) {
    return 0;
  }

  // Clamp into [PEDAL_RAW_MIN, PEDAL_RAW_MAX]
  if (raw < PEDAL_RAW_MIN) raw = PEDAL_RAW_MIN;
  if (raw > PEDAL_RAW_MAX) raw = PEDAL_RAW_MAX;

  // Map from (min+deadband .. max) → 0..255
  int pwm = map(raw,
                PEDAL_RAW_MIN + PEDAL_DEADBAND,
                PEDAL_RAW_MAX,
                0,
                255);

  if (pwm < 0)   pwm = 0;
  if (pwm > 255) pwm = 255;
  return pwm;
}

void setup()
{
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println(F("=== FRONT+Steer MOTOR: Pedal + FWD/REV (11 & 12, PWM 13/22 & 2/3) ==="));

  // ---- FRONT motor driver pins ----
  pinMode(PIN_R_EN,  OUTPUT);
  pinMode(PIN_L_EN,  OUTPUT);
  pinMode(PIN_R_PWM, OUTPUT);
  pinMode(PIN_L_PWM, OUTPUT);

  digitalWrite(PIN_R_EN, HIGH);
  digitalWrite(PIN_L_EN, HIGH);

  analogWrite(PIN_R_PWM, 0);
  analogWrite(PIN_L_PWM, 0);

  // ---- STEER motor driver pins ----
  pinMode(PIN_R2_EN,  OUTPUT);
  pinMode(PIN_L2_EN,  OUTPUT);
  pinMode(PIN_R2_PWM, OUTPUT);
  pinMode(PIN_L2_PWM, OUTPUT);

  // Enable both sides of Steer BTS7960 (through level shifter)
  digitalWrite(PIN_R2_EN, HIGH);
  digitalWrite(PIN_L2_EN, HIGH);

  analogWrite(PIN_R2_PWM, 0);
  analogWrite(PIN_L2_PWM, 0);

  // ---- Shifter inputs ----
  // Internal pullups: HIGH when idle, LOW when switch connects to GND.
  pinMode(PIN_FWD, INPUT_PULLUP);
  pinMode(PIN_REV, INPUT_PULLUP);

  // ---- ADC setup for pedal ----
  analogReadResolution(12);  // 0..4095
  analogReadAveraging(8);

  // ---- PWM frequency on motor pins ----
  analogWriteFrequency(PIN_R_PWM,  20000);
  analogWriteFrequency(PIN_L_PWM,  20000);
  analogWriteFrequency(PIN_R2_PWM, 20000);
  analogWriteFrequency(PIN_L2_PWM, 20000);
}

void loop()
{
  // ----- Pedal read -----
  int rawPedal = analogRead(PIN_PEDAL);
  int pwm = computePedalPWM(rawPedal);

  // ----- Shifter read (LOW = active) -----
  int rawFwd = digitalRead(PIN_FWD); // 0 = forward selected
  int rawRev = digitalRead(PIN_REV); // 0 = reverse selected

  enum GearState { GEAR_NEUTRAL, GEAR_FORWARD, GEAR_REVERSE };
  GearState gear = GEAR_NEUTRAL;

  // Logic with INPUT_PULLUP:
  //   Forward: FWD LOW, REV HIGH
  //   Reverse: FWD HIGH, REV LOW
  //   Neutral: anything else (both HIGH or both LOW)
  if (rawFwd == LOW && rawRev == HIGH) {
    gear = GEAR_FORWARD;
  } else if (rawFwd == HIGH && rawRev == LOW) {
    gear = GEAR_REVERSE;
  } else {
    gear = GEAR_NEUTRAL;
  }

  // ----- Apply motor outputs (FRONT + Steer) -----
  switch (gear) {
    case GEAR_FORWARD:
      // Forward direction:
      //   FRONT:  L_PWM (13)
      //   STEER:   L2_PWM (2)
      analogWrite(PIN_L_PWM,  pwm);
      analogWrite(PIN_R_PWM,  0);
      analogWrite(PIN_L2_PWM, pwm);
      analogWrite(PIN_R2_PWM, 0);
      break;

    case GEAR_REVERSE:
      // Reverse direction:
      //   FRONT:  R_PWM (22)
      //   STEER:   R2_PWM (3)
      analogWrite(PIN_L_PWM,  0);
      analogWrite(PIN_R_PWM,  pwm);
      analogWrite(PIN_L2_PWM, 0);
      analogWrite(PIN_R2_PWM, pwm);
      break;

    case GEAR_NEUTRAL:
    default:
      // All off
      analogWrite(PIN_R_PWM,  0);
      analogWrite(PIN_L_PWM,  0);
      analogWrite(PIN_R2_PWM, 0);
      analogWrite(PIN_L2_PWM, 0);
      break;
  }

  // ----- Debug print -----
  static uint32_t lastPrint = 0;
  uint32_t now = millis();
  if (now - lastPrint >= 200) {
    lastPrint = now;

    Serial.print(F("rawPedal="));
    Serial.print(rawPedal);
    Serial.print(F(" pwm="));
    Serial.print(pwm);

    Serial.print(F("  rawFwd="));
    Serial.print(rawFwd);
    Serial.print(F(" rawRev="));
    Serial.print(rawRev);
    Serial.print(F(" gear="));
    if (gear == GEAR_FORWARD)      Serial.print("FORWARD");
    else if (gear == GEAR_REVERSE) Serial.print("REVERSE");
    else                           Serial.print("NEUTRAL");

    Serial.println();
  }
}
