#ifndef CMD_MOTORS_H
#define CMD_MOTORS_H

// Motor control command handlers
// Pin wiring: Motor 1 = A0/A1, Motor 2 = A2/A3
// Accesses globals: robot_cmd, tx_estring_value, tx_characteristic_string

#define MOTOR1_FWD A0
#define MOTOR1_REV A1
#define MOTOR2_REV A2
#define MOTOR2_FWD A3

// Artemis Nano (Apollo3) analogWrite() range: 0-255
#define PWM_MAX 255

// Calibration factor for motor 2 (multiply by this to match motor 1)
// Adjust empirically: >1.0 if motor 2 is slower, <1.0 if faster
float motor2_cal = 1.0;

// Auto-shutoff: motors stop after this many ms (0 = disabled)
unsigned long motor_timeout_ms = 0;
unsigned long motor_start_time = 0;
bool motors_active = false;

void motors_stop() {
  analogWrite(MOTOR1_FWD, 0);
  analogWrite(MOTOR1_REV, 0);
  analogWrite(MOTOR2_FWD, 0);
  analogWrite(MOTOR2_REV, 0);
  motors_active = false;
}

void motors_init() {
  pinMode(MOTOR1_FWD, OUTPUT);
  pinMode(MOTOR1_REV, OUTPUT);
  pinMode(MOTOR2_FWD, OUTPUT);
  pinMode(MOTOR2_REV, OUTPUT);

  // analogWrite(MOTOR1_FWD, 100);
  for (int k = 50; k <= 200; k += 50) {
    analogWrite(MOTOR2_FWD, k);
    delay(3000);
    motors_stop();
    delay(1000);
    analogWrite(MOTOR1_FWD, k);
    delay(3000);
    motors_stop();
    delay(1000);
  }
}

// Set a single motor: speed in [-PWM_MAX, PWM_MAX]
// Always zero the inactive pin BEFORE setting the active pin
// to guarantee both pins are never driven high simultaneously.
static void set_motor(int fwd_pin, int rev_pin, int speed) {
  Serial.print(F("  set_motor fwd="));
  Serial.print(fwd_pin);
  Serial.print(F(" rev="));
  Serial.print(rev_pin);
  Serial.print(F(" speed="));
  Serial.println(speed);

  if (speed > 0) {
    analogWrite(rev_pin, 0);
    analogWrite(fwd_pin, speed);
  } else if (speed < 0) {
    analogWrite(fwd_pin, 0);
    analogWrite(rev_pin, -speed);
  } else {
    analogWrite(fwd_pin, 0);
    analogWrite(rev_pin, 0);
  }
}

// Apply speeds with calibration and start timeout tracking
static void set_motors(int left, int right) {
  // Apply calibration to motor 2
  int cal_right = constrain((int)(right * motor2_cal), -PWM_MAX, PWM_MAX);
  set_motor(MOTOR1_FWD, MOTOR1_REV, left);
  set_motor(MOTOR2_FWD, MOTOR2_REV, cal_right);

  motors_active = (left != 0 || right != 0);
  if (motors_active) {
    motor_start_time = millis();
  }
}

// Call from loop() to enforce auto-shutoff
void check_motor_timeout() {
  if (motors_active && motor_timeout_ms > 0) {
    if (millis() - motor_start_time >= motor_timeout_ms) {
      motors_stop();
      Serial.println(F("Motor timeout - stopped"));
    }
  }
}

//////////// Motor Command Handlers ////////////

// MOTOR_CMD: left_speed|right_speed
// Speeds in [-PWM_MAX, PWM_MAX], negative = reverse
void cmd_motor() {
  int left, right;
  if (!robot_cmd.get_next_value(left))
    return;
  left = constrain(left, -PWM_MAX, PWM_MAX);
  if (!robot_cmd.get_next_value(right))
    return;
  right = constrain(right, -PWM_MAX, PWM_MAX);
  set_motors(left, right);
  Serial.print(F("Motors: L="));
  Serial.print(left);
  Serial.print(F(" R="));
  Serial.print(right);
  Serial.print(F(" PWM_MAX="));
  Serial.println(PWM_MAX);
}

// MOTOR_STOP: immediately stop both motors
void cmd_motor_stop() {
  motors_stop();
  Serial.println(F("Motors stopped"));
}

// MOTOR_CAL: calibration_factor (float)
void cmd_motor_cal() {
  float cal;
  if (!robot_cmd.get_next_value(cal))
    return;
  motor2_cal = cal;
  tx_estring_value.clear();
  tx_estring_value.append("CAL:");
  tx_estring_value.append(motor2_cal);
  tx_characteristic_string.writeValue(tx_estring_value.c_str());
  Serial.print(F("Motor2 calibration: "));
  Serial.println(motor2_cal);
}

// MOTOR_TIMEOUT: timeout_ms (0 = disable)
void cmd_motor_timeout() {
  int timeout;
  if (!robot_cmd.get_next_value(timeout))
    return;
  motor_timeout_ms = (unsigned long)timeout;
  Serial.print(F("Motor timeout: "));
  Serial.print(motor_timeout_ms);
  Serial.println(F("ms"));
}

void register_motor_commands() {
  register_command(MOTOR_CMD, cmd_motor);
  register_command(MOTOR_STOP, cmd_motor_stop);
  register_command(MOTOR_CAL, cmd_motor_cal);
  register_command(MOTOR_TIMEOUT, cmd_motor_timeout);
}

#endif // CMD_MOTORS_H
