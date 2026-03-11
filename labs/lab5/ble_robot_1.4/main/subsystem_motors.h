#pragma once

// Motors subsystem
// Owns: motor pins, calibration, timeout, motor control functions.

#include "subsystem_serial.h"
#include "subsystem_ble.h"

#define MOTOR1_FWD A0
#define MOTOR1_REV A1
#define MOTOR2_FWD A15
#define MOTOR2_REV A16

#define PWM_MAX 255

namespace motors {

    // Variables

    float cal = 1.0;
    unsigned long timeout_ms = 0;
    unsigned long start_time = 0;
    bool active = false;

    // Methods

    namespace methods {

        void stop() {
            analogWrite(MOTOR1_FWD, 0);
            analogWrite(MOTOR1_REV, 0);
            analogWrite(MOTOR2_FWD, 0);
            analogWrite(MOTOR2_REV, 0);
            active = false;
        }

        static void set_motor(int fwd_pin, int rev_pin, int speed) {
            SERIAL_PRINT(F("  set_motor fwd="));
            SERIAL_PRINT(fwd_pin);
            SERIAL_PRINT(F(" rev="));
            SERIAL_PRINT(rev_pin);
            SERIAL_PRINT(F(" speed="));
            SERIAL_PRINTLN(speed);

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

        void set(int left, int right) {
            int cal_right = constrain((int)(right * cal), -PWM_MAX, PWM_MAX);
            set_motor(MOTOR1_FWD, MOTOR1_REV, left);
            set_motor(MOTOR2_FWD, MOTOR2_REV, cal_right);

            active = (left != 0 || right != 0);
            if (active) {
                start_time = millis();
            }
        }

        void check_timeout() {
            if (active && timeout_ms > 0) {
                if (millis() - start_time >= timeout_ms) {
                    stop();
                    SERIAL_PRINTLN(F("Motor timeout - stopped"));
                }
            }
        }

    } // namespace methods

    // Commands

    namespace commands {

        void motor(BLERequest &req) {
            int32_t left, right;
            BLE_CHECK_READ(req, req.read(left), "left");
            BLE_CHECK_READ(req, req.read(right), "right");
            left = constrain(left, -PWM_MAX, PWM_MAX);
            right = constrain(right, -PWM_MAX, PWM_MAX);
            methods::set(left, right);
            SERIAL_PRINT(F("Motors: L="));
            SERIAL_PRINT(left);
            SERIAL_PRINT(F(" R="));
            SERIAL_PRINT(right);
            SERIAL_PRINT(F(" PWM_MAX="));
            SERIAL_PRINTLN(PWM_MAX);
            req.new_response().end();
        }

        void motor_stop(BLERequest &req) {
            methods::stop();
            SERIAL_PRINTLN(F("Motors stopped"));
            req.new_response().end();
        }

        void motor_cal(BLERequest &req) {
            float c;
            BLE_CHECK_READ(req, req.read(c), "cal");
            cal = c;

            BLEResponse res = req.new_response();
            res.add(cal);
            res.end();

            SERIAL_PRINT(F("Motor2 calibration: "));
            SERIAL_PRINTLN(cal);
        }

        void motor_timeout(BLERequest &req) {
            int32_t timeout;
            BLE_CHECK_READ(req, req.read(timeout), "timeout");
            timeout_ms = (unsigned long)timeout;
            SERIAL_PRINT(F("Motor timeout: "));
            SERIAL_PRINT(timeout_ms);
            SERIAL_PRINTLN(F("ms"));
            req.new_response().end();
        }

    } // namespace commands

    // Init

    void init() {
        pinMode(MOTOR1_FWD, OUTPUT);
        pinMode(MOTOR1_REV, OUTPUT);
        pinMode(MOTOR2_FWD, OUTPUT);
        pinMode(MOTOR2_REV, OUTPUT);

        methods::stop();

        ble::methods::register_command(MOTOR_CMD, commands::motor);
        ble::methods::register_command(MOTOR_STOP, commands::motor_stop);
        ble::methods::register_command(MOTOR_CAL, commands::motor_cal);
        ble::methods::register_command(MOTOR_TIMEOUT, commands::motor_timeout);
    }

    // Periodic

    void periodic() {
        methods::check_timeout();
    }

} // namespace motors
