#pragma once

// Motors subsystem
// Owns: motor pins, calibration, timeout, motor control functions.

#include "subsystem_serial.h"
#include "subsystem_ble.h"

#define MOTOR1_FWD A1
#define MOTOR1_REV A0
#define MOTOR2_FWD A16
#define MOTOR2_REV A15

#define PWM_MAX 255

namespace motors {

    // Variables

    float cal = 1.0;
    unsigned long timeout_ms = 1000;
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

        void set(int left, int right) {
            left = constrain(left, -PWM_MAX, PWM_MAX);
            right = constrain((int)(right * cal), -PWM_MAX, PWM_MAX);

            if (left > 0) {
                analogWrite(MOTOR1_FWD, left);
                analogWrite(MOTOR1_REV, 0);
            } else if (left < 0) {
                analogWrite(MOTOR1_REV, -left);
                analogWrite(MOTOR1_FWD, 0);
            } else {
                analogWrite(MOTOR1_FWD, 0);
                analogWrite(MOTOR1_REV, 0);
            }

            if (right > 0) {
                analogWrite(MOTOR2_FWD, right);
                analogWrite(MOTOR2_REV, 0);
            } else if (right < 0) {
                analogWrite(MOTOR2_REV, -right);
                analogWrite(MOTOR2_FWD, 0);
            } else {
                analogWrite(MOTOR2_FWD, 0);
                analogWrite(MOTOR2_REV, 0);
            }

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

        void motor_test(BLERequest &req) {
            methods::stop();
            SERIAL_PRINTLN(F("Starting motor test...\n"));
            SERIAL_PRINTLN(F("\tTesting MOTOR2_FWD..."));
            for (int k = 0; k <= 255; k += 1) {
                analogWrite(MOTOR2_FWD, k);
                delay(10);
            }
            SERIAL_PRINTLN(F("\tFinished MOTOR2_FWD!\n"));
            methods::stop();
            delay(1000);
            SERIAL_PRINTLN(F("\tTesting MOTOR1_FWD..."));
            for (int k = 0; k <= 255; k += 1) {
                analogWrite(MOTOR1_FWD, k);
                delay(10);
            }
            SERIAL_PRINTLN(F("\tFinished MOTOR1_FWD!\n"));
            methods::stop();
            delay(1000);
            SERIAL_PRINTLN(F("\tTesting MOTOR2_REV..."));
            for (int k = 0; k <= 255; k += 1) {
                analogWrite(MOTOR2_REV, k);
                delay(10);
            }
            SERIAL_PRINTLN(F("\tFinished MOTOR2_REV!\n"));
            methods::stop();
            delay(1000);
            SERIAL_PRINTLN(F("\tTesting MOTOR1_REV..."));
            for (int k = 0; k <= 255; k += 1) {
                analogWrite(MOTOR1_REV, k);
                delay(10);
            }
            SERIAL_PRINTLN(F("\tFinished MOTOR1_REV!\n"));
            methods::stop();
            SERIAL_PRINTLN(F("Motor test complete"));
            req.new_response().end();
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
        analogReadResolution(8);
        analogWriteResolution(8);

        analogWriteFrequency(1000);

        pinMode(MOTOR1_FWD, OUTPUT);
        pinMode(MOTOR1_REV, OUTPUT);
        pinMode(MOTOR2_FWD, OUTPUT);
        pinMode(MOTOR2_REV, OUTPUT);

        methods::stop();

        ble::methods::register_command(MOTOR_CMD, commands::motor);
        ble::methods::register_command(MOTOR_STOP, commands::motor_stop);
        ble::methods::register_command(MOTOR_CAL, commands::motor_cal);
        ble::methods::register_command(MOTOR_TIMEOUT, commands::motor_timeout);
        ble::methods::register_command(MOTOR_TEST, commands::motor_test);
    }

    // Periodic

    void periodic() {
        methods::check_timeout();
    }

} // namespace motors
