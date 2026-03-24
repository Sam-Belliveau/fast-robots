#pragma once

// Motors subsystem
// Owns: motor pins, calibration, motor control functions with smoothing.

#include "subsystem_serial.h"
#include "subsystem_timer.h"
#include "subsystem_ble.h"

#define MOTOR1_FWD A1
#define MOTOR1_REV A0
#define MOTOR2_FWD A16
#define MOTOR2_REV A15

#define PWM_MAX 255

namespace motors {

    // Variables

    float cal = 1.0;

    float target_left = 0.0;
    float target_right = 0.0;

    float current_left = 0.0;
    float current_right = 0.0;

    constexpr float motor_rc = 0.02f; // seconds
    constexpr float motor_alpha = timer::methods::alpha(motor_rc);

    unsigned long last_set = 0;
    const unsigned long watchdog_timeout_us = 500000; // 0.1 seconds

    // Methods

    namespace methods {

        static void set_pwm(int left, int right) {
            left = constrain(left, -PWM_MAX, PWM_MAX);
            right = constrain(right, -PWM_MAX, PWM_MAX);

            if (left > 0) {
                analogWrite(MOTOR1_FWD, left);
                analogWrite(MOTOR1_REV, 0);
            } else {
                analogWrite(MOTOR1_FWD, 0);
                analogWrite(MOTOR1_REV, -left);
            }
            if (right > 0) {
                analogWrite(MOTOR2_FWD, right);
                analogWrite(MOTOR2_REV, 0);
            } else {
                analogWrite(MOTOR2_FWD, 0);
                analogWrite(MOTOR2_REV, -right);
            }
        }

        void stop() {
            target_left = 0.0;
            target_right = 0.0;
            current_left = 0.0;
            current_right = 0.0;
            last_set = 0;
            set_pwm(0, 0);
        }

        void set(float left, float right) {
            target_left = constrain(left, -PWM_MAX, PWM_MAX);
            target_right = constrain(right, -PWM_MAX, PWM_MAX);
            last_set = timer::methods::time_us();
        }

        static void update() {
            unsigned long now = timer::methods::time_us();

            // Watchdog: stop motors if set() hasn't been called recently
            if (last_set != 0 && (now - last_set) > watchdog_timeout_us) {
                stop();
                return;
            }

            current_left += (target_left - current_left) * motor_alpha;
            current_right += (target_right - current_right) * motor_alpha;

            set_pwm((int)(current_left), (int)(current_right * cal));
        }

    } // namespace methods

    // Commands

    namespace commands {

        void motor(BLERequest &req) {
            int32_t left, right;
            BLE_CHECK_READ(req, req.read(left), "left");
            BLE_CHECK_READ(req, req.read(right), "right");
            methods::set(left, right);
            INFO_PRINT(F("Motors: L="));
            INFO_PRINT(left);
            INFO_PRINT(F(" R="));
            INFO_PRINTLN(right);
            req.new_response().end();
        }

        void motor_stop(BLERequest &req) {
            methods::stop();
            INFO_PRINTLN(F("Motors stopped"));
            req.new_response().end();
        }

        void motor_cal(BLERequest &req) {
            float c;
            BLE_CHECK_READ(req, req.read(c), "cal");
            cal = c;

            BLEResponse res = req.new_response();
            res.add(cal);
            res.end();

            INFO_PRINT(F("Motor2 calibration: "));
            INFO_PRINTLN(cal);
        }

        void motor_test(BLERequest &req) {
            methods::stop();
            INFO_PRINTLN(F("Starting motor test..."));

            const int pins[] = {MOTOR2_FWD, MOTOR1_FWD, MOTOR2_REV, MOTOR1_REV};
            const char *names[] = {
                "MOTOR2_FWD", "MOTOR1_FWD", "MOTOR2_REV", "MOTOR1_REV"
            };

            for (int i = 0; i < 4; i++) {
                if (i > 0)
                    delay(1000);
                INFO_PRINT(F("\tTesting "));
                INFO_PRINTLN(names[i]);
                for (int k = 0; k <= 255; k++) {
                    analogWrite(pins[i], k);
                    delay(10);
                }
                methods::stop();
            }

            INFO_PRINTLN(F("Motor test complete"));
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
        ble::methods::register_command(MOTOR_TEST, commands::motor_test);
    }

    // Periodic

    void periodic() {
        methods::update();
    }

} // namespace motors
