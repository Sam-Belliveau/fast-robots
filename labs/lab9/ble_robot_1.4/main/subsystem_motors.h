#pragma once

// Motors subsystem
// Owns: motor pins, calibration, motor control with smoothing.
// Pull-based: other subsystems register their output variables as sources.
// Each periodic tick, motors sums all sources, clamps, and drives hardware.

#include "subsystem_serial.h"
#include "subsystem_timer.h"
#include "subsystem_ble.h"
#include "lib_DeSticker.h"

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

    DeSticker desticker_left(120, 160);
    DeSticker desticker_right(80, 120);

    // Source registration: each source provides pointers to its output pair.
    static constexpr int MAX_SOURCES = 8;
    struct Source {
        float *left;
        float *right;
    };
    Source sources[MAX_SOURCES];
    int num_sources = 0;

    // Methods

    namespace methods {

        // Register a motor source. Call during init().
        void register_source(float *left, float *right) {
            if (num_sources < MAX_SOURCES) {
                sources[num_sources++] = {left, right};
            }
        }

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
            set_pwm(0, 0);
        }

        static void update() {
            // Normalize each source to [-1, 1] then sum
            float sum_left = 0.0f;
            float sum_right = 0.0f;
            for (int i = 0; i < num_sources; i++) {
                float l = *sources[i].left;
                float r = *sources[i].right;
                float m = max(max(fabsf(l), fabsf(r)), (float)PWM_MAX);
                sum_left += l / m;
                sum_right += r / m;
            }

            // Normalize final sum to [-PWM_MAX, PWM_MAX]
            float m = max(max(fabsf(sum_left), fabsf(sum_right)), 1.0f);
            target_left = sum_left / m * PWM_MAX;
            target_right = sum_right / m * PWM_MAX;

            current_left += (target_left - current_left) * motor_alpha;
            current_right += (target_right - current_right) * motor_alpha;

            int left_out = desticker_left.update((int)(current_left));
            int right_out = desticker_right.update((int)(current_right));
            set_pwm(left_out, right_out);
        }

    } // namespace methods

    // Commands

    namespace commands {

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

            const char *names[] = {"L_FWD", "R_FWD", "L_REV", "R_REV"};

            for (int i = 0; i < 4; i++) {
                if (i > 0)
                    delay(1000);
                INFO_PRINT(F("\tTesting "));
                INFO_PRINTLN(names[i]);
                for (int k = 0; k <= 255; k++) {
                    int left = (i == 0) ? k : (i == 2) ? -k : 0;
                    int right = (i == 1) ? k : (i == 3) ? -k : 0;
                    methods::set_pwm(left, right);
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

        ble::methods::register_command(MOTOR_CAL, commands::motor_cal);
        ble::methods::register_command(MOTOR_TEST, commands::motor_test);
    }

    // Periodic

    void periodic() {
        methods::update();
    }

} // namespace motors
