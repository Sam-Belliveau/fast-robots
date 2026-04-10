#pragma once

// Gamepad subsystem
// Owns: direct motor commands from BLE (MOTOR_CMD, MOTOR_STOP, MOTOR_TIMEOUT).
// Writes output_left / output_right for the motors subsystem to pull.

#include "subsystem_serial.h"
#include "subsystem_timer.h"
#include "subsystem_ble.h"
#include "subsystem_motors.h"

namespace gamepad {

    // Variables

    float output_left = 0.0f;
    float output_right = 0.0f;

    unsigned long last_set = 0;
    unsigned long watchdog_timeout_us = 2500000; // 2.5 seconds

    // Methods

    namespace methods {

        void set(float left, float right) {
            output_left = left;
            output_right = right;
            last_set = timer::methods::time_us();
        }

        void stop() {
            output_left = 0.0f;
            output_right = 0.0f;
            last_set = 0;
        }

        void update() {
            unsigned long now = timer::methods::time_us();
            if (last_set != 0 && (now - last_set) > watchdog_timeout_us) {
                stop();
            }
        }

    } // namespace methods

    // Commands

    namespace commands {

        void motor(BLERequest &req) {
            int32_t left, right;
            BLE_CHECK_READ(req, req.read(left), "left");
            BLE_CHECK_READ(req, req.read(right), "right");
            methods::set(left, right);
            INFO_PRINT(F("Gamepad: L="));
            INFO_PRINT(left);
            INFO_PRINT(F(" R="));
            INFO_PRINTLN(right);
            req.new_response().end();
        }

        void motor_stop(BLERequest &req) {
            methods::stop();
            INFO_PRINTLN(F("Gamepad stopped"));
            req.new_response().end();
        }

        void motor_timeout(BLERequest &req) {
            int32_t timeout;
            BLE_CHECK_READ(req, req.read(timeout), "timeout");
            watchdog_timeout_us = (unsigned long)timeout * 1000UL;
            INFO_PRINT(F("Gamepad timeout: "));
            INFO_PRINTLN(timeout);
            req.new_response().end();
        }

    } // namespace commands

    // Init

    void init() {
        motors::methods::register_source(&output_left, &output_right);

        ble::methods::register_command(MOTOR_CMD, commands::motor);
        ble::methods::register_command(MOTOR_STOP, commands::motor_stop);
        ble::methods::register_command(MOTOR_TIMEOUT, commands::motor_timeout);
    }

    // Periodic

    void periodic() {
        methods::update();
    }

} // namespace gamepad
