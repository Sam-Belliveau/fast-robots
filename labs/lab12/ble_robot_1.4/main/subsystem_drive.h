#pragma once

// Drive subsystem
// One-shot drivetrain driver for offboard state estimation. The host sends
// (target_speed, target_heading) at high rate; the robot replies with the
// latest ToF readings, current time, and yaw. Heading is closed-loop via a
// local angle PID; forward speed passes straight through. If no update
// arrives within DRIVE_TIMEOUT_MS, outputs are zeroed.

#include "subsystem_serial.h"
#include "subsystem_timer.h"
#include "subsystem_ble.h"
#include "subsystem_imu.h"
#include "subsystem_tof.h"
#include "subsystem_motors.h"
#include "lib_PID.h"

namespace drive {

    // Variables

    static constexpr unsigned long DRIVE_TIMEOUT_MS = 250;

    PID heading_controller;

    float output_left = 0.0f;
    float output_right = 0.0f;

    float target_speed = 0.0f;   // PWM units, [-PWM_MAX, PWM_MAX]
    float target_heading = 0.0f; // absolute yaw in degrees

    unsigned long last_update_ms = 0;
    bool active = false;

    // Methods

    namespace methods {

        static float wrap_angle(float angle) {
            return angle - 360.0f * roundf(angle / 360.0f);
        }

        void update() {
            if (!active || (millis() - last_update_ms) > DRIVE_TIMEOUT_MS) {
                output_left = 0.0f;
                output_right = 0.0f;
                heading_controller.reset();
                active = false;
                return;
            }

            float corr = 0.0f;
            if (imu::yaw_valid) {
                float offset = wrap_angle(imu::yaw - target_heading);
                corr = heading_controller.compute(offset, 0);
            }

            output_left = target_speed - corr;
            output_right = target_speed + corr;
        }

    } // namespace methods

    // Commands

    namespace commands {

        void update_cmd(BLERequest &req) {
            float speed, heading;
            BLE_CHECK_READ(req, req.read(speed), "speed");
            BLE_CHECK_READ(req, req.read(heading), "heading");

            target_speed = speed;
            target_heading = heading;
            last_update_ms = millis();
            active = true;

            // Pull latest raw ToF samples (most recent push lives at index 0).
            int32_t tof1_t = -1, tof2_t = -1;
            int16_t tof1_d = -1, tof2_d = -1;
            if (tof::dist1.size() > 0) {
                tof1_t = (int32_t)tof::times1[0];
                tof1_d = (int16_t)tof::dist1[0];
            }
            if (tof::dist2.size() > 0) {
                tof2_t = (int32_t)tof::times2[0];
                tof2_d = (int16_t)tof::dist2[0];
            }

            BLEResponse res = req.new_response();
            res.add((int32_t)timer::methods::time_us());
            res.add(tof1_t);
            res.add(tof1_d);
            res.add(tof2_t);
            res.add(tof2_d);
            res.add(imu::yaw);
            res.end();
        }

    } // namespace commands

    // Init

    void init() {
        motors::methods::register_source(&output_left, &output_right);

        heading_controller.kP = 4.0f;
        heading_controller.kI = 0.0f;
        heading_controller.kD = 0.1f;
        heading_controller.integrator_cap = 0;
        heading_controller.integrator_range = 0;

        ble::methods::register_command(DRIVE_UPDATE, commands::update_cmd);
    }

    // Periodic

    void periodic() {
        methods::update();
    }

} // namespace drive
