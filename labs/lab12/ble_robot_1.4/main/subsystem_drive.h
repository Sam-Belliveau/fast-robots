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

    // Heading-error speed weighting. forward = target_speed *
    // exp(-(offset/align_tol_deg)^2), computed every periodic tick against the
    // live yaw so the rolloff tracks the freshest IMU sample instead of waiting
    // for the next BLE round-trip. Settable from the host via
    // DRIVE_SET_ALIGN_TOL so the robot's rolloff and the host's predict-step
    // Gaussian stay in sync without duplicated tuning constants.
    float align_tol_deg = 10.0f;

    PID heading_controller;

    // Two independent motor sources. The motors subsystem normalizes each
    // source to [-1, 1] before summing, so registering forward and turn
    // separately lets the turn term saturate the differential without the
    // forward term scaling it down (and vice versa).
    float forward = 0.0f; // common-mode (drives both wheels)
    float turn_left = 0.0f;
    float turn_right = 0.0f;

    float target_speed = 0.0f;   // PWM units, [-PWM_MAX, PWM_MAX]
    float target_heading = 0.0f; // absolute yaw in degrees

    unsigned long last_update_ms = 0;
    bool active = false;

    // Robot-side odometry integrator. Each periodic tick accumulates
    //   dx_pwm_s += forward * cos(yaw) * dt
    //   dy_pwm_s += forward * sin(yaw) * dt
    //   abs_pwm_s += |forward| * dt
    // using the post-rolloff `forward` value and the live IMU yaw, so
    // the host gets a faithful PWM-seconds displacement at firmware
    // loop rate (~128 Hz) instead of approximating it from a single
    // RTT-rate sample. Reset each time we ship a DriveUpdate response.
    float dx_pwm_s = 0.0f;
    float dy_pwm_s = 0.0f;
    float abs_pwm_s = 0.0f;

    // Methods

    namespace methods {

        static float wrap_angle(float angle) {
            return angle - 360.0f * roundf(angle / 360.0f);
        }

        void update() {
            if (!active || (millis() - last_update_ms) > DRIVE_TIMEOUT_MS) {
                forward = 0.0f;
                turn_left = 0.0f;
                turn_right = 0.0f;
                heading_controller.reset();
                active = false;
                return;
            }

            constexpr float dt = timer::PERIOD_S;
            float offset = wrap_angle(imu::yaw - target_heading);

            // Gaussian rolloff on heading error so the bot does not
            // drive forward while badly mis-aligned. Computed against
            // live yaw for minimal latency vs doing it on the host.
            float a = offset / align_tol_deg;
            float t = expf(-a * a);

            forward = target_speed * t;

            if (imu::yaw_valid) {
                // Use the gyro yaw rate for the derivative term so step
                // changes in target_heading (e.g. waypoint flips) do not
                // slam the D term. PID expects velocity = d(measurement)/dt;
                // measurement here is offset = imu::yaw - target_heading,
                // so d(offset)/dt = d(imu::yaw)/dt = gyr_z (both DMP yaw
                // and gyrZ are CCW-positive about the chip +Z, so signs
                // match without inversion).
                float yaw_rate = imu::gyr_z.size() > 0 ? imu::gyr_z[0] : 0.0f;
                float corr = heading_controller.compute(offset, 0, yaw_rate);
                turn_left = -corr;
                turn_right = +corr;
            }

            // Integrate the post-rolloff forward against live yaw so the
            // host's predict step gets a faithful PWM-seconds displacement
            float yaw_rad = imu::yaw * (float)(M_PI / 180.0);
            dx_pwm_s += forward * cosf(yaw_rad) * dt;
            dy_pwm_s += forward * sinf(yaw_rad) * dt;
            abs_pwm_s += fabsf(forward) * dt;
        }

    } // namespace methods

    // Commands

    namespace commands {

        void set_align_tol_cmd(BLERequest &req) {
            float v;
            BLE_CHECK_READ(req, req.read(v), "align_tol_deg");
            // Guard against zero/negative which would NaN the exp().
            if (v < 0.1f)
                v = 0.1f;
            align_tol_deg = v;
            INFO_PRINT(F("drive align_tol_deg = "));
            INFO_PRINTLN(align_tol_deg);
            req.new_response().end();
        }

        void update_cmd(BLERequest &req) {
            float speed, heading;
            uint8_t long_mode_1, long_mode_2;
            BLE_CHECK_READ(req, req.read(speed), "speed");
            BLE_CHECK_READ(req, req.read(heading), "heading");
            BLE_CHECK_READ(req, req.read(long_mode_1), "long_mode_1");
            BLE_CHECK_READ(req, req.read(long_mode_2), "long_mode_2");

            target_speed = speed;
            target_heading = heading;
            last_update_ms = millis();
            active = true;

            tof::methods::set_long_mode_1(long_mode_1 != 0);
            tof::methods::set_long_mode_2(long_mode_2 != 0);

            // Pull latest raw ToF samples (most recent push lives at index 0).
            int32_t tof1_t = -1, tof2_t = -1;
            int16_t tof1_d = -1, tof2_d = -1;
            float tof1_y = imu::yaw;
            float tof2_y = methods::wrap_angle(tof1_y - 90.0f);

            if (tof::dist1.size() > 1) {
                tof1_t = (int32_t)tof::times1[0];
                tof1_d = (int16_t)tof::dist1[0];
                // Wraparound-safe average of the two most recent yaws:
                // a plain (a + b) / 2 puts the midpoint of +179 and -179
                // at 0 instead of +/-180, which sends the host ToF ray
                // into the wrong half-plane and bugs the Bayes update.
                float dy = methods::wrap_angle(tof::yaw1[1] - tof::yaw1[0]);
                tof1_y = methods::wrap_angle(tof::yaw1[0] + dy * 0.5f);
            }
            if (tof::dist2.size() > 1) {
                tof2_t = (int32_t)tof::times2[0];
                tof2_d = (int16_t)tof::dist2[0];

                float dy = methods::wrap_angle(tof::yaw2[1] - tof::yaw2[0]);
                tof2_y = methods::wrap_angle(tof::yaw2[0] + dy * 0.5f  - 90.0f);
            }
            // Sensor 2 is mounted 90 deg clockwise of sensor 1, so its
            // bearing in the world frame is tof1_y - 90.

            BLEResponse res = req.new_response();
            res.add((int32_t)timer::methods::time_us());
            res.add(tof1_t);
            res.add(tof1_d);
            res.add(tof1_y);
            res.add(tof2_t);
            res.add(tof2_d);
            res.add(tof2_y);
            res.add(imu::yaw);
            res.add(dx_pwm_s);
            res.add(dy_pwm_s);
            res.add(abs_pwm_s);
            res.end();

            // Reset integrators after shipping so the host gets the
            // displacement accumulated since the previous DriveUpdate.
            dx_pwm_s = 0.0f;
            dy_pwm_s = 0.0f;
            abs_pwm_s = 0.0f;
        }

    } // namespace commands

    // Init

    void init() {
        motors::methods::register_source(&forward);
        motors::methods::register_source(&turn_left, &turn_right);

        // Higher kP than the standalone angle PID: the turn source is
        // normalized independently, so a large corr just saturates the
        // turn-in-place differential at full diff. Saturating sooner means
        // small heading errors drive a meaningful wheel difference instead
        // of dying inside the desticker dead band.
        heading_controller.kP = 6.0f;
        heading_controller.kI = 0.0f;
        heading_controller.kD = 0.325f;
        heading_controller.integrator_cap = 0;
        heading_controller.integrator_range = 0;

        ble::methods::register_command(DRIVE_UPDATE, commands::update_cmd);
        ble::methods::register_command(
            DRIVE_SET_ALIGN_TOL, commands::set_align_tol_cmd
        );
    }

    // Periodic

    void periodic() {
        methods::update();
    }

} // namespace drive
