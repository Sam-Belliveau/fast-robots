#pragma once

// Step response subsystem
// Drives motors at constant PWM while logging ToF distance.
// Used to estimate drag and momentum for the Kalman filter.
// Depends on: tof (current1), motors (set, stop).

#include "subsystem_serial.h"
#include "subsystem_ble.h"
#include "subsystem_imu.h"
#include "subsystem_tof.h"
#include "subsystem_motors.h"
#include "lib_PID.h"
#include "lib_CircularBuffer.h"
#include "lib_Zip.h"

namespace step {

    // Variables

    bool active = false;
    unsigned long start_time = 0;
    unsigned long duration_ms = 3000;
    int pwm = 0;
    float min_distance = 100; // emergency stop distance (mm)
    float min_ttc_ms = 200;   // emergency stop time-to-crash (ms)
    int last_tof_time = -1;   // timestamp of last logged ToF reading

    // Yaw PID for driving straight
    PID yaw_controller;
    float yaw_offset = 0;

    CircularBuffer<int, 0x100> times;
    CircularBuffer<int, 0x100> distances;
    CircularBuffer<int, 0x100> motor_out;

    // Methods

    namespace methods {

        static float wrap_angle(float angle) {
            return angle - 360.0f * roundf(angle / 360.0f);
        }

        void update() {
            if (!active)
                return;

            if (millis() - start_time >= duration_ms) {
                active = false;
                motors::methods::stop();
                INFO_PRINTLN(F("Step complete"));
                return;
            }

            int dist = tof::methods::current1();

            // Emergency stop: distance or time-to-crash
            float ttc = tof::methods::time_to_crash1();
            if ((dist >= 0 && dist < (int)min_distance) ||
                (ttc >= 0 && ttc < min_ttc_ms)) {
                active = false;
                motors::methods::stop();
                INFO_PRINT(F("Step ESTOP: d="));
                INFO_PRINT(dist);
                INFO_PRINT(F("mm ttc="));
                INFO_PRINT((int)ttc);
                INFO_PRINTLN(F("ms"));
                return;
            }

            // Yaw correction for driving straight
            float rel_angle = wrap_angle(imu::yaw - yaw_offset);
            int corr = constrain(
                (int)yaw_controller.compute(rel_angle, 0), -PWM_MAX, PWM_MAX
            );
            motors::methods::set(pwm - corr, pwm + corr);

            // Only log when a new raw ToF reading arrives
            if (tof::times1.size() == 0)
                return;
            int cur_tof_time = tof::times1[0];
            if (cur_tof_time == last_tof_time)
                return;
            last_tof_time = cur_tof_time;

            times.push(cur_tof_time);
            distances.push(tof::dist1[0]);
            motor_out.push(pwm);
        }

    } // namespace methods

    // Commands

    namespace commands {

        void start(BLERequest &req) {
            int32_t dur, pwm_val;
            BLE_CHECK_READ(req, req.read(dur), "duration");
            BLE_CHECK_READ(req, req.read(pwm_val), "pwm");
            duration_ms = (unsigned long)dur;
            pwm = (int)pwm_val;

            // Reset yaw correction
            yaw_controller.reset();
            imu::myICM.resetFIFO();
            delay(20);
            imu::methods::read_dmp();
            yaw_offset = imu::yaw;

            times.clear();
            distances.clear();
            motor_out.clear();
            last_tof_time = -1;
            active = true;
            start_time = millis();

            INFO_PRINT(F("Step response: pwm="));
            INFO_PRINT(pwm);
            INFO_PRINT(F(" dur="));
            INFO_PRINTLN(duration_ms);
            req.new_response().end();
        }

        void send_data(BLERequest &req) {
            BLEResponse res = req.new_response();
            zip(
                [&](int t, int d, int m) {
                    res.add((int32_t)t);
                    res.add((int32_t)d);
                    res.add((int32_t)m);
                },
                times.begin(),
                times.end(),
                distances.begin(),
                motor_out.begin()
            );
            res.end();

            INFO_PRINT(F("SEND_STEP_DATA: "));
            INFO_PRINT(times.size());
            INFO_PRINTLN(F(" samples"));
        }

    } // namespace commands

    // Init

    void init() {
        // Yaw PD defaults for driving straight
        yaw_controller.kP = 3.0;
        yaw_controller.kI = 0;
        yaw_controller.kD = 0.5;
        yaw_controller.integrator_cap = 0;
        yaw_controller.integrator_range = 0;

        ble::methods::register_command(STEP_RESPONSE, commands::start);
        ble::methods::register_command(SEND_STEP_DATA, commands::send_data);
    }

    // Periodic

    void periodic() {
        methods::update();
    }

} // namespace step
