#pragma once

// Mapping subsystem
// Orchestrates angle PID to step through orientations while recording
// ToF readings at each step. Used for Lab 9 room mapping.
//
// Flow: MAP_START -> robot steps through N evenly-spaced angles ->
//       at each angle, waits for PID to settle, takes ToF reading ->
//       after full rotation, stops -> SEND_MAP_DATA returns all readings.

#include "subsystem_serial.h"
#include "subsystem_ble.h"
#include "subsystem_imu.h"
#include "subsystem_tof.h"
#include "subsystem_angle_pid.h"
#include "lib_CircularBuffer.h"
#include "lib_Zip.h"

namespace mapping {

    // Configuration
    int num_steps = 18; // angular steps per rotation (18 = 20 deg each)
    float settle_threshold = 5.0f;         // degrees error to consider settled
    unsigned long settle_time_ms = 500;    // ms to hold settled before reading
    unsigned long step_timeout_ms = 5000;  // max time per step

    // State
    bool active = false;
    int current_step = 0;
    float start_yaw = 0;
    unsigned long step_start_time = 0;
    bool settled = false;
    unsigned long settle_start_time = 0;

    // Data buffers
    CircularBuffer<int, 0x100> times;
    CircularBuffer<int, 0x100> angles;   // yaw * 10 (0.1 deg resolution)
    CircularBuffer<int, 0x100> tof1_buf; // front sensor (mm)
    CircularBuffer<int, 0x100> tof2_buf; // side sensor (mm)

    namespace methods {

        void record_reading() {
            times.push((int)(timer::methods::time_us()));
            angles.push((int)(imu::yaw * 10));
            tof1_buf.push(tof::methods::current1());
            tof2_buf.push(tof::methods::current2());

            INFO_PRINT(F("Map step "));
            INFO_PRINT(current_step);
            INFO_PRINT(F(": yaw="));
            INFO_PRINT(imu::yaw);
            INFO_PRINT(F(" d1="));
            INFO_PRINT(tof::methods::current1());
            INFO_PRINT(F(" d2="));
            INFO_PRINTLN(tof::methods::current2());
        }

        void advance_step() {
            float target =
                start_yaw + current_step * (360.0f / num_steps);
            angle_pid::setpoint = target;
            angle_pid::controller.reset();
            step_start_time = millis();
            settled = false;
        }

        void finish() {
            active = false;
            angle_pid::active = false;
            angle_pid::output_left = 0;
            angle_pid::output_right = 0;
            INFO_PRINT(F("Mapping complete: "));
            INFO_PRINT(times.size());
            INFO_PRINTLN(F(" readings"));
        }

        void update() {
            if (!active)
                return;
            if (!imu::yaw_valid)
                return;

            float error = fabsf(
                angle_pid::methods::wrap_angle(
                    imu::yaw - angle_pid::setpoint
                )
            );

            // Check if settled at target angle
            if (error < settle_threshold) {
                if (!settled) {
                    settled = true;
                    settle_start_time = millis();
                } else if (millis() - settle_start_time >=
                           settle_time_ms) {
                    // Settled long enough, take reading and advance
                    record_reading();
                    current_step++;
                    if (current_step >= num_steps) {
                        finish();
                        return;
                    }
                    advance_step();
                }
            } else {
                settled = false;
            }

            // Step timeout: take reading anyway and move on
            if (millis() - step_start_time >= step_timeout_ms) {
                INFO_PRINTLN(F("Map step timeout"));
                record_reading();
                current_step++;
                if (current_step >= num_steps) {
                    finish();
                    return;
                }
                advance_step();
            }
        }

    } // namespace methods

    namespace commands {

        void start(BLERequest &req) {
            int32_t steps;
            BLE_CHECK_READ(req, req.read(steps), "steps");
            num_steps = steps;

            // Clear data buffers
            times.clear();
            angles.clear();
            tof1_buf.clear();
            tof2_buf.clear();

            // Reset DMP FIFO and read fresh yaw
            imu::myICM.resetFIFO();
            delay(20);
            imu::methods::read_dmp();

            // Initialize state
            current_step = 0;
            start_yaw = imu::yaw;
            settled = false;
            active = true;

            // Activate angle PID with long duration
            angle_pid::controller.reset();
            angle_pid::times.clear();
            angle_pid::angle_buf.clear();
            angle_pid::error_buf.clear();
            angle_pid::motor_out.clear();
            angle_pid::motor_left.clear();
            angle_pid::motor_right.clear();
            angle_pid::p_buf.clear();
            angle_pid::d_buf.clear();
            angle_pid::duration_ms = 600000;
            angle_pid::start_time = millis();
            angle_pid::active = true;

            methods::advance_step();

            INFO_PRINT(F("Mapping start: steps="));
            INFO_PRINT(num_steps);
            INFO_PRINT(F(" start_yaw="));
            INFO_PRINTLN(start_yaw);
            req.new_response().end();
        }

        void stop_cmd(BLERequest &req) {
            methods::finish();
            req.new_response().end();
        }

        void set_params(BLERequest &req) {
            float threshold;
            int32_t settle, timeout;
            BLE_CHECK_READ(req, req.read(threshold), "threshold");
            BLE_CHECK_READ(req, req.read(settle), "settle_ms");
            BLE_CHECK_READ(req, req.read(timeout), "timeout_ms");
            settle_threshold = threshold;
            settle_time_ms = (unsigned long)settle;
            step_timeout_ms = (unsigned long)timeout;

            INFO_PRINT(F("Map params: thresh="));
            INFO_PRINT(threshold);
            INFO_PRINT(F(" settle="));
            INFO_PRINT(settle);
            INFO_PRINT(F(" timeout="));
            INFO_PRINTLN(timeout);
            req.new_response().end();
        }

        void send_data(BLERequest &req) {
            BLEResponse res = req.new_response();
            zip(
                [&](int t, int angle, int d1, int d2) {
                    res.add((int32_t)t);
                    res.add((int32_t)angle);
                    res.add((int32_t)d1);
                    res.add((int32_t)d2);
                },
                times.begin(),
                times.end(),
                angles.begin(),
                tof1_buf.begin(),
                tof2_buf.begin()
            );
            res.end();

            INFO_PRINT(F("SEND_MAP_DATA: "));
            INFO_PRINT(times.size());
            INFO_PRINTLN(F(" readings"));
        }

    } // namespace commands

    void init() {
        ble::methods::register_command(MAP_START, commands::start);
        ble::methods::register_command(MAP_STOP, commands::stop_cmd);
        ble::methods::register_command(MAP_PARAMS, commands::set_params);
        ble::methods::register_command(
            SEND_MAP_DATA, commands::send_data
        );
    }

    void periodic() {
        methods::update();
    }

} // namespace mapping
