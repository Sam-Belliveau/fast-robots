#pragma once

// Stunts subsystem
// Registers two motor sources: thrust (forward/reverse) and yaw (differential).
//
// Drift: constant forward speed + yaw PID straight ahead.
//        When close to wall, yaw setpoint += 180.
//
// Flip:  constant forward speed + yaw PID straight ahead.
//        When close to wall, slam reverse.
//        Uses imu::orientation to detect flip completion and drive back.

#include "subsystem_serial.h"
#include "subsystem_ble.h"
#include "subsystem_imu.h"
#include "subsystem_tof.h"
#include "subsystem_motors.h"
#include "subsystem_kalman.h"
#include "lib_PID.h"
#include "lib_CircularBuffer.h"
#include "lib_Zip.h"

namespace stunts {

    enum StuntMode { STUNT_NONE, STUNT_FLIP_MODE, STUNT_DRIFT_MODE };

    // Motor sources
    float thrust = 0.0f;  // both sides share one value
    float yaw_left = 0.0f;
    float yaw_right = 0.0f;

    StuntMode mode = STUNT_NONE;
    bool triggered = false; // true after distance trigger fires

    PID yaw_controller;
    float yaw_setpoint = 0.0f;

    // Tunable parameters
    float approach_speed = 200.0f;
    float trigger_distance = 914.0f;
    float return_speed = 200.0f;

    unsigned long start_time = 0;
    unsigned long duration_ms = 5000;

    // Debug buffers
    CircularBuffer<int, 0x100> times;
    CircularBuffer<int, 0x100> dist_buf;
    CircularBuffer<int, 0x100> angle_buf;
    CircularBuffer<int, 0x100> orient_buf;
    CircularBuffer<int, 0x100> motor_left_buf;
    CircularBuffer<int, 0x100> motor_right_buf;

    namespace methods {

        static float wrap_angle(float angle) {
            return angle - 360.0f * roundf(angle / 360.0f);
        }

        static float get_distance() {
            return kalman::methods::distance();
        }

        void stop_stunt() {
            mode = STUNT_NONE;
            triggered = false;
            thrust = 0.0f;
            yaw_left = 0.0f;
            yaw_right = 0.0f;
        }

        void update_yaw() {
            if (!imu::yaw_valid)
                return;
            float offset = wrap_angle(imu::yaw - yaw_setpoint);
            float corr = yaw_controller.compute(offset, 0);
            int pwm = constrain((int)corr, -PWM_MAX, PWM_MAX);
            yaw_left = -pwm;
            yaw_right = pwm;
        }

        void log_state(float distance) {
            times.push((int)(timer::methods::time_us()));
            dist_buf.push((int)distance);
            angle_buf.push((int)(imu::yaw * 10));
            orient_buf.push((int)imu::orientation);
            motor_left_buf.push((int)motors::current_left);
            motor_right_buf.push((int)motors::current_right);
        }

        void update_drift() {
            float distance = get_distance();

            if (!triggered && distance < trigger_distance && distance > 0) {
                yaw_setpoint = imu::yaw + 180.0f;
                triggered = true;
                INFO_PRINT(F("Drift: turning, d="));
                INFO_PRINTLN(distance);
            }

            thrust = approach_speed;
            update_yaw();
            log_state(distance);
        }

        void update_flip() {
            float distance = get_distance();

            if (!triggered) {
                // Drive forward, waiting for trigger
                thrust = approach_speed;
                update_yaw();

                if (distance < trigger_distance && distance > 0) {
                    triggered = true;
                    INFO_PRINT(F("Flip: reversing, d="));
                    INFO_PRINTLN(distance);
                }
            } else {
                // Upside down — drive back (motors are inverted)
                thrust = -return_speed;
                yaw_left = 0.0f;
                yaw_right = 0.0f;
            }

            log_state(distance);
        }

        void update() {
            if (mode == STUNT_NONE)
                return;

            if (millis() - start_time >= duration_ms) {
                stop_stunt();
                INFO_PRINTLN(F("Stunt: timeout"));
                return;
            }

            if (mode == STUNT_DRIFT_MODE)
                update_drift();
            else
                update_flip();
        }

    } // namespace methods

    namespace commands {

        static void start_common(BLERequest &req, StuntMode m) {
            int32_t duration;
            BLE_CHECK_READ(req, req.read(duration), "duration");
            duration_ms = (unsigned long)duration;

            times.clear();
            dist_buf.clear();
            angle_buf.clear();
            orient_buf.clear();
            motor_left_buf.clear();
            motor_right_buf.clear();

            yaw_controller.reset();
            imu::myICM.resetFIFO();
            delay(20);
            imu::methods::read_dmp();
            yaw_setpoint = imu::yaw;

            mode = m;
            triggered = false;
            start_time = millis();
            req.new_response().end();
        }

        void drift(BLERequest &req) {
            start_common(req, STUNT_DRIFT_MODE);
            INFO_PRINT(F("Stunt DRIFT: speed="));
            INFO_PRINT(approach_speed);
            INFO_PRINT(F(" trigger="));
            INFO_PRINTLN(trigger_distance);
        }

        void flip(BLERequest &req) {
            start_common(req, STUNT_FLIP_MODE);
            INFO_PRINT(F("Stunt FLIP: speed="));
            INFO_PRINT(approach_speed);
            INFO_PRINT(F(" trigger="));
            INFO_PRINTLN(trigger_distance);
        }

        void stop(BLERequest &req) {
            methods::stop_stunt();
            INFO_PRINTLN(F("Stunt stopped"));
            req.new_response().end();
        }

        void set_params(BLERequest &req) {
            float speed, trigger, ret_speed;
            BLE_CHECK_READ(req, req.read(speed), "speed");
            BLE_CHECK_READ(req, req.read(trigger), "trigger");
            BLE_CHECK_READ(req, req.read(ret_speed), "ret_speed");
            approach_speed = speed;
            trigger_distance = trigger;
            return_speed = ret_speed;

            INFO_PRINT(F("Stunt params: spd="));
            INFO_PRINT(speed);
            INFO_PRINT(F(" trig="));
            INFO_PRINT(trigger);
            INFO_PRINT(F(" ret="));
            INFO_PRINTLN(ret_speed);
            req.new_response().end();
        }

        void set_yaw_gains(BLERequest &req) {
            float kp, kd;
            BLE_CHECK_READ(req, req.read(kp), "kp");
            BLE_CHECK_READ(req, req.read(kd), "kd");
            yaw_controller.kP = kp;
            yaw_controller.kD = kd;
            INFO_PRINT(F("Stunt yaw: kP="));
            INFO_PRINT(kp);
            INFO_PRINT(F(" kD="));
            INFO_PRINTLN(kd);
            req.new_response().end();
        }

        void send_data(BLERequest &req) {
            BLEResponse res = req.new_response();
            zip(
                [&](int t, int dist, int angle, int ori, int ml, int mr) {
                    res.add((int32_t)t);
                    res.add((int32_t)dist);
                    res.add((int32_t)angle);
                    res.add((int32_t)ori);
                    res.add((int32_t)ml);
                    res.add((int32_t)mr);
                },
                times.begin(),
                times.end(),
                dist_buf.begin(),
                angle_buf.begin(),
                orient_buf.begin(),
                motor_left_buf.begin(),
                motor_right_buf.begin()
            );
            res.end();

            INFO_PRINT(F("SEND_STUNT_DATA: "));
            INFO_PRINT(times.size());
            INFO_PRINTLN(F(" samples"));
        }

    } // namespace commands

    void init() {
        motors::methods::register_source(&thrust, &thrust);
        motors::methods::register_source(&yaw_left, &yaw_right);

        yaw_controller.kP = 4.0f;
        yaw_controller.kD = 0.1f;
        yaw_controller.kI = 0;
        yaw_controller.integrator_cap = 0;
        yaw_controller.integrator_range = 0;

        ble::methods::register_command(STUNT_DRIFT, commands::drift);
        ble::methods::register_command(STUNT_FLIP, commands::flip);
        ble::methods::register_command(STUNT_STOP, commands::stop);
        ble::methods::register_command(STUNT_PARAMS, commands::set_params);
        ble::methods::register_command(
            STUNT_YAW_GAINS, commands::set_yaw_gains
        );
        ble::methods::register_command(SEND_STUNT_DATA, commands::send_data);
    }

    void periodic() {
        methods::update();
    }

} // namespace stunts
