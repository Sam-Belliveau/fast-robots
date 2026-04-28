#pragma once

// Angle PID subsystem
// Owns: orientation PD controller, debug buffers.
// Writes output_left / output_right for the motors subsystem to pull.

#include "subsystem_serial.h"
#include "subsystem_ble.h"
#include "subsystem_imu.h"
#include "subsystem_motors.h"
#include "lib_PID.h"
#include "lib_CircularBuffer.h"
#include "lib_Zip.h"

namespace angle_pid {

    // Variables

    PID controller;

    float output_left = 0.0f;
    float output_right = 0.0f;

    bool active = false;
    unsigned long start_time = 0;
    unsigned long duration_ms = 3000;
    float setpoint = 0; // absolute target angle in degrees

    CircularBuffer<int, 0x100> times;
    CircularBuffer<int, 0x100> angle_buf;
    CircularBuffer<int, 0x100> error_buf;
    CircularBuffer<int, 0x100> motor_out;
    CircularBuffer<int, 0x100> motor_left;
    CircularBuffer<int, 0x100> motor_right;
    CircularBuffer<int, 0x100> p_buf;
    CircularBuffer<int, 0x100> d_buf;

    // Methods

    namespace methods {

        static int to_pwm(float output) {
            return constrain((int)output, -PWM_MAX, PWM_MAX);
        }

        // Wrap angle difference to [-180, 180]
        static float wrap_angle(float angle) {
            return angle - 360.0f * roundf(angle / 360.0f);
        }

        void update() {
            if (!active)
                return;

            if (millis() - start_time >= duration_ms) {
                active = false;
                output_left = 0.0f;
                output_right = 0.0f;
                INFO_PRINTLN(F("Angle PID complete"));
                return;
            }

            if (!imu::yaw_valid)
                return;

            float offset = wrap_angle(imu::yaw - setpoint);

            // Pass error directly so the derivative doesn't spike on wrap
            float output = controller.compute(offset, 0);
            int pwm = to_pwm(output);

            // Differential drive: left and right spin in opposite directions
            output_left = -pwm;
            output_right = pwm;

            times.push((int)(timer::methods::time_us()));
            angle_buf.push((int)(imu::yaw * 10)); // store 0.1 deg resolution
            error_buf.push((int)(offset * 10));
            motor_out.push(pwm);
            motor_left.push((int)(motors::current_left));
            motor_right.push((int)(motors::current_right));
            p_buf.push((int)(controller.p_out));
            d_buf.push((int)(controller.d_out));
        }

    } // namespace methods

    // Commands

    namespace commands {

        void start(BLERequest &req) {
            int32_t duration;
            BLE_CHECK_READ(req, req.read(duration), "duration");
            duration_ms = (unsigned long)duration;

            // Reset output and controller state
            output_left = 0.0f;
            output_right = 0.0f;
            controller.reset();

            times.clear();
            angle_buf.clear();
            error_buf.clear();
            motor_out.clear();
            motor_left.clear();
            motor_right.clear();
            p_buf.clear();
            d_buf.clear();
            active = true;
            start_time = millis();

            INFO_PRINT(F("Angle PID start: sp="));
            INFO_PRINT(setpoint);
            INFO_PRINT(F(" kP="));
            INFO_PRINT(controller.kP);
            INFO_PRINT(F(" kD="));
            INFO_PRINT(controller.kD);
            INFO_PRINT(F(" dur="));
            INFO_PRINTLN(duration_ms);
            req.new_response().end();
        }

        void stop(BLERequest &req) {
            active = false;
            output_left = 0.0f;
            output_right = 0.0f;
            INFO_PRINTLN(F("Angle PID stopped"));
            req.new_response().end();
        }

        void set_setpoint(BLERequest &req) {
            float sp;
            BLE_CHECK_READ(req, req.read(sp), "setpoint");
            setpoint = sp;
            INFO_PRINT(F("Angle PID setpoint: "));
            INFO_PRINTLN(sp);
            req.new_response().end();
        }

        void set_setpoint_relative(BLERequest &req) {
            float delta;
            BLE_CHECK_READ(req, req.read(delta), "delta");
            setpoint = imu::yaw + delta;
            INFO_PRINT(F("Angle PID setpoint (rel): "));
            INFO_PRINTLN(setpoint);
            req.new_response().end();
        }

        void set_gains(BLERequest &req) {
            float kp, kd;
            BLE_CHECK_READ(req, req.read(kp), "kp");
            BLE_CHECK_READ(req, req.read(kd), "kd");
            controller.kP = kp;
            controller.kD = kd;
            INFO_PRINT(F("Angle PID gains: kP="));
            INFO_PRINT(kp);
            INFO_PRINT(F(" kD="));
            INFO_PRINTLN(kd);
            req.new_response().end();
        }

        void set_params(BLERequest &req) {
            float rc;
            BLE_CHECK_READ(req, req.read(rc), "rc");
            controller.d_filter.set_rc(rc);
            INFO_PRINT(F("Angle PID params: rc="));
            INFO_PRINTLN(rc);
            req.new_response().end();
        }

        void send_data(BLERequest &req) {
            BLEResponse res = req.new_response();
            zip(
                [&](int t,
                    int angle,
                    int err,
                    int pwm,
                    int ml,
                    int mr,
                    int p,
                    int d) {
                    res.add((int32_t)t);
                    res.add((int32_t)angle);
                    res.add((int32_t)err);
                    res.add((int32_t)pwm);
                    res.add((int32_t)ml);
                    res.add((int32_t)mr);
                    res.add((int32_t)p);
                    res.add((int32_t)d);
                },
                times.begin(),
                times.end(),
                angle_buf.begin(),
                error_buf.begin(),
                motor_out.begin(),
                motor_left.begin(),
                motor_right.begin(),
                p_buf.begin(),
                d_buf.begin()
            );

            res.end();

            INFO_PRINT(F("SEND_ANGLE_PID_DATA: "));
            INFO_PRINT(times.size());
            INFO_PRINTLN(F(" samples"));
        }

    } // namespace commands

    // Init

    void init() {
        motors::methods::register_source(&output_left, &output_right);

        // PD controller: no integral term
        controller.kP = 4.0f;
        controller.kD = 0.1f;
        controller.kI = 0;
        controller.integrator_cap = 0;
        controller.integrator_range = 0;

        ble::methods::register_command(ANGLE_PID_START, commands::start);
        ble::methods::register_command(ANGLE_PID_STOP, commands::stop);
        ble::methods::register_command(
            ANGLE_PID_SETPOINT, commands::set_setpoint
        );
        ble::methods::register_command(ANGLE_PID_GAINS, commands::set_gains);
        ble::methods::register_command(
            ANGLE_PID_SETPOINT_REL, commands::set_setpoint_relative
        );
        ble::methods::register_command(ANGLE_PID_PARAMS, commands::set_params);
        ble::methods::register_command(
            SEND_ANGLE_PID_DATA, commands::send_data
        );
    }

    // Periodic

    void periodic() {
        methods::update();
    }

} // namespace angle_pid
