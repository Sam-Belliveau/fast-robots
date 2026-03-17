#pragma once

// Angle PID subsystem
// Owns: orientation PD controller, debug buffers.
// Depends on: imu (yaw, yaw_valid), motors (set, stop).

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

    bool active = false;
    unsigned long start_time = 0;
    unsigned long duration_ms = 3000;
    float setpoint = 90;  // relative target angle in degrees
    float yaw_offset = 0; // yaw at start, used to make setpoint relative

    int out_deadband = 80;

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

        // Convert PD output to PWM with deadband compensation
        static int to_pwm(float output) {
            float abs_v = fabs(output);
            float sign = output > 0 ? 1.0 : -1.0;
            if (abs_v < 1.0)
                return 0;
            int pwm = (int)(sign *
                            (out_deadband + abs_v *
                                                (PWM_MAX - out_deadband) /
                                                PWM_MAX));
            return constrain(pwm, -PWM_MAX, PWM_MAX);
        }

        // Wrap angle difference to [-180, 180]
        static float wrap_angle(float angle) {
            while (angle > 180.0f)
                angle -= 360.0f;
            while (angle < -180.0f)
                angle += 360.0f;
            return angle;
        }

        void update() {
            if (!active)
                return;

            if (millis() - start_time >= duration_ms) {
                active = false;
                motors::methods::stop();
                SERIAL_PRINTLN(F("Angle PID complete"));
                return;
            }

            if (!imu::yaw_valid)
                return;

            // Relative angle from start
            float rel_angle = wrap_angle(imu::yaw - yaw_offset);
            float error = wrap_angle(setpoint - rel_angle);

            // Use PD controller (kI should be 0)
            float output = controller.compute(rel_angle, rel_angle + error);
            int pwm = to_pwm(output);

            // Differential drive: left and right spin in opposite directions
            motors::methods::set(-pwm, pwm);

            times.push((int)(timer::methods::time_us()));
            angle_buf.push((int)(rel_angle * 10)); // store 0.1 deg resolution
            error_buf.push((int)(error * 10));
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

            // Stop motors and reset controller state
            motors::methods::stop();
            controller.reset();

            // Flush stale DMP data and get a fresh yaw
            imu::myICM.resetFIFO();
            delay(20);
            imu::methods::read_dmp();
            yaw_offset = imu::yaw;

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

            SERIAL_PRINT(F("Angle PID start: sp="));
            SERIAL_PRINT(setpoint);
            SERIAL_PRINT(F(" offset="));
            SERIAL_PRINT(yaw_offset);
            SERIAL_PRINT(F(" kP="));
            SERIAL_PRINT(controller.kP);
            SERIAL_PRINT(F(" kD="));
            SERIAL_PRINT(controller.kD);
            SERIAL_PRINT(F(" dur="));
            SERIAL_PRINTLN(duration_ms);
            req.new_response().end();
        }

        void stop(BLERequest &req) {
            active = false;
            motors::methods::stop();
            SERIAL_PRINTLN(F("Angle PID stopped"));
            req.new_response().end();
        }

        void set_setpoint(BLERequest &req) {
            float sp;
            BLE_CHECK_READ(req, req.read(sp), "setpoint");
            setpoint = sp;
            SERIAL_PRINT(F("Angle PID setpoint: "));
            SERIAL_PRINTLN(sp);
            req.new_response().end();
        }

        void set_gains(BLERequest &req) {
            float kp, kd;
            BLE_CHECK_READ(req, req.read(kp), "kp");
            BLE_CHECK_READ(req, req.read(kd), "kd");
            controller.kP = kp;
            controller.kD = kd;
            SERIAL_PRINT(F("Angle PID gains: kP="));
            SERIAL_PRINT(kp);
            SERIAL_PRINT(F(" kD="));
            SERIAL_PRINTLN(kd);
            req.new_response().end();
        }

        void set_params(BLERequest &req) {
            float rc;
            int32_t db;
            BLE_CHECK_READ(req, req.read(rc), "rc");
            BLE_CHECK_READ(req, req.read(db), "deadband");
            controller.d_filter.set_rc(rc);
            out_deadband = db;
            SERIAL_PRINT(F("Angle PID params: rc="));
            SERIAL_PRINT(rc);
            SERIAL_PRINT(F(" deadband="));
            SERIAL_PRINTLN(db);
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

            SERIAL_PRINT(F("SEND_ANGLE_PID_DATA: "));
            SERIAL_PRINT(times.size());
            SERIAL_PRINTLN(F(" samples"));
        }

    } // namespace commands

    // Init

    void init() {
        // PD controller: no integral term
        controller.kI = 0;
        controller.integrator_cap = 0;
        controller.integrator_range = 0;

        ble::methods::register_command(
            ANGLE_PID_START, commands::start
        );
        ble::methods::register_command(
            ANGLE_PID_STOP, commands::stop
        );
        ble::methods::register_command(
            ANGLE_PID_SETPOINT, commands::set_setpoint
        );
        ble::methods::register_command(
            ANGLE_PID_GAINS, commands::set_gains
        );
        ble::methods::register_command(
            ANGLE_PID_PARAMS, commands::set_params
        );
        ble::methods::register_command(
            SEND_ANGLE_PID_DATA, commands::send_data
        );
    }

    // Periodic

    void periodic() {
        methods::update();
    }

} // namespace angle_pid
