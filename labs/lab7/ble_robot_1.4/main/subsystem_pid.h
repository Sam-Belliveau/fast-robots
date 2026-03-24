#pragma once

// PID subsystem
// Owns: PID controller, PID state, debug buffers.
// Depends on: tof (current1), motors (set, stop).

#include "subsystem_serial.h"
#include "subsystem_ble.h"
#include "subsystem_tof.h"
#include "subsystem_motors.h"
#include "lib_PID.h"
#include "lib_CircularBuffer.h"
#include "lib_Zip.h"

namespace pid {

    // Variables

    PID controller;

    bool active = false;
    unsigned long start_time = 0;
    unsigned long duration_ms = 3000;
    float setpoint = 304; // default distance in mm

    const int in_deadband = 16;
    int out_deadband = 120;

    CircularBuffer<int, 0x100> times;
    CircularBuffer<int, 0x100> measurement;
    CircularBuffer<int, 0x100> error_buf;
    CircularBuffer<int, 0x100> motor_out;
    CircularBuffer<int, 0x100> motor_left;
    CircularBuffer<int, 0x100> motor_right;
    CircularBuffer<int, 0x100> p_buf;
    CircularBuffer<int, 0x100> i_buf;
    CircularBuffer<int, 0x100> d_buf;

    // Methods

    namespace methods {

        static int to_pwm(float output) {
            float abs_v = fabs(output);
            float sign = output > 0 ? 1.0 : -1.0;
            if (abs_v <= in_deadband) {
                return (int)(sign * abs_v *
                             ((float)out_deadband / in_deadband));
            }
            int pwm =
                (int)(sign * (out_deadband + (abs_v - in_deadband) *
                                                 (PWM_MAX - out_deadband) /
                                                 (PWM_MAX - in_deadband)));
            return constrain(pwm, -PWM_MAX, PWM_MAX);
        }

        void update() {
            if (!active)
                return;

            if (millis() - start_time >= duration_ms) {
                active = false;
                motors::methods::stop();
                INFO_PRINTLN(F("PID complete"));
                return;
            }

            int distance = tof::methods::current1();
            if (distance < 0)
                return;

            float output = -controller.compute((float)distance, setpoint);
            int pwm = to_pwm(output);
            motors::methods::set(pwm, pwm);

            times.push((int)(timer::methods::time_us()));
            measurement.push(distance);
            error_buf.push((int)(setpoint - distance));
            motor_out.push(pwm);
            motor_left.push((int)(motors::current_left));
            motor_right.push((int)(motors::current_right));
            p_buf.push((int)(controller.p_out));
            i_buf.push((int)(controller.i_out));
            d_buf.push((int)(controller.d_out));
        }

    } // namespace methods

    // Commands

    namespace commands {

        void start(BLERequest &req) {
            int32_t duration;
            BLE_CHECK_READ(req, req.read(duration), "duration");
            duration_ms = (unsigned long)duration;

            controller.reset();
            times.clear();
            measurement.clear();
            error_buf.clear();
            motor_out.clear();
            motor_left.clear();
            motor_right.clear();
            p_buf.clear();
            i_buf.clear();
            d_buf.clear();
            active = true;
            start_time = millis();

            INFO_PRINT(F("PID start: sp="));
            INFO_PRINT(setpoint);
            INFO_PRINT(F(" kP="));
            INFO_PRINT(controller.kP);
            INFO_PRINT(F(" kI="));
            INFO_PRINT(controller.kI);
            INFO_PRINT(F(" kD="));
            INFO_PRINT(controller.kD);
            INFO_PRINT(F(" dur="));
            INFO_PRINTLN(duration_ms);
            req.new_response().end();
        }

        void stop(BLERequest &req) {
            active = false;
            motors::methods::stop();
            INFO_PRINTLN(F("PID stopped"));
            req.new_response().end();
        }

        void set_setpoint(BLERequest &req) {
            float sp;
            BLE_CHECK_READ(req, req.read(sp), "setpoint");
            setpoint = sp;
            INFO_PRINT(F("PID setpoint: "));
            INFO_PRINTLN(sp);
            req.new_response().end();
        }

        void set_gains(BLERequest &req) {
            float kp, ki, kd;
            BLE_CHECK_READ(req, req.read(kp), "kp");
            BLE_CHECK_READ(req, req.read(ki), "ki");
            BLE_CHECK_READ(req, req.read(kd), "kd");
            controller.kP = kp;
            controller.kI = ki;
            controller.kD = kd;
            INFO_PRINT(F("PID gains: "));
            INFO_PRINT(kp);
            INFO_PRINT(F(" "));
            INFO_PRINT(ki);
            INFO_PRINT(F(" "));
            INFO_PRINTLN(kd);
            req.new_response().end();
        }

        void set_params(BLERequest &req) {
            float cap, range, rc;
            int32_t db;
            BLE_CHECK_READ(req, req.read(cap), "cap");
            BLE_CHECK_READ(req, req.read(range), "range");
            BLE_CHECK_READ(req, req.read(rc), "rc");
            BLE_CHECK_READ(req, req.read(db), "deadband");
            controller.integrator_cap = cap;
            controller.integrator_range = range;
            controller.d_filter.set_rc(rc);
            out_deadband = db;
            INFO_PRINT(F("PID params: cap="));
            INFO_PRINT(cap);
            INFO_PRINT(F(" range="));
            INFO_PRINT(range);
            INFO_PRINT(F(" rc="));
            INFO_PRINT(rc);
            INFO_PRINT(F(" deadband="));
            INFO_PRINTLN(db);
            req.new_response().end();
        }

        void send_data(BLERequest &req) {
            BLEResponse res = req.new_response();
            zip(
                [&](int t,
                    int meas,
                    int err,
                    int pwm,
                    int ml,
                    int mr,
                    int p,
                    int i,
                    int d) {
                    res.add((int32_t)t);
                    res.add((int32_t)meas);
                    res.add((int32_t)err);
                    res.add((int32_t)pwm);
                    res.add((int32_t)ml);
                    res.add((int32_t)mr);
                    res.add((int32_t)p);
                    res.add((int32_t)i);
                    res.add((int32_t)d);
                },
                times.begin(),
                times.end(),
                measurement.begin(),
                error_buf.begin(),
                motor_out.begin(),
                motor_left.begin(),
                motor_right.begin(),
                p_buf.begin(),
                i_buf.begin(),
                d_buf.begin()
            );

            res.end();

            INFO_PRINT(F("SEND_PID_DATA: "));
            INFO_PRINT(times.size());
            INFO_PRINTLN(F(" samples"));
        }

    } // namespace commands

    // Init

    void init() {
        ble::methods::register_command(PID_START, commands::start);
        ble::methods::register_command(PID_STOP, commands::stop);
        ble::methods::register_command(PID_SETPOINT, commands::set_setpoint);
        ble::methods::register_command(PID_GAINS, commands::set_gains);
        ble::methods::register_command(PID_PARAMS, commands::set_params);
        ble::methods::register_command(SEND_PID_DATA, commands::send_data);
    }

    // Periodic

    void periodic() {
        methods::update();
    }

} // namespace pid
