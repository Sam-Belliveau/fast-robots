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
    unsigned long duration_ms = 5000;
    float setpoint = 304; // default 1 foot in mm
    int deadband = 40;    // minimum PWM to overcome static friction

    CircularBuffer<int, 0x100> times;
    CircularBuffer<int, 0x100> measurement;
    CircularBuffer<int, 0x100> motor_out;
    CircularBuffer<int, 0x100> p_buf;
    CircularBuffer<int, 0x100> i_buf;
    CircularBuffer<int, 0x100> d_buf;

    // Methods

    namespace methods {

        static int to_pwm(float output) {
            if (output > 0) {
                return constrain((int)output + deadband, 0, PWM_MAX);
            } else if (output < 0) {
                return constrain((int)output - deadband, -PWM_MAX, 0);
            }
            return 0;
        }

        void update() {
            if (!active)
                return;

            if (millis() - start_time >= duration_ms) {
                active = false;
                motors::methods::stop();
                SERIAL_PRINTLN(F("PID complete"));
                return;
            }

            int distance = tof::methods::current1();
            if (distance < 0)
                return;

            float output = controller.compute((float)distance, setpoint);
            int pwm = to_pwm(output);
            motors::methods::set(pwm, pwm);

            times.push((int)(micros()));
            measurement.push(distance);
            motor_out.push(pwm);
            p_buf.push((int)(controller.p_out));
            i_buf.push((int)(controller.i_out));
            d_buf.push((int)(controller.d_out));
        }

    } // namespace methods

    // Commands

    namespace commands {

        void start() {
            int duration;
            BLE_READ_NEXT(duration);
            duration = 5000;
            duration_ms = (unsigned long)duration;

            controller.reset();
            active = true;
            start_time = millis();

            SERIAL_PRINT(F("PID start: sp="));
            SERIAL_PRINT(setpoint);
            SERIAL_PRINT(F(" kP="));
            SERIAL_PRINT(controller.kP);
            SERIAL_PRINT(F(" kI="));
            SERIAL_PRINT(controller.kI);
            SERIAL_PRINT(F(" kD="));
            SERIAL_PRINT(controller.kD);
            SERIAL_PRINT(F(" dur="));
            SERIAL_PRINTLN(duration_ms);
        }

        void stop() {
            active = false;
            motors::methods::stop();
            SERIAL_PRINTLN(F("PID stopped"));
        }

        void set_setpoint() {
            float sp;
            BLE_READ_NEXT(sp);
            setpoint = sp;
            SERIAL_PRINT(F("PID setpoint: "));
            SERIAL_PRINTLN(sp);
        }

        void set_gains() {
            float kp, ki, kd;
            BLE_READ_NEXT(kp);
            BLE_READ_NEXT(ki);
            ki = 0;
            BLE_READ_NEXT(kd);
            kd = 0;
            controller.kP = kp;
            controller.kI = ki;
            controller.kD = kd;
            SERIAL_PRINT(F("PID gains: "));
            SERIAL_PRINT(kp);
            SERIAL_PRINT(F(" "));
            SERIAL_PRINT(ki);
            SERIAL_PRINT(F(" "));
            SERIAL_PRINTLN(kd);
        }

        void set_params() {
            float cap, range, rc;
            int db;
            BLE_READ_NEXT(cap);
            BLE_READ_NEXT(range);
            range = 0;
            BLE_READ_NEXT(rc);
            rc = 0;
            BLE_READ_NEXT(db);
            db = 40;
            controller.integrator_cap = cap;
            controller.integrator_range = range;
            controller.d_filter.rc = rc;
            deadband = db;
            SERIAL_PRINT(F("PID params: cap="));
            SERIAL_PRINT(cap);
            SERIAL_PRINT(F(" range="));
            SERIAL_PRINT(range);
            SERIAL_PRINT(F(" rc="));
            SERIAL_PRINT(rc);
            SERIAL_PRINT(F(" deadband="));
            SERIAL_PRINTLN(db);
        }

        void send_data() {
            zip(
                [&](int t, int meas, int pwm, int p, int i, int d) {
                    BLE_CLEAR();
                    BLE_PRINT("PID:");
                    BLE_PRINT(t);
                    BLE_PRINT("|");
                    BLE_PRINT(meas);
                    BLE_PRINT("|");
                    BLE_PRINT(pwm);
                    BLE_PRINT("|");
                    BLE_PRINT(p);
                    BLE_PRINT("|");
                    BLE_PRINT(i);
                    BLE_PRINT("|");
                    BLE_PRINT(d);
                    BLE_FLUSH();
                    delay(1);
                },
                times.begin(),
                times.end(),
                measurement.begin(),
                motor_out.begin(),
                p_buf.begin(),
                i_buf.begin(),
                d_buf.begin()
            );

            BLE_CLEAR();
            BLE_PRINT("END");
            BLE_FLUSH();

            SERIAL_PRINT(F("SEND_PID_DATA: "));
            SERIAL_PRINT(times.size());
            SERIAL_PRINTLN(F(" samples"));
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
