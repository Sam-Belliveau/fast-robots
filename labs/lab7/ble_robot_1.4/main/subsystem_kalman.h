#pragma once

// Kalman filter subsystem
// 2-state KF [distance, velocity] for ToF sensor 1.
// Uses motor PWM as control input with drag model from step response.
// Follows Lab 7 task spec: x_dot = A*x + B*u, y = C*x

#include "subsystem_timer.h"
#include "subsystem_serial.h"
#include "subsystem_ble.h"
#include "subsystem_tof.h"
#include "lib_KalmanFilter.h"
#include "lib_CircularBuffer.h"
#include "lib_Zip.h"

namespace kalman {

    // Filter instance: state = [distance (mm), velocity (mm/s)]
    KalmanFilter<2> kf;

    // Dynamics parameters (from step response)
    float drag = 0.8095f;     // d: drag coefficient (1/s)
    float momentum = 11.8318f; // m: momentum term (mm/s² per PWM unit)

    // Discretized matrices (recomputed when params change)
    BLA::Matrix<2, 2> Ad;
    BLA::Matrix<2, 1> Bd;

    // Measurement matrix: state is distance, ToF measures distance
    BLA::Matrix<1, 2> C = {1, 0};

    // Noise covariances
    BLA::Matrix<2, 2> Sigma_u; // process noise
    BLA::Matrix<1, 1> Sigma_z; // measurement noise

    float sigma_pos = 1.0f;   // process noise position (mm)
    float sigma_vel = 100.0f; // process noise velocity (mm/s)
    float sigma_tof = 50.0f;  // measurement noise (mm)

    // Motor input (set by PID or externally)
    float motor_input = 0.0f; // normalized PWM input u
    int last_tof_time = -1;   // timestamp of last ToF update

    // Output log buffers
    CircularBuffer<int, 0x100> log_times;
    CircularBuffer<int, 0x100> log_dist;
    CircularBuffer<int, 0x100> log_vel;

    namespace methods {

        // Recompute discretized matrices from drag/momentum/dt
        void update_matrices() {
            // Continuous: A = [[0, 1], [0, -d]]
            //             B = [[0], [m]]
            // Discretize: Ad = I + dt*A, Bd = dt*B
            float dt = timer::PERIOD_S;
            Ad(0, 0) = 1;
            Ad(0, 1) = dt;
            Ad(1, 0) = 0;
            Ad(1, 1) = 1 - drag * dt;

            Bd(0, 0) = 0;
            Bd(1, 0) = -momentum * dt;  // positive PWM decreases distance

            Sigma_u(0, 0) = sigma_pos * sigma_pos;
            Sigma_u(0, 1) = 0;
            Sigma_u(1, 0) = 0;
            Sigma_u(1, 1) = sigma_vel * sigma_vel;

            Sigma_z(0, 0) = sigma_tof * sigma_tof;
        }

        // Predict step using motor input
        void predict(float u) {
            BLA::Matrix<1, 1> u_mat;
            u_mat(0, 0) = u;
            kf.predict<1>(Ad, Bd, u_mat, Sigma_u);
        }

        // Update step with ToF measurement
        void update(float tof_mm) {
            BLA::Matrix<1, 1> z;
            z(0, 0) = tof_mm;
            kf.update<1>(C, Sigma_z, z);
        }

        float distance() {
            return kf.x(0, 0);
        }
        float velocity() {
            return kf.x(1, 0);
        }

    } // namespace methods

    namespace commands {

        void send_data(BLERequest &req) {
            BLEResponse res = req.new_response();
            zip(
                [&](int t, int d, int v) {
                    res.add((int32_t)t);
                    res.add((int32_t)d);
                    res.add((int32_t)v);
                },
                log_times.begin(),
                log_times.end(),
                log_dist.begin(),
                log_vel.begin()
            );
            res.end();

            INFO_PRINT(F("SEND_KF_DATA: "));
            INFO_PRINT(log_times.size());
            INFO_PRINTLN(F(" samples"));
        }

        void set_params(BLERequest &req) {
            float d, m, s1, s2, s3;
            BLE_CHECK_READ(req, req.read(d), "drag");
            BLE_CHECK_READ(req, req.read(m), "momentum");
            BLE_CHECK_READ(req, req.read(s1), "sigma_pos");
            BLE_CHECK_READ(req, req.read(s2), "sigma_vel");
            BLE_CHECK_READ(req, req.read(s3), "sigma_tof");
            drag = d;
            momentum = m;
            sigma_pos = s1;
            sigma_vel = s2;
            sigma_tof = s3;
            methods::update_matrices();

            INFO_PRINT(F("KF params: d="));
            INFO_PRINT(drag);
            INFO_PRINT(F(" m="));
            INFO_PRINT(momentum);
            INFO_PRINT(F(" sigma_pos="));
            INFO_PRINT(sigma_pos);
            INFO_PRINT(F(" sigma_vel="));
            INFO_PRINT(sigma_vel);
            INFO_PRINT(F(" sigma_tof="));
            INFO_PRINTLN(sigma_tof);
            req.new_response().end();
        }

        void reset(BLERequest &req) {
            kf.reset();
            motor_input = 0.0f;

            log_times.clear();
            log_dist.clear();
            log_vel.clear();

            // Large initial uncertainty
            kf.P(0, 0) = 1000;
            kf.P(1, 1) = 1000;

            INFO_PRINTLN(F("KF reset"));
            req.new_response().end();
        }

    } // namespace commands

    void init() {
        methods::update_matrices();

        // Large initial uncertainty
        kf.P(0, 0) = 1000;
        kf.P(1, 1) = 1000;

        ble::methods::register_command(KF_PARAMS, commands::set_params);
        ble::methods::register_command(KF_RESET, commands::reset);
        ble::methods::register_command(SEND_KF_DATA, commands::send_data);
    }

    void periodic() {
        // Feed current motor PWM as control input
        motor_input = (motors::current_left + motors::current_right) / 2.0f;

        // Predict every loop iteration
        methods::predict(motor_input);

        // Update only when a new ToF reading arrives
        if (tof::times1.size() > 0 && tof::times1[0] != last_tof_time) {
            last_tof_time = tof::times1[0];
            methods::update((float)tof::dist1[0]);
        }

        // Log
        log_times.push((int)timer::methods::time_us());
        log_dist.push((int)methods::distance());
        log_vel.push((int)methods::velocity());
    }

} // namespace kalman
