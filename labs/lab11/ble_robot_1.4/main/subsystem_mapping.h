#pragma once

// Mapping subsystem
// Drives the robot through a full rotation, stopping every angular step to
// collect a fixed number of unique ToF samples from both sensors. Each
// sample is fused into an AngleMap with a Gaussian angular kernel so the
// result is a smooth 360 degree distance estimate instead of a sparse set
// of step-aligned readings.
//
// Flow: MAP_START clears the map and commands angle PID to start_yaw.
//       Each step rotates by 360/num_steps degrees, waits for the PID
//       to settle, then requires N unique fresh ToF samples from both
//       sensors before advancing. Sensor 2 is mounted 90 degrees
//       clockwise of sensor 1 so its beam is fused at yaw + offset.

#include "lib_AngleMap.h"
#include "subsystem_angle_pid.h"
#include "subsystem_ble.h"
#include "subsystem_imu.h"
#include "subsystem_serial.h"
#include "subsystem_tof.h"

namespace mapping {

    static constexpr int MAP_BUCKETS = 18;

    // Configuration
    int num_steps = 18;            // 18 steps = 20 deg each
    int samples_per_step = 25;     // unique fresh ToF samples per step
    float settle_threshold = 5.0f; // deg, PID error considered settled
    unsigned long settle_time_ms =
        500; // ms held under threshold before sampling
    unsigned long step_timeout_ms = 8000; // ms max per phase
    float sensor2_offset_deg = -90.0f;    // sensor 2 beam offset (clockwise)
    float angle_std_deg = 10.0f;          // Gaussian kernel std fed to AngleMap

    // Radial mounting offsets from rotation center, in mm. Each sensor reads
    // the distance from its lens to a wall, so true distance from the robot
    // origin along that beam is reading + radial offset.
    static constexpr float SENSOR1_OFFSET_MM = 70.0f; // front sensor
    static constexpr float SENSOR2_OFFSET_MM = 31.0f; // side sensor

    // State
    AngleMap<MAP_BUCKETS> map;

    enum Phase { IDLE, TURNING, SAMPLING };
    Phase phase = IDLE;
    bool active = false;
    int current_step = 0;
    float start_yaw = 0;
    unsigned long phase_start_ms = 0;
    unsigned long settle_start_ms = 0;
    bool settled = false;

    int collected1 = 0;
    int collected2 = 0;
    unsigned long baseline_reads1 = 0;
    unsigned long baseline_reads2 = 0;

    namespace methods {

        void begin_turning() {
            const float target =
                start_yaw + (float)current_step * (360.0f / (float)num_steps);
            angle_pid::setpoint = target;
            angle_pid::controller.reset();
            angle_pid::active = true;
            phase = TURNING;
            phase_start_ms = millis();
            settled = false;
        }

        void begin_sampling() {
            phase = SAMPLING;
            collected1 = 0;
            collected2 = 0;
            baseline_reads1 = tof::reads1 + 1;
            baseline_reads2 = tof::reads2 + 1;
            phase_start_ms = millis();
            // Cut motor drive so the chassis is mechanically still during
            // sampling; otherwise PID corrections keep nudging the yaw and
            // smear bearings across buckets.
            angle_pid::active = false;
            angle_pid::output_left = 0.0f;
            angle_pid::output_right = 0.0f;
        }

        void finish() {
            active = false;
            phase = IDLE;
            angle_pid::active = false;
            angle_pid::output_left = 0;
            angle_pid::output_right = 0;
            INFO_PRINTLN(F("Mapping complete"));
        }

        void step_done() {
            current_step++;
            if (current_step >= num_steps) {
                finish();
                return;
            }
            begin_turning();
        }

        void update_turning() {
            const float err = fabsf(
                angle_pid::methods::wrap_angle(imu::yaw - angle_pid::setpoint)
            );

            if (err < settle_threshold) {
                if (!settled) {
                    settled = true;
                    settle_start_ms = millis();
                } else if (millis() - settle_start_ms >= settle_time_ms) {
                    begin_sampling();
                    return;
                }
            } else {
                settled = false;
            }

            if (millis() - phase_start_ms >= step_timeout_ms) {
                INFO_PRINT(F("Map turn timeout step="));
                INFO_PRINTLN(current_step);
                begin_sampling();
            }
        }

        void update_sampling() {
            const float rel_yaw = imu::yaw - start_yaw;

            if (collected1 < samples_per_step &&
                tof::reads1 > baseline_reads1) {
                baseline_reads1 = tof::reads1;
                const int raw = tof::dist1.size() > 0 ? tof::dist1[0] : -1;
                if (raw > 0) {
                    const float dist = (float)raw + SENSOR1_OFFSET_MM;
                    map.update(rel_yaw, dist, angle_std_deg);
                    collected1++;
                }
            }

            if (collected2 < samples_per_step &&
                tof::reads2 > baseline_reads2) {
                baseline_reads2 = tof::reads2;
                const int raw = tof::dist2.size() > 0 ? tof::dist2[0] : -1;
                if (raw > 0) {
                    const float dist = (float)raw + SENSOR2_OFFSET_MM;
                    map.update(
                        rel_yaw + sensor2_offset_deg, dist, angle_std_deg
                    );
                    collected2++;
                }
            }

            if (collected1 >= samples_per_step &&
                collected2 >= samples_per_step) {
                INFO_PRINT(F("Map step "));
                INFO_PRINT(current_step);
                INFO_PRINT(F(" yaw="));
                INFO_PRINTLN(imu::yaw);
                step_done();
                return;
            }

            if (millis() - phase_start_ms >= step_timeout_ms) {
                INFO_PRINT(F("Map sample timeout step="));
                INFO_PRINT(current_step);
                INFO_PRINT(F(" c1="));
                INFO_PRINT(collected1);
                INFO_PRINT(F(" c2="));
                INFO_PRINTLN(collected2);
                step_done();
            }
        }

        void update() {
            if (!active)
                return;
            if (!imu::yaw_valid)
                return;

            if (phase == TURNING)
                update_turning();
            else if (phase == SAMPLING)
                update_sampling();
        }

    } // namespace methods

    namespace commands {

        void start(BLERequest &req) {
            int32_t steps;
            BLE_CHECK_READ(req, req.read(steps), "steps");
            if (steps > 0)
                num_steps = steps;

            map.clear();

            imu::myICM.resetFIFO();
            delay(20);
            imu::methods::read_dmp();

            current_step = 0;
            start_yaw = imu::yaw;
            active = true;

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

            methods::begin_turning();

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
            int32_t settle, timeout, samples;
            float std_deg;
            BLE_CHECK_READ(req, req.read(threshold), "settle_threshold");
            BLE_CHECK_READ(req, req.read(settle), "settle_ms");
            BLE_CHECK_READ(req, req.read(timeout), "timeout_ms");
            BLE_CHECK_READ(req, req.read(samples), "samples_per_step");
            BLE_CHECK_READ(req, req.read(std_deg), "angle_std_deg");
            settle_threshold = threshold;
            settle_time_ms = (unsigned long)settle;
            step_timeout_ms = (unsigned long)timeout;
            if (samples > 0)
                samples_per_step = samples;
            if (std_deg > 0)
                angle_std_deg = std_deg;

            INFO_PRINT(F("Map params: thresh="));
            INFO_PRINT(threshold);
            INFO_PRINT(F(" settle="));
            INFO_PRINT(settle);
            INFO_PRINT(F(" timeout="));
            INFO_PRINT(timeout);
            INFO_PRINT(F(" samples="));
            INFO_PRINT(samples_per_step);
            INFO_PRINT(F(" std="));
            INFO_PRINTLN(angle_std_deg);
            req.new_response().end();
        }

        void status(BLERequest &req) {
            BLEResponse res = req.new_response();
            res.add((int32_t)(active ? 1 : 0));
            res.end();
        }

        void send_data(BLERequest &req) {
            BLEResponse res = req.new_response();
            for (int i = 0; i < MAP_BUCKETS; ++i) {
                const float center = AngleMap<MAP_BUCKETS>::bucket_center(i);
                const float dist = map.bucket(i);
                res.add((int32_t)i);
                res.add(center);
                res.add(dist);
            }
            res.end();

            INFO_PRINT(F("SEND_MAP_DATA: "));
            INFO_PRINT(MAP_BUCKETS);
            INFO_PRINTLN(F(" buckets"));
        }

    } // namespace commands

    void init() {
        ble::methods::register_command(MAP_START, commands::start);
        ble::methods::register_command(MAP_STOP, commands::stop_cmd);
        ble::methods::register_command(MAP_PARAMS, commands::set_params);
        ble::methods::register_command(SEND_MAP_DATA, commands::send_data);
        ble::methods::register_command(MAP_STATUS, commands::status);
    }

    void periodic() {
        methods::update();
    }

} // namespace mapping
