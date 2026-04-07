#pragma once

// ToF subsystem
// Owns: VL53L1X sensors, ToF data buffers, extrapolation, init and periodic
// read.

#include "lib_CircularBuffer.h"
#include "SparkFun_VL53L1X.h"
#include "lib_Zip.h"
#include "subsystem_ble.h"
#include "subsystem_serial.h"

#define TOF1_XSHUT_PIN D5
#define TOF2_XSHUT_PIN D6

namespace tof {

    // Variables

    SFEVL53L1X sensor1(WIRE_PORT, TOF1_XSHUT_PIN);
    SFEVL53L1X sensor2(WIRE_PORT, TOF2_XSHUT_PIN);

    CircularBuffer<int, 0x10> times1;
    CircularBuffer<int, 0x10> dist1;

    CircularBuffer<int, 0x10> times2;
    CircularBuffer<int, 0x10> dist2;

    CircularBuffer<int, 0x100> plot_times;
    CircularBuffer<int, 0x100> plot_raw1;
    CircularBuffer<int, 0x100> plot_extrap1;

    CircularBuffer<int, 0x100> plot_raw2;
    CircularBuffer<int, 0x100> plot_extrap2;

    enum Mode { SHORT = 0, LONG = 1 };
    Mode mode1 = SHORT;
    Mode mode2 = SHORT;

    static constexpr int AUTO_SWITCH_UP = 1200;
    static constexpr int AUTO_SWITCH_DOWN = 900;

    static constexpr int MAX_SHORT_DISTANCE = 2000;
    static constexpr int MAX_LONG_DISTANCE = 4000;

    // Methods

    namespace methods {

        // Velocity from finite difference (mm/s).
        // Negative = approaching sensor. Returns 0 if insufficient data.
        static float velocity(
            const CircularBuffer<int, 0x10> &times,
            const CircularBuffer<int, 0x10> &dists
        ) {
            if (times.size() < 2)
                return 0.0f;
            int dt_us = times[0] - times[1];
            if (dt_us <= 0)
                return 0.0f;
            return (float)(dists[0] - dists[1]) / ((float)dt_us * 1e-6f);
        }

        static int extrapolate(
            const CircularBuffer<int, 0x10> &times,
            const CircularBuffer<int, 0x10> &dists
        ) {
            int n = times.size();
            if (n == 0)
                return -1;
            if (n < 2)
                return dists[0];

            float vel = velocity(times, dists);
            if (n >= 3) {
                int dir_old = dists[1] - dists[2];
                int dir_new = dists[0] - dists[1];
                if ((dir_old > 0 && dir_new < 0) ||
                    (dir_old < 0 && dir_new > 0))
                    return dists[0];
            }

            if (vel == 0.0f)
                return dists[0];

            int elapsed = (int)timer::methods::time_us() - times[0];
            if (elapsed > 500000)
                return dists[0];

            int extrapolated = dists[0] + (int)(vel * (float)elapsed * 1e-6f);
            return extrapolated > 0 ? extrapolated : 0;
        }

        int current1() {
            return extrapolate(times1, dist1);
        }
        int current2() {
            return extrapolate(times2, dist2);
        }
        float velocity1() {
            return velocity(times1, dist1);
        }
        float velocity2() {
            return velocity(times2, dist2);
        }

        // Time until distance reaches zero at current velocity (ms).
        // Returns -1 if not approaching (velocity >= 0) or no data.
        static float time_to_crash(
            const CircularBuffer<int, 0x10> &times,
            const CircularBuffer<int, 0x10> &dists
        ) {
            float vel = velocity(times, dists);
            if (vel >= 0.0f)
                return -1.0f;
            int dist = extrapolate(times, dists);
            if (dist <= 0)
                return 0.0f;
            return (float)dist / (-vel) * 1000.0f;
        }
        float time_to_crash1() {
            return time_to_crash(times1, dist1);
        }
        float time_to_crash2() {
            return time_to_crash(times2, dist2);
        }

        int update(SFEVL53L1X *sensor, Mode *sensor_mode) {
            if (!sensor->checkForDataReady())
                return -1;

            int distance = sensor->getDistance();
            sensor->clearInterrupt();

            if (distance <= 0) {
                switch (*sensor_mode) {
                case SHORT:
                    distance = MAX_SHORT_DISTANCE;
                    break;
                case LONG:
                    distance = MAX_LONG_DISTANCE;
                    break;
                default:
                    break;
                }
            }

            if (AUTO_SWITCH_UP < distance && *sensor_mode != LONG) {
                sensor->stopRanging();
                sensor->setDistanceModeLong();
                *sensor_mode = LONG;
                sensor->startRanging();
            } else if (distance < AUTO_SWITCH_DOWN && *sensor_mode != SHORT) {
                sensor->stopRanging();
                sensor->setDistanceModeShort();
                *sensor_mode = SHORT;
                sensor->startRanging();
            }

            return distance;
        }

        void read() {
            const int time = timer::methods::time_us();
            int distance = -1;

            if (0 <= (distance = update(&sensor1, &mode1))) {
                times1.push(time);
                dist1.push(distance);
            }

            if (0 <= (distance = update(&sensor2, &mode2))) {
                times2.push(time);
                dist2.push(distance);
            }

            plot_times.push(time);
            plot_raw1.push(dist1.size() > 0 ? dist1[0] : -1);
            plot_extrap1.push(current1());
            plot_raw2.push(dist2.size() > 0 ? dist2[0] : -1);
            plot_extrap2.push(current2());
        }

    } // namespace methods

    // Commands

    namespace commands {

        void send_data(BLERequest &req) {
            BLEResponse res = req.new_response();

            zip(
                [&](int t, int d, int e) {
                    res.add((int32_t)t);
                    res.add((int16_t)d);
                    res.add((int16_t)e);
                    res.add((int32_t)1); // sensor ID
                },
                plot_times.begin(),
                plot_times.end(),
                plot_raw1.begin(),
                plot_extrap1.begin()
            );

            zip(
                [&](int t, int d, int e) {
                    res.add((int32_t)t);
                    res.add((int16_t)d);
                    res.add((int16_t)e);
                    res.add((int32_t)2); // sensor ID
                },
                plot_times.begin(),
                plot_times.end(),
                plot_raw2.begin(),
                plot_extrap2.begin()
            );

            res.end();

            INFO_PRINT(F("SEND_TOF_DATA: "));
            INFO_PRINT(dist1.size());
            INFO_PRINT(F(" S1 samples, "));
            INFO_PRINT(dist2.size());
            INFO_PRINTLN(F(" S2 samples"));
        }

        void get_mode(BLERequest &req) {
            static const char *names[] = {"SHORT", "LONG"};
            BLEResponse res = req.new_response();
            res.add((int32_t)mode1);
            res.add((int32_t)mode2);
            res.end();
            INFO_PRINT(F("ToF mode: S1="));
            INFO_PRINT(names[mode1]);
            INFO_PRINT(F(" S2="));
            INFO_PRINTLN(names[mode2]);
        }

        void stats(BLERequest &req) {
            BLEResponse res = req.new_response();

            // Sensor 1 stats
            {
                const int n = dist1.size();
                float mean = 0, std_dev = 0;
                if (n > 0) {
                    float sum = 0;
                    for (auto d : dist1)
                        sum += d;
                    mean = sum / n;
                    float sq_sum = 0;
                    for (auto d : dist1) {
                        float diff = d - mean;
                        sq_sum += diff * diff;
                    }
                    std_dev = sqrt(sq_sum / n);
                }
                res.add((int32_t)1); // sensor ID
                res.add((int32_t)n);
                res.add(mean);
                res.add(std_dev);
            }

            // Sensor 2 stats
            {
                const int n = dist2.size();
                float mean = 0, std_dev = 0;
                if (n > 0) {
                    float sum = 0;
                    for (auto d : dist2)
                        sum += d;
                    mean = sum / n;
                    float sq_sum = 0;
                    for (auto d : dist2) {
                        float diff = d - mean;
                        sq_sum += diff * diff;
                    }
                    std_dev = sqrt(sq_sum / n);
                }
                res.add((int32_t)2); // sensor ID
                res.add((int32_t)n);
                res.add(mean);
                res.add(std_dev);
            }

            res.end();

            INFO_PRINT(F("TOF_STATS: S1 n="));
            INFO_PRINT(dist1.size());
            INFO_PRINT(F(", S2 n="));
            INFO_PRINTLN(dist2.size());
        }

    } // namespace commands

    // Init

    void init() {
        pinMode(TOF1_XSHUT_PIN, OUTPUT);
        pinMode(TOF2_XSHUT_PIN, OUTPUT);

        // Shut down sensor 1 via XSHUT
        digitalWrite(TOF1_XSHUT_PIN, LOW);
        digitalWrite(TOF2_XSHUT_PIN, HIGH);
        delay(100);

        while (sensor2.begin() != 0) {
            INFO_PRINTLN(F("WARNING: ToF sensor 2 failed to begin..."));
            delay(100);
        }

        sensor2.setI2CAddress(0x54);
        INFO_PRINTLN(F("ToF sensor 2 online at address 0x54!"));

        // Bring sensor 1 back up
        delay(100);
        digitalWrite(TOF1_XSHUT_PIN, HIGH);
        delay(100);

        while (sensor1.begin() != 0) {
            INFO_PRINTLN(F("WARNING: ToF sensor 1 failed to begin..."));
            delay(100);
        }

        INFO_PRINTLN(F("ToF sensor 1 online!"));

        sensor1.setDistanceModeShort();
        sensor2.setDistanceModeShort();
        sensor1.startRanging();
        sensor2.startRanging();

        ble::methods::register_command(SEND_TOF_DATA, commands::send_data);
        ble::methods::register_command(TOF_MODE, commands::get_mode);
        ble::methods::register_command(TOF_STATS, commands::stats);
    }

    // Periodic

    void periodic() {
        methods::read();
    }

} // namespace tof
