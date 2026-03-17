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

    enum State {
        RESET = 0,
        RANGING = 1,
    };

    // Methods

    namespace methods {

        static int extrapolate(
            const CircularBuffer<int, 0x10> &times,
            const CircularBuffer<int, 0x10> &dists
        ) {
            int n = times.size();
            if (n == 0)
                return -1;
            if (n < 2)
                return dists[0];

            if (n >= 3) {
                int dir_old = dists[1] - dists[2];
                int dir_new = dists[0] - dists[1];
                if ((dir_old > 0 && dir_new < 0) ||
                    (dir_old < 0 && dir_new > 0))
                    return dists[0];
            }

            int dt = times[0] - times[1];
            if (dt <= 0)
                return dists[0];

            int elapsed = (int)micros() - times[0];
            if (elapsed > 500000)
                return dists[0];

            float slope = (float)(dists[0] - dists[1]) / (float)dt;
            int extrapolated = dists[0] + (int)(slope * (float)elapsed);
            return extrapolated > 0 ? extrapolated : 0;
        }

        int current1() {
            return extrapolate(times1, dist1);
        }
        int current2() {
            return extrapolate(times2, dist2);
        }

        int update(State *state, SFEVL53L1X *sensor) {
            int distance = -1;
            switch (*state) {
            case RESET:
                sensor->startRanging();
                *state = RANGING;
                break;

            case RANGING:
                if (sensor->checkForDataReady()) {
                    distance = sensor->getDistance();
                    sensor->clearInterrupt();
                    sensor->stopRanging();
                    *state = RESET;
                } else {
                    *state = RANGING;
                }
                break;
            }

            return distance;
        }

        void read() {
            static State state1 = RESET;
            static State state2 = RESET;

            const int time = micros();
            int distance = -1;

            if (0 <= (distance = update(&state1, &sensor1))) {
                times1.push(time);
                dist1.push(distance);
            }

            if (0 <= (distance = update(&state2, &sensor2))) {
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

            SERIAL_PRINT(F("SEND_TOF_DATA: "));
            SERIAL_PRINT(dist1.size());
            SERIAL_PRINT(F(" S1 samples, "));
            SERIAL_PRINT(dist2.size());
            SERIAL_PRINTLN(F(" S2 samples"));
        }

        void short_mode(BLERequest &req) {
            sensor1.setDistanceModeShort();
            sensor2.setDistanceModeShort();
            SERIAL_PRINTLN(F("ToF: short mode"));
            req.new_response().end();
        }

        void long_mode(BLERequest &req) {
            sensor1.setDistanceModeLong();
            sensor2.setDistanceModeLong();
            SERIAL_PRINTLN(F("ToF: long mode"));
            req.new_response().end();
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

            SERIAL_PRINT(F("TOF_STATS: S1 n="));
            SERIAL_PRINT(dist1.size());
            SERIAL_PRINT(F(", S2 n="));
            SERIAL_PRINTLN(dist2.size());
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
            SERIAL_PRINTLN(F("WARNING: ToF sensor 2 failed to begin..."));
            delay(100);
        }

        sensor2.setI2CAddress(0x54);
        SERIAL_PRINTLN(F("ToF sensor 2 online at address 0x54!"));

        // Bring sensor 1 back up
        delay(100);
        digitalWrite(TOF1_XSHUT_PIN, HIGH);
        delay(100);

        while (sensor1.begin() != 0) {
            SERIAL_PRINTLN(F("WARNING: ToF sensor 1 failed to begin..."));
            delay(100);
        }

        SERIAL_PRINTLN(F("ToF sensor 1 online!"));

        sensor1.setDistanceModeShort();
        sensor2.setDistanceModeShort();

        ble::methods::register_command(SEND_TOF_DATA, commands::send_data);
        ble::methods::register_command(TOF_SHORT, commands::short_mode);
        ble::methods::register_command(TOF_LONG, commands::long_mode);
        ble::methods::register_command(TOF_STATS, commands::stats);
    }

    // Periodic

    void periodic() {
        methods::read();
    }

} // namespace tof
