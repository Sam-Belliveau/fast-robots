#pragma once

// Timer subsystem
// Owns: fixed-rate loop timing.
// Provides constexpr alpha computation for low-pass filters at the
// known fixed dt, eliminating per-cycle floating point work.

#include <math.h>

namespace timer {

    // Variables

    constexpr float FREQ_HZ = 64.0f;
    constexpr float PERIOD_S = 1.0f / FREQ_HZ;
    constexpr unsigned long PERIOD_US = (unsigned long)(PERIOD_S * 1e6);
    constexpr unsigned long MAX_DELAY_US = 1000000; // 1 second

    unsigned long sleep_until = 0;

    unsigned long prev_tick = 0;
    float avg_dt_us = PERIOD_US; // exponential moving average of actual dt

    // Methods

    namespace methods {

        constexpr inline float alpha(float rc) {
            return 1.0f - expf(-PERIOD_S / rc);
        }

        constexpr inline unsigned long period_us() {
            return PERIOD_US;
        }

        constexpr inline float period_s() {
            return PERIOD_S;
        }

        inline unsigned long time_us() {
            return sleep_until;
        }

        inline float time_s() {
            return time_us() * 1e-6f;
        }

        inline float avg_hz() {
            return 1e6f / avg_dt_us;
        }

    } // namespace methods

    // Init

    void init() {
        sleep_until = micros() + PERIOD_US;
        prev_tick = micros();
    }

    // Periodic — call at the END of the loop to sleep until the next tick

    void periodic() {
        unsigned long now = micros();

        // Track average dt
        unsigned long actual_dt = now - prev_tick;
        prev_tick = now;

        constexpr float a = methods::alpha(1.0f);
        avg_dt_us += a * (actual_dt - avg_dt_us);

        if (now < sleep_until) {
            delayMicroseconds(sleep_until - now);
        } else if (sleep_until + MAX_DELAY_US < now) {
            sleep_until = now + PERIOD_US;
            ERROR_PRINT(F("Timer overrun (avg "));
            ERROR_PRINT(methods::avg_hz(), 1);
            ERROR_PRINTLN(F(" Hz)"));
        }

        sleep_until += PERIOD_US;
    }

} // namespace timer
