#pragma once

#include "lib_LowPassFilter.h"

// PID controller with integrator wind-up protection and
// derivative-on-measurement. Uses LowPassFilter for the derivative term.

class PID {
  public:
    float kP = 0;
    float kI = 0;
    float kD = 0;

    // Integrator wind-up clamp.
    float integrator_cap = 0;
    float integrator_range = 0;

    // Low-pass filter for derivative term. Set d_filter.rc to control
    // smoothing.
    LowPassFilter d_filter;

    void reset() {
        integral = 0;
        prev_error = 0;
        prev_time = -1;
        d_filter.reset();
    }

    // Compute PID output given a measurement.
    // Calls micros() internally and tracks dt since last call.
    float compute(float measurement, float setpoint = 0) {
        long now_us = micros();
        bool first = (prev_time == -1);
        if (first)
            prev_time = now_us - 1;

        float error = setpoint - measurement;

        long dt_us = now_us - prev_time;
        float dt = dt_us * 1e-6f;

        // Proportional
        p_out = kP * error;

        // Integral
        if (integrator_range == 0 ||
            (-integrator_range < error && error < integrator_range)) {
            integral += error * dt;
            integral = constrain(integral, -integrator_cap, integrator_cap);
        } else {
            integral = 0;
        }

        i_out = kI * integral;

        // Derivative on measurement (skip first call to avoid spike from
        // uninitialized prev_measurement)
        if (first) {
            d_out = 0;
            d_filter.reset();
            d_filter.filter(0);
        } else {
            float raw_derivative = (prev_measurement - measurement) / dt;
            d_out = kD * d_filter.filter(raw_derivative);
        }

        prev_error = error;
        prev_measurement = measurement;
        prev_time = now_us;

        return p_out + i_out + d_out;
    }

    // Branch outputs
    float p_out = 0;
    float i_out = 0;
    float d_out = 0;

  private:
    float integral = 0;
    float prev_error = 0;
    float prev_measurement = 0;
    long prev_time = -1;
};
