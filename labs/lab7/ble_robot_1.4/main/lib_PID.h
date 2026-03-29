#pragma once

#include "lib_LowPassFilter.h"
#include "subsystem_timer.h"

// PID controller with integrator wind-up protection and
// derivative-on-measurement. Uses the timer subsystem's fixed dt.

class PID {
  public:
    float kP = 0;
    float kI = 0;
    float kD = 0;

    // Integrator wind-up clamp.
    float integrator_cap = 0;
    float integrator_range = 0;

    // Low-pass filter for derivative term. Call d_filter.set_rc() to
    // control smoothing.
    LowPassFilter d_filter;

    void reset() {
        integral = 0;
        prev_error = 0;
        first = true;
        d_filter.reset();
    }

    // Compute PID output given a measurement.
    // Uses timer::PERIOD_S as the fixed dt.
    // If velocity is provided (not NAN), it is used directly for the
    // derivative term instead of computing it from successive measurements.
    float compute(
        float measurement,    //
        float setpoint = 0,   //
        float velocity = NAN  //
    ) {
        constexpr float dt = timer::PERIOD_S;

        float error = setpoint - measurement;

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

        // Derivative: use provided velocity or compute from measurement
        if (!isnan(velocity)) {
            d_out = kD * d_filter.filter(-velocity);
            first = false;
        } else if (first) {
            d_out = 0;
            d_filter.reset();
            d_filter.filter(0);
            first = false;
        } else {
            float raw_derivative = (prev_measurement - measurement) / dt;
            d_out = kD * d_filter.filter(raw_derivative);
        }

        prev_error = error;
        prev_measurement = measurement;

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
    bool first = true;
};
