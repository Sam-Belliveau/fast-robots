#pragma once

// First-order RC low-pass filter using the timer subsystem's fixed dt.
// Call set_rc() to set the time constant; alpha is cached to avoid
// per-call exp().

#include "subsystem_timer.h"

class LowPassFilter {
public:
    void set_rc(float rc_seconds) {
        _rc = rc_seconds;
        _alpha = timer::methods::alpha(rc_seconds);
    }

    float rc() const { return _rc; }

    void reset() {
        first = true;
        value = 0;
    }

    float filter(float input) {
        if (first || _rc <= 0) {
            value = input;
            first = false;
            return value;
        }

        value += _alpha * (input - value);
        return value;
    }

    float value = 0;

private:
    float _rc = 0;
    float _alpha = 0;
    bool first = true;
};
