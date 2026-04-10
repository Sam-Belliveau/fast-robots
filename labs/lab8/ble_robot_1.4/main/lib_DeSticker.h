#pragma once

// DeSticker: Stiction-aware motor controller.
// Accumulates requested PWM as energy. When enough accumulates,
// fires a fixed-duration kick pulse. Blends PDM and raw output
// across a configurable transition zone.

#include "subsystem_timer.h"

class DeSticker {
  public:
    DeSticker(
        int thresh_low,
        int thresh_high,
        int kick_pwm = 255,
        unsigned long pulse_us = 30000,
        unsigned long pulse_ext_us = 10000
    )
        : thresh_low(thresh_low), thresh_high(thresh_high), kick_pwm(kick_pwm),
          pulse_us(pulse_us), pulse_ext_us(pulse_ext_us) {}

    int update(int pwm) {
        int mag = abs(pwm);

        if (mag <= thresh_low)
            pdm_mode = true;

        if (mag >= thresh_high)
            pdm_mode = false;

        if (pdm_mode && mag > 0) {
            accumulator += timer::methods::period_us();
            accumulator %= pulse_us * kick_pwm / mag;
        } else {
            accumulator = 0;
        }

        int pdm_out = (pwm > 0) ? kick_pwm : (pwm < 0) ? -kick_pwm : 0;
        pdm_out = (accumulator <= pulse_us + pulse_ext_us) ? pdm_out : 0;

        return pdm_mode ? pdm_out : pwm;
    }

  private:
    unsigned long accumulator = 0;
    bool pdm_mode = false;

    int thresh_low;
    int thresh_high;
    int kick_pwm;

    unsigned long pulse_us;
    unsigned long pulse_ext_us;
};
