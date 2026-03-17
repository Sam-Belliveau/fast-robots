#pragma once

// First-order RC low-pass filter. Calls micros() internally.
// Set rc to the time constant in seconds (e.g. 0.1 for 100ms).

class LowPassFilter {
public:
  float rc = 0; // time constant in seconds. 0 = passthrough (no filtering)

  void reset() {
    first = true;
    value = 0;
  }

  float filter(float input) {
    unsigned long now = micros();

    if (first || rc <= 0) {
      value = input;
      prev_time = now;
      first = false;
      return value;
    }

    float dt = (now - prev_time) * 1e-6f;
    prev_time = now;

    float a = dt / (rc + dt);
    value = value + a * (input - value);
    return value;
  }

  float value = 0;

private:
  unsigned long prev_time = 0;
  bool first = true;
};

