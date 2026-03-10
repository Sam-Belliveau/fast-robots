---
layout: page
title: "Lab 5: Linear PID and Linear Interpolation"
permalink: /labs/lab5/
---

# Lab 5: Linear PID and Linear Interpolation

## Prelab

### BLE Debugging Infrastructure

<!-- TODO: Describe the BLE command flow: computer sends start command, Artemis runs PID for fixed duration, stores data in arrays, then transmits back over BLE -->
<!-- TODO: Show Arduino code for storing timestamped PID debug data (sensor readings, P/I/D terms, motor input) -->
<!-- TODO: Show Python code for receiving and parsing the debug data -->
<!-- TODO: Mention hard stop safety mechanism on Artemis in case BLE disconnects -->

### BLE Gain Tuning

<!-- TODO: Describe BLE command for updating PID gains without reflashing -->

---

## P/I/D Discussion

### Controller Selection

<!-- TODO: State which controller type was chosen (P, PI, PD, or PID) and why -->

### Gain Selection

<!-- TODO: Explain how proportional gain range was estimated from motor input range (0-255) and TOF output range -->
<!-- TODO: Document final Kp, Ki, Kd values and how each was tuned -->
<!-- TODO: Discuss tradeoffs observed when adjusting each term -->

---

## Sensor Configuration

### TOF Range and Sampling Time

<!-- TODO: State which TOF range mode was selected (short/long) and why -->
<!-- TODO: Document sensor integration time setting and resulting update rate -->
<!-- TODO: Note the tradeoff between accuracy and speed -->

---

## PID Implementation

### Control Loop

<!-- TODO: Show the main PID loop code -->
<!-- TODO: Explain how the loop avoids blocking statements (no delay(), no polling loops) -->
<!-- TODO: Document loop timing and how it compares to the TOF update rate -->

### Deadband Handling

<!-- TODO: Show how PID output maps to motor PWM, accounting for the deadband found in Lab 4 -->

### Derivative Kick

<!-- TODO: Explain how derivative kick was addressed (derivative on measurement vs. derivative on error) -->

### Integrator Wind-up

<!-- TODO: Describe wind-up protection strategy (clamping, conditional integration, etc.) -->
<!-- TODO: Show performance with and without wind-up protection -->
<!-- TODO: Argue why wind-up protection is necessary (different floor surfaces, large initial errors) -->

---

## Linear Extrapolation

### TOF Data Rate

<!-- TODO: Measure and report actual TOF sensor update frequency -->

### Extrapolation Function

<!-- TODO: Show the extrapolation code: slope from two most recent readings, project forward based on elapsed time -->
<!-- TODO: Explain how PID loop runs faster than TOF by using extrapolated values between sensor updates -->

### Comparison

<!-- TODO: Plot raw TOF data vs. extrapolated data over time -->
<!-- TODO: Compare PID loop rate with and without extrapolation -->

---

## Results

### Speed

<!-- TODO: Document maximum linear speed achieved, computed from sensor data -->

### Plots

<!-- TODO: ToF distance vs. time plot showing approach and stop at 304mm setpoint -->
<!-- TODO: PID output vs. time plot showing P, I, D contributions -->
<!-- TODO: Motor input vs. time plot -->

### Robustness

<!-- TODO: Test from multiple starting distances (2-4 meters) -->
<!-- TODO: Test on different floor surfaces if possible -->
<!-- TODO: Test with external perturbation (push car toward/away from wall) -->

---

## Demo

<!-- TODO: Upload at least three videos of successful repeated runs -->
<!-- TODO: Show the robot approaching the wall at speed and stopping at 1 foot -->
