---
layout: page
title: "Lab 4: Motor Drivers and Open Loop Control"
permalink: /labs/lab4/
---

# Lab 4: Motor Drivers and Open Loop Control

## Prelab

### Wiring Diagram

Each dual motor driver (DRV8833) has two H-bridge channels. Since the robot needs more current than a single channel can handle, we parallel-couple both channels on each driver — both inputs tied together, both outputs tied together. This doubles the average current capacity without overheating the chip. It's safe to do because both channels share the same clock on one IC.

So the setup is:

- **Motor Driver 1** → Left motor (both channels paralleled)
- **Motor Driver 2** → Right motor (both channels paralleled)

<!-- TODO: Draw wiring diagram showing: -->
<!-- - Artemis GPIO pins → Motor Driver 1 (AIN1, AIN2) and Motor Driver 2 (BIN1, BIN2) -->
<!-- - Motor Driver outputs → motors -->
<!-- - 850mAh battery → motor driver VIN -->
<!-- - 650mAh battery → Artemis -->
<!-- - Common ground between Artemis and motor drivers -->
<!-- TODO: Specify which Artemis pins you're using (must support PWM — check the pin diagram) -->

### Pin Selection

<!-- TODO: Pick the specific Artemis pins for motor control. Consider: -->
<!-- - Pins must support analogWrite (PWM) -->
<!-- - Physical location on the board — shorter wires = less noise + easier routing -->
<!-- - You need 4 PWM pins total (2 per motor driver for direction control) -->
<!-- Look at the Artemis Nano pin diagram: https://cdn.sparkfun.com/assets/5/5/1/6/3/RedBoard-Artemis-Nano.pdf -->

### Battery Discussion

The Artemis and the motors use **separate batteries** on purpose:

- **Motors are noisy.** When a motor starts, stalls, or reverses, it draws huge current spikes. These spikes cause voltage drops that can reset the Artemis or corrupt sensor readings.
- **Separate power rails** keep the digital logic clean and stable while the motors do their thing.

The 850mAh Li-Ion battery powers the motors (higher current capacity), and the 650mAh battery powers the Artemis and sensors.

### Wiring Considerations

<!-- TODO: Think about: -->
<!-- - Keep motor wires short to reduce EMI -->
<!-- - Use stranded wire (not solid core) — the car hits things and accelerates hard, solid core will fatigue and snap -->
<!-- - Color code your wires for sanity -->
<!-- - Route wires so they don't snag on wheels or get pinched by the chassis -->

---

## Motor Driver Setup

### External Power Supply Testing

First, I tested one motor driver powered from an external power supply (not the battery). This makes debugging easier because you can set current limits and see exactly what's happening.

<!-- TODO: What voltage and current limit did you set on the power supply? -->
<!-- (The motors are rated for ~3.7V from the Li-Ion battery. A reasonable starting point might be 3.7V with a 1-2A current limit) -->
<!-- TODO: Photo of your bench setup with the power supply, oscilloscope, and motor driver -->

### PWM Signal Verification

I used `analogWrite()` to generate PWM signals to the motor driver inputs and verified the output on an oscilloscope.

<!-- TODO: Code snippet showing your analogWrite test code -->
<!-- TODO: Oscilloscope screenshot showing the PWM waveforms on the motor driver output -->
<!-- TODO: Show a few different duty cycles (e.g., 25%, 50%, 75%) so it's clear you can control the power -->

```cpp
// TODO: Paste your analogWrite test code here
// Example structure:
// analogWrite(PIN_MOTOR1_FWD, 128);  // 50% duty cycle forward
// analogWrite(PIN_MOTOR1_REV, 0);    // not reversing
```

### Single Motor Spin Test

With the car on its side (wheels off the ground), I confirmed the motor spins in both directions.

<!-- TODO: Short video of wheels spinning forward and backward -->
<!-- TODO: Include the code snippet that controls direction -->

---

## Battery Power

After verifying everything works on the bench supply, I switched to the 850mAh battery for motor power.

<!-- TODO: Double check wire polarity before plugging in the battery! -->
<!-- TODO: Confirm both motors still work on battery power -->
<!-- TODO: Short video showing both wheels spinning with battery power (no USB tether) -->

---

## Assembly

I mounted everything inside the RC car chassis:

<!-- TODO: Photo(s) of all components installed in the car -->
<!-- TODO: Consider labeling the photo if components are hard to see -->
<!-- TODO: Keep components below the wheel line so they don't stick out when the car flips -->

Things to keep in mind:

- The car is fast and _will_ flip, so nothing should stick out past the wheels
- Add a timer in code so the car stops after a few seconds — otherwise you'll be chasing it down the hallway

---

## Lower PWM Limit

I experimented with the minimum PWM value needed to get the robot moving.

<!-- TODO: What's the minimum PWM value to start moving forward from rest? -->
<!-- TODO: What's the minimum PWM value for on-axis turns from rest? -->
<!-- TODO: Note that starting from rest requires more power than keeping it moving (static vs kinetic friction) -->

---

## Calibration

The motors probably don't spin at exactly the same rate, so the robot drifts to one side. I added a calibration factor to compensate.

<!-- TODO: What calibration factor did you end up with? (e.g., left motor at 100%, right motor at 95%) -->
<!-- TODO: Video of the robot driving in a reasonably straight line for at least 2m/6ft -->
<!-- TODO: The robot should start on a tape line and still overlap with it at the end -->
<!-- TODO: Code snippet showing how you apply the calibration factor -->

---

## Open Loop Demo

Finally, I programmed a sequence of moves (forward, turn, forward, turn, etc.) and ran the robot untethered.

<!-- TODO: Video of open loop control with turns -->
<!-- TODO: Code snippet of your move sequence -->
<!-- TODO: Brief discussion of how repeatable it is (spoiler: open loop is not very repeatable) -->

---

## 5000-Level Tasks

<!-- TODO: (5000 level) What frequency does analogWrite generate? Use the oscilloscope to measure it -->
<!-- TODO: (5000 level) Is this frequency fast enough for these motors? What happens if you increase it? -->
<!-- TODO: (5000 level) Benefits of manually configuring timers for faster PWM? (less audible whine, smoother control at low speeds) -->
<!-- TODO: (5000 level) Find the lowest PWM value that keeps the robot moving once it's already in motion (vs the higher value needed to start from rest) -->
<!-- TODO: (5000 level) How quickly can you settle to the slowest speed? (Start with a kick, then drop to the minimum sustaining value) -->
