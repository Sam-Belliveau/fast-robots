---
layout: page
title: "Lab 8: Stunts!"
permalink: /labs/lab8/
---

# Lab 8: Stunts!

## Task

I completed both stunts. For Task A (flip), the robot drives forward at constant speed, when its close enough to my wall, it quickly changes direction, causing it to flip end over end, and drives back. For Task B (drift), the robot drives forward and initiates a 180 degree turn once the wall is within the same distance. Both runs start less than 2 m from the wall. (I was in my appartment because of break + traveling for a conference)

## Motor Subsystem Redesign

To run stunts, manual driving, and angle PID without the subsystems stepping on each other, I rewrote the motor subsystem to accept multiple inputs. Any subsystem can register a pair of float pointers as a source. Each periodic tick, the motor subsystem sums every registered source, normalizes, and drives the hardware.

```cpp
static constexpr int MAX_SOURCES = 16;
struct Source { float *left; float *right; };
Source sources[MAX_SOURCES];

void register_source(float *left, float *right) {
    if (num_sources < MAX_SOURCES) {
        sources[num_sources++] = {left, right};
    }
}
```

The update step normalizes each source individually, sums them, then renormalizes the total to the PWM range. This keeps any single source from saturating the output and lets thrust, yaw, and operator input add cleanly:

This let me move the stunts into their own subsystem. The stunt subsystem registers two sources. One source is a scalar thrust that writes the same value to both wheels. The other is a differential yaw pair driven by a yaw PID controller. The web UI registers its own pair for manual joystick control. All three can run at once without any central mixer.

## Stunt Logic

The stunt subsystem holds a small state machine with two modes. Both modes track distance from the Kalman filter and heading from the DMP yaw.

### Drift Logic

Drift mode drives forward at a constant approach speed while the yaw PID holds the starting heading. When the distance falls below the trigger, the yaw setpoint jumps by 180 degrees and the PID swings the robot through the turn:

```cpp
float distance = kalman::methods::distance();

if (!triggered && distance < trigger_distance && distance > 0) {
    yaw_setpoint = imu::yaw + 180.0f;
    triggered = true;
}
thrust = approach_speed;
update_yaw();
```

### Flip Logic

Flip mode drives forward under the same yaw hold. When the trigger fires, the stunt commits to reverse thrust so the robot drives out from under the wall after the flip:

```cpp
float distance = kalman::methods::distance();

if (!triggered) {
    thrust = approach_speed;
    update_yaw();
    if (distance < trigger_distance && distance > 0) {
        triggered = true;
    }
} else {
    thrust = -return_speed;
    yaw_left = 0.0f;
    yaw_right = 0.0f;
}
```

## Kalman Filter Usage

The distance used in the logic was from the Kalman filter output. Because each stunt only defined a periodic loop, the Kalman filter from lab 7 was able to be used without modification. The Kalman filter already ran prediction steps every loop and update steps when new ToF data arrived, so no changes were needed to accommodate the new stunt subsystem. I just called `kalman::methods::distance()` to get the latest estimate.

## Results

Drift run. The robot approaches the wall, rotates 180 degrees as soon as the Kalman distance drops below the trigger, and drives back out:

<video controls width="100%">
  <source src="images/DRIFT.MOV" type="video/mp4">
</video>

Flip run. The robot hits the sticky mat at full speed, flips over the front wheels, and reverses past the start line:

<video controls width="100%">
  <source src="images/FLIP.MOV" type="video/mp4">
</video>
