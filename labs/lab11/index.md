---
layout: page
title: "Lab 11: Localization (real)"
permalink: /labs/lab11/
---

# Lab 11: Localization (real)

## Introduction

Lab 11 ports the grid Bayes filter from Lab 10 onto the physical Artemis robot. Only the update step runs, because wheel odometry on this platform is too noisy to inform the prediction step. The robot performs an in-place 360 degree sweep, samples its two ToF sensors at 18 angles spaced 20 degrees apart, and ships the range vector to the host. The host runs a single update step against the precached ray map and reports the posterior at four marked poses on the lab floor.

## Observation Loop

The lab handout provides a legacy string-protocol BLE client and asks the student to copy four files into the notebooks directory. My BLE client uses a binary packet protocol I built in earlier labs. Porting Lab 11 onto it was a matter of converting the few commands the notebook needs into typed dataclasses on the new client. The new system returns values directly from each `await ble.execute(command)` call, which is cleaner than the legacy subscriber pattern where the response arrives later through a UUID-keyed callback and has to be wired back to the caller by hand.

```python
async def perform_observation_loop(self, rot_vel=120):
    await self.ble.execute(MapStart(num_steps=18))
    while await self.ble.execute(MapStatus()):
        await asyncio.sleep(0.5)

    buckets = await self.ble.execute(SendMapData())
    buckets.sort(key=lambda b: b.index)
    ranges_m = np.array([b.distance / 1000.0 for b in buckets])
    return ranges_m[np.newaxis].T, np.array([])
```

`BaseLocalization.get_observation_data` was promoted to `async` so the coroutine can be awaited end to end. The notebook awaits the connection at startup with `await ble.__aenter__()` and awaits the update step in the same cell that calls `loc.update_step()`.

## Results

<!-- TODO: brief paragraph summarizing localization accuracy across all four poses -->

### Pose 1: (-3 ft, -2 ft, 0 deg)

<!-- TODO: belief heatmap image, ground truth marker, belief argmax marker -->
<!-- TODO: short paragraph: argmax cell, posterior probability, xy error in meters -->

### Pose 2: (0 ft, 3 ft, 0 deg)

<!-- TODO: belief heatmap image -->
<!-- TODO: short paragraph -->

### Pose 3: (5 ft, -3 ft, 0 deg)

<!-- TODO: belief heatmap image -->
<!-- TODO: short paragraph -->

### Pose 4: (5 ft, 3 ft, 0 deg)

<!-- TODO: belief heatmap image -->
<!-- TODO: short paragraph -->

### Per pose statistics

<!-- TODO: fill table after collecting data -->

| pose            | belief (x, y, yaw)  | prob  | ground truth (x, y, yaw) | xy err (m) |
| --------------- | ------------------- | ----- | ------------------------ | ---------- |
| (-3, -2, 0)     | TODO                | TODO  | (-0.914, -0.610, 0)      | TODO       |
| (0, 3, 0)       | TODO                | TODO  | (0.000, 0.914, 0)        | TODO       |
| (5, -3, 0)      | TODO                | TODO  | (1.524, -0.914, 0)       | TODO       |
| (5, 3, 0)       | TODO                | TODO  | (1.524, 0.914, 0)        | TODO       |

### Sweep video

<!-- TODO: embed video of one full sweep at one pose -->
<!--
<video controls width="100%">
  <source src="images/sweep.mp4" type="video/mp4">
</video>
-->

## Discussion

<!-- TODO: which poses localized best and why. Compare to features visible from each pose. The corner poses sit near two perpendicular walls and produce highly discriminative range patterns. Interior poses are closer to the center and may share several rays with neighboring cells, giving a less peaky posterior. Refer to actual numbers once collected. -->

<!-- TODO: failure modes observed during data collection. Drift in start_yaw between sweeps, ToF saturation past the long range mode ceiling, or PID overshoot at a stop angle. -->

<!-- TODO: comparison to the simulator. The simulator posterior in Lab 10 collapsed onto a single cell at almost every step. Real measurements introduce per ray bias from sensor mounting and from sensor 2's 90 degree mounting offset. Note the effect on belief sharpness. -->
