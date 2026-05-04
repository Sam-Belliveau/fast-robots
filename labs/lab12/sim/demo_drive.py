"""Live matplotlib demo: simulated robot drives a path while a 2D
Bayes filter (motion-model predict + ToF update) tracks the pose.

Run from this directory:

    python demo_drive.py

For a step-through, interactive version, see lab12.ipynb in the same
directory.
"""

from __future__ import annotations

import asyncio
import time

import matplotlib.pyplot as plt

from world import load_world
from sim_robot import SimRobot
from localization2d import Localization2D
from controller import SplineController, BeliefPlot, step_once
from paths import LAB_PATH_M


PATH = LAB_PATH_M
START_THETA_DEG = 45.0


async def main():
    map_lines, bounds, loc_params, sim_params, ctl_kwargs = load_world()
    sim = SimRobot(map_lines, sim_params, x0=PATH[0][0], y0=PATH[0][1],
                   theta_deg=START_THETA_DEG)

    print("Building localization (includes ray cache)...")
    t0 = time.time()
    loc = Localization2D(map_lines, bounds, params=loc_params)
    print(f"  grid={loc.bel.shape}  cache={loc.cache.mean_mm.shape}  "
          f"build={time.time()-t0:.2f}s")
    loc.init_at(PATH[0][0], PATH[0][1], sigma_m=0.10)

    ctl = SplineController(path=PATH, loc=loc, map_lines=map_lines, **ctl_kwargs)
    _, spline_xy = ctl.spline.dense_sample(0.01)
    plot = BeliefPlot(loc, map_lines, path=PATH, spline_xy=spline_xy)

    plt.ion()
    last_t_us, last_yaw, step = 0, 0.0, 0
    while plt.fignum_exists(plot.fig.number) and not ctl.done:
        resp, last_t_us = await step_once(
            sim, loc, ctl, last_t_us=last_t_us, last_yaw_deg=last_yaw,
        )
        last_yaw = resp.yaw_deg
        gx, gy, _ = sim.pose
        plot.update((gx, gy), resp,
                    title=f"step {step}  t={sim.t_s:.2f}s  s={ctl.s_prog:.2f}/{ctl.spline.S:.2f} m")
        try:
            plt.pause(0.02)
        except Exception:
            break
        step += 1

    print(f"done. steps={step}, final pose={sim.pose}, "
          f"belief mean={loc.mean_xy()}")
    plt.ioff()
    plt.show()


if __name__ == "__main__":
    asyncio.run(main())
