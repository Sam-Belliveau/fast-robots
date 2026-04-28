"""Waypoint controller and a small plotting helper.

The controller is intentionally dumb: from a given (x, y), point at
the next waypoint and drive at a fixed PWM until inside a stop
radius, then advance. Use it open-loop on ground-truth or closed-loop
on the belief argmax by passing the relevant pose into `command()`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import matplotlib.pyplot as plt
import numpy as np

from localization2d import Localization2D
from sim_robot import DriveUpdateResponse


# ----------------------------- controller ---------------------------

@dataclass
class WaypointController:
    """Sequentially aims at each waypoint; advances inside stop radius."""

    path: list[tuple[float, float]]
    stop_radius_mm: float = 250.0
    stop_flip_radius_mm: float = 250.0  # freeze flip decision inside this radius
    speed_pwm: float = 40.0            # ~500 mm/s at 12.5 mm/s per PWM
    align_tol_deg: float = 10.0        # within this, drive and re-evaluate flip
    idx: int = 1                       # path[0] is the start pose
    flip: bool = False                 # True = drive backwards toward the waypoint
    log: list[int] = field(default_factory=list)

    @property
    def done(self) -> bool:
        return self.idx >= len(self.path)

    @property
    def current_waypoint(self) -> tuple[float, float] | None:
        return None if self.done else self.path[self.idx]

    def command(self, x: float, y: float) -> tuple[float, float]:
        """Return (speed_pwm, target_heading_deg) for the next BLE call.

        Advances the waypoint index automatically when within the stop
        radius. Returns (0, last_heading) once the path is finished.
        """
        while not self.done:
            wx, wy = self.path[self.idx]
            dx, dy = wx - x, wy - y
            if math.hypot(dx, dy) * 1000.0 < self.stop_radius_mm:
                self.log.append(self.idx)
                self.idx += 1
                continue
            return self.speed_pwm, math.degrees(math.atan2(dy, dx))
        return 0.0, 0.0


# ------------------------------ plotting ----------------------------

class BeliefPlot:
    """Heatmap + ground truth + belief argmax + ToF rays + path overlay."""

    def __init__(
        self,
        loc: Localization2D,
        map_lines,
        path: list[tuple[float, float]] | None = None,
        figsize: tuple[float, float] = (10.0, 7.5),
    ):
        self.loc = loc
        self.fig, self.ax = plt.subplots(figsize=figsize)

        self.im = self.ax.imshow(
            loc.bel.T, origin="lower", extent=loc.extent,
            cmap="hot", vmin=0.0, vmax=loc.bel.max() + 1e-12, alpha=0.85,
        )
        for (x0, y0), (x1, y1) in map_lines:
            self.ax.plot([x0, x1], [y0, y1], "k-", lw=2)

        if path:
            px = [p[0] for p in path]
            py = [p[1] for p in path]
            self.ax.plot(px, py, "c--", lw=1.5, alpha=0.7, label="path")
            self.ax.plot(px, py, "c*", ms=12, alpha=0.7)

        self.gt_trail,  = self.ax.plot([], [], "g-", lw=1.0, alpha=0.7)
        self.bel_trail, = self.ax.plot([], [], "r-", lw=1.0, alpha=0.7)
        self.gt_pt,  = self.ax.plot([], [], "go", ms=10, label="ground truth")
        self.bel_pt, = self.ax.plot([], [], "rx", ms=12, mew=3, label="belief mean")
        self._gt_xs:  list[float] = []
        self._gt_ys:  list[float] = []
        self._bel_xs: list[float] = []
        self._bel_ys: list[float] = []
        self.ray1,     = self.ax.plot([], [], color="lime",        lw=1.5, alpha=0.9, label="tof1")
        self.ray1_hit, = self.ax.plot([], [], "o", mfc="none", mec="lime",        ms=8, mew=1.5)
        self.ray2,     = self.ax.plot([], [], color="deepskyblue", lw=1.5, alpha=0.9, label="tof2")
        self.ray2_hit, = self.ax.plot([], [], "o", mfc="none", mec="deepskyblue", ms=8, mew=1.5)

        self.title = self.ax.set_title("step 0")
        self.ax.set_aspect("equal")
        self.ax.legend(loc="upper left")
        self.fig.tight_layout()

    def update(
        self,
        gt_xy: tuple[float, float] | None,
        resp: DriveUpdateResponse,
        title: str = "",
    ):
        """Refresh all artists. Pass gt_xy=None on the real robot
        (no ground truth) -- ToF rays are then drawn from the belief
        mean instead, and the green GT trail/marker stays empty."""
        bx, by = self.loc.mean_xy()
        self.im.set_data(self.loc.bel.T)
        self.im.set_clim(vmin=0.0, vmax=self.loc.bel.max() + 1e-12)

        self._bel_xs.append(bx); self._bel_ys.append(by)
        self.bel_trail.set_data(self._bel_xs, self._bel_ys)
        self.bel_pt.set_data([bx], [by])

        if gt_xy is None:
            ox, oy = bx, by                # rays anchored at belief mean
        else:
            ox, oy = gt_xy
            self._gt_xs.append(ox); self._gt_ys.append(oy)
            self.gt_trail.set_data(self._gt_xs, self._gt_ys)
            self.gt_pt.set_data([ox], [oy])

        for ray, hit, dist_mm, bearing_deg in (
            (self.ray1, self.ray1_hit, resp.tof1_dist_mm, resp.tof1_yaw_deg),
            (self.ray2, self.ray2_hit, resp.tof2_dist_mm, resp.tof2_yaw_deg),
        ):
            if dist_mm and dist_mm > 0:
                br = math.radians(bearing_deg)
                d_m = dist_mm / 1000.0
                ex, ey = ox + d_m * math.cos(br), oy + d_m * math.sin(br)
                ray.set_data([ox, ex], [oy, ey])
                hit.set_data([ex], [ey])
            else:
                ray.set_data([ox, ox], [oy, oy])
                hit.set_data([ox], [oy])

        if title:
            self.title.set_text(title)


# ---------------------------- filter step ---------------------------

def filter_step(
    loc: Localization2D,
    resp: DriveUpdateResponse,
    speed_pwm: float,
    dt_s: float,
):
    """One Bayes filter cycle from a DriveUpdate response.

    Predict places a fresh Gaussian at (belief mean + v*dt) using
    LocParams.pwm_to_vel_mm_s; update folds in both ToF readings.
    Pure -- works the same against SimRobot or a real BLE response.
    """
    loc.predict(heading_deg=resp.yaw_deg, speed_pwm=speed_pwm, dt_s=dt_s)
    if resp.tof1_dist_mm > 0:
        loc.update(resp.tof1_yaw_deg, float(resp.tof1_dist_mm))
    if resp.tof2_dist_mm > 0:
        loc.update(resp.tof2_yaw_deg, float(resp.tof2_dist_mm))


# ---------------------------- one BLE step --------------------------

async def step_once(
    robot,
    loc: Localization2D,
    controller: WaypointController,
    *,
    use_belief: bool = True,
    last_t_us: int = 0,
    last_yaw_deg: float = 0.0,
) -> tuple[DriveUpdateResponse, int]:
    """Run one BLE round-trip: command -> robot.update -> filter_step.

    `robot` only needs `async def update(target_speed, target_heading)
    -> DriveUpdateResponse`. Both `SimRobot` and `RealRobot` satisfy
    this. `use_belief=False` falls back to ground truth from
    `robot.pose`, which only `SimRobot` provides.

    Two-state heading controller: drive forwards or backwards along
    the bearing to the waypoint. The flip recommendation is evaluated
    against the goal-pointing heading (not the body yaw) so that turns
    do not destabilize it, and frozen inside `stop_flip_radius_mm` of
    the waypoint to kill near-goal chatter. Drive speed is scaled by a
    Gaussian in heading error with width `align_tol_deg`.
    """
    if use_belief:
        x, y = loc.mean_xy()
    else:
        gx, gy, _ = robot.pose
        x, y = gx, gy

    max_speed, target_dir = controller.command(x, y)
    wp = controller.current_waypoint
    if wp is not None:
        dist_mm = math.hypot(wp[0] - x, wp[1] - y) * 1000.0
        if dist_mm > controller.stop_flip_radius_mm and loc.recommend_flip(
            target_dir, x, y, controller.flip
        ):
            controller.flip = not controller.flip

    target_heading = (target_dir + 180.0) if controller.flip else target_dir
    err = ((target_heading - last_yaw_deg + 180.0) % 360.0) - 180.0

    speed_factor = math.exp(-(err / controller.align_tol_deg) ** 2)
    signed_speed = -max_speed if controller.flip else max_speed
    speed = signed_speed * speed_factor

    resp = await robot.update(target_speed=speed, target_heading=target_heading)
    dt_s = max(1e-3, (resp.current_us - last_t_us) * 1e-6)
    filter_step(loc, resp, speed, dt_s)
    return resp, resp.current_us
