"""Waypoint controller and a small plotting helper.

The controller is intentionally dumb: from a given (x, y), point at
the next waypoint and drive at a fixed PWM until inside a stop
radius, then advance. Use it open-loop on ground-truth or closed-loop
on the belief argmax by passing the relevant pose into `command()`.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np

from localization2d import Localization2D
from sim_robot import DriveUpdateResponse


# ----------------------------- controller ---------------------------

@dataclass
class WaypointController:
    """Sequentially aims at each waypoint; advances inside stop radius.

    All knobs are required and must come from `config/world.yaml` via
    `world.load_world()`; there are no in-class defaults so a missing
    YAML key surfaces immediately instead of silently falling back.
    """

    path: list[tuple[float, float]]
    stop_radius_mm: float
    stop_flip_radius_mm: float
    max_speed_pwm: float
    min_speed_pwm: float
    max_speed_distance_mm: float
    align_tol_deg: float
    position_rc_s: float
    tof_long_threshold_mm: float
    tof_short_threshold_mm: float

    def __post_init__(self):
        # Runtime state, not configuration: starts fresh on construction.
        self.idx: int = 1               # path[0] is the start pose
        self.flip: bool = False         # True = drive backwards toward the waypoint
        self.log: list[int] = []
        # ToF distance-mode hysteresis state, seeded to LONG so the first
        # DriveUpdate pushes the safer (longer-range) mode.
        self.long_mode_1: bool = True
        self.long_mode_2: bool = True
        self.filt_x: float | None = None
        self.filt_y: float | None = None
        self._last_filt_t: float = 0.0

    def filtered(self, x: float, y: float) -> tuple[float, float]:
        """Exponential low-pass on the planning position, RC = position_rc_s.

        Uses wall-clock dt between successive calls, so it works the
        same against the simulator and the real robot. Disable by
        setting position_rc_s = 0.
        """
        if self.position_rc_s <= 0.0 or self.filt_x is None:
            self.filt_x, self.filt_y = x, y
            self._last_filt_t = time.monotonic()
            return x, y
        now = time.monotonic()
        dt = max(1e-3, now - self._last_filt_t)
        self._last_filt_t = now
        alpha = 1.0 - math.exp(-dt / self.position_rc_s)
        fx: float = self.filt_x  # type: ignore[assignment]
        fy: float = self.filt_y  # type: ignore[assignment]
        fx += (x - fx) * alpha
        fy += (y - fy) * alpha
        self.filt_x, self.filt_y = fx, fy
        return fx, fy

    @property
    def done(self) -> bool:
        return self.idx >= len(self.path)

    @property
    def current_waypoint(self) -> tuple[float, float] | None:
        return None if self.done else self.path[self.idx]

    def command(self, x: float, y: float) -> tuple[float, float]:
        """Return (speed_pwm, target_heading_deg) for the next BLE call.

        Advances the waypoint index automatically when within the stop
        radius. Speed ramps linearly with distance: `min_speed_pwm` at
        `stop_radius_mm`, `max_speed_pwm` at or beyond
        `max_speed_distance_mm`. Returns (0, 0) once the path is finished.
        """
        while not self.done:
            wx, wy = self.path[self.idx]
            dx, dy = wx - x, wy - y
            dist_mm = math.hypot(dx, dy) * 1000.0
            if dist_mm < self.stop_radius_mm:
                self.log.append(self.idx)
                self.idx += 1
                continue
            span = max(1e-6, self.max_speed_distance_mm - self.stop_radius_mm)
            t = (dist_mm - self.stop_radius_mm) / span
            t = max(0.0, min(1.0, t))
            speed = self.min_speed_pwm + t * (self.max_speed_pwm - self.min_speed_pwm)
            return speed, math.degrees(math.atan2(dy, dx))
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
        self.filt_pt, = self.ax.plot([], [], "y+", ms=14, mew=3, label="planner pos (filtered)")
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
        filtered_xy: tuple[float, float] | None = None,
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

        if filtered_xy is not None:
            self.filt_pt.set_data([filtered_xy[0]], [filtered_xy[1]])

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
):
    """One Bayes filter cycle from a DriveUpdate response.

    Predict always runs from the robot-side integrated displacement.
    The sensor update only fires for ToF samples whose timestamp has
    advanced since the previous filter step; otherwise the firmware
    is just re-handing us the same cached reading and folding it in
    twice would over-sharpen the belief.
    """
    loc.predict_displacement(resp.dx_pwm_s, resp.dy_pwm_s, resp.abs_pwm_s)
    if (
        resp.tof1_dist_mm > 0
        and math.isfinite(resp.tof1_yaw_deg)
        and resp.tof1_time_us != loc.last_tof1_time_us
    ):
        loc.update(resp.tof1_yaw_deg, float(resp.tof1_dist_mm))
        loc.last_tof1_time_us = resp.tof1_time_us
    if (
        resp.tof2_dist_mm > 0
        and math.isfinite(resp.tof2_yaw_deg)
        and resp.tof2_time_us != loc.last_tof2_time_us
    ):
        loc.update(resp.tof2_yaw_deg, float(resp.tof2_dist_mm))
        loc.last_tof2_time_us = resp.tof2_time_us


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
        raw_x, raw_y = loc.mean_xy()
    else:
        gx, gy, _ = robot.pose
        raw_x, raw_y = gx, gy

    # Low-pass the planning position so noisy belief jitter does not
    # induce stuttery target bearings or flip flicker. Visualization and
    # the localization filter still operate on the raw mean.
    x, y = controller.filtered(raw_x, raw_y)

    max_speed, target_dir = controller.command(x, y)
    wp = controller.current_waypoint
    if wp is not None:
        dist_mm = math.hypot(wp[0] - x, wp[1] - y) * 1000.0
        if dist_mm > controller.stop_flip_radius_mm and loc.recommend_flip(
            target_dir, x, y, controller.flip
        ):
            controller.flip = not controller.flip

    target_heading = (target_dir + 180.0) if controller.flip else target_dir
    signed_speed = -max_speed if controller.flip else max_speed

    # ToF mode hysteresis: pick LONG when the predicted distance along
    # each sensor's bearing crosses the high threshold, drop to SHORT
    # only when it falls below the low threshold. Sensor 2 sits 90 deg
    # clockwise of sensor 1, matching subsystem_drive::update_cmd.
    bearing1 = last_yaw_deg
    bearing2 = last_yaw_deg - 90.0
    exp1 = loc.expected_distance_mm(bearing1)
    exp2 = loc.expected_distance_mm(bearing2)
    if controller.long_mode_1 and exp1 < controller.tof_short_threshold_mm:
        controller.long_mode_1 = False
    elif not controller.long_mode_1 and exp1 > controller.tof_long_threshold_mm:
        controller.long_mode_1 = True
    if controller.long_mode_2 and exp2 < controller.tof_short_threshold_mm:
        controller.long_mode_2 = False
    elif not controller.long_mode_2 and exp2 > controller.tof_long_threshold_mm:
        controller.long_mode_2 = True

    # Heading-error Gaussian weighting now lives on the robot
    # (subsystem_drive::ALIGN_TOL_DEG) so it tracks live yaw with no BLE
    # round-trip latency. Send the unweighted signed speed.
    resp = await robot.update(
        target_speed=signed_speed,
        target_heading=target_heading,
        long_mode_1=controller.long_mode_1,
        long_mode_2=controller.long_mode_2,
    )
    # Predict from the robot-side integrated PWM*s displacement; no
    # speed_factor or dt approximation needed on the host.
    filter_step(loc, resp)
    return resp, resp.current_us
