"""Lightweight simulated robot mirroring the drive_update BLE command.

The interface matches subsystem_drive.h / DriveUpdate: each call takes a
target PWM and target heading in degrees, advances simulated time by
one BLE round-trip, and returns the latest cached ToF + yaw. Use this
to develop and test the motion-model predict/update Bayes filter
offline, without having to drive the physical robot.

Timing model
------------
- physics_dt_s   : internal integration step (e.g. 5 ms)
- tof_rate_hz    : how fast the on-board sensor pushes new samples
                   into the circular buffer (~10-20 Hz like the real
                   VL53L1X long-range mode)
- ble_rtt_s      : one update() call advances sim time by this much,
                   with optional jitter, simulating BLE round-trip
                   latency (~10 commands/sec)
The response contains the *most recent* ToF sample timestamped at
when it was taken, exactly like subsystem_drive.h reads from the ToF
circular buffer.

Example
-------
    sim = SimRobot(world_lines, x0=0.0, y0=0.0, theta_deg=0.0)
    while True:
        resp = sim.update(target_speed=80.0, target_heading=0.0)
        # resp.current_us  - sim time now
        # resp.tof1_time_us - when the returned reading was actually taken
        # resp.tof1_dist_mm, resp.tof2_dist_mm, resp.yaw_deg
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin, radians, degrees, pi
from typing import NamedTuple

import numpy as np


class DriveUpdateResponse(NamedTuple):
    """Same shape as ble_robot_1.4/main_python/commands/drive.py.

    `tof{N}_yaw_deg` is the (noisy) yaw at the moment that ToF sample
    was actually taken; `yaw_deg` is the yaw at command receipt.
    """

    current_us: int
    tof1_time_us: int
    tof1_dist_mm: int
    tof1_yaw_deg: float
    tof2_time_us: int
    tof2_dist_mm: int
    tof2_yaw_deg: float
    yaw_deg: float


def _wrap_deg(a: float) -> float:
    return a - 360.0 * round(a / 360.0)


@dataclass
class SimParams:
    # Drivetrain
    pwm_to_vel_mm_s: float
    vel_tau_s: float            # first-order spool-up time
    max_vel_mm_s: float         # saturation
    # Heading (modeling the on-board angle PID)
    heading_tau_s: float        # exponential approach
    yaw_rate_max_dps: float     # slew limit, deg/s
    # Process noise
    vel_sigma_mm_s: float       # per-step noise on velocity
    # Heading bias: Ornstein-Uhlenbeck process with steady-state sigma
    # `heading_bias_sigma_deg` and time constant `heading_bias_tau_s`.
    # Models a slow gyro offset that drifts on the minute scale rather
    # than a pure random walk that diverges without bound.
    heading_bias_sigma_deg: float
    heading_bias_tau_s: float
    heading_step_sigma_deg: float  # per-step heading jitter
    # ToF sensor model
    tof_sigma_mm: float
    tof_max_mm: float           # readings saturate at this value
    tof1_angle_deg: float       # mount angle off body heading
    tof2_angle_deg: float       # mount angle off body heading
    tof_rate_hz: float          # on-board sample rate
    tof_jitter_frac: float      # +/- fraction of period
    # Timing
    physics_dt_s: float         # internal integration step
    ble_rtt_s: float            # one update() == one BLE rtt
    ble_rtt_jitter_s: float     # gaussian sigma on rtt
    rng_seed: int | None = None


class SimRobot:
    """Pose simulator with PWM/heading drive, ToF, and BLE latency.

    Internal state is updated in fine physics steps; user-facing
    update() advances by one BLE round-trip and returns the most
    recent ToF sample from a buffer that fills at tof_rate_hz.
    """

    def __init__(
        self,
        world_lines,
        params: SimParams,
        x0: float = 0.0,
        y0: float = 0.0,
        theta_deg: float = 0.0,
    ):
        self.params = params
        self.rng = np.random.default_rng(self.params.rng_seed)

        # Pack world lines for vectorized raycasts.
        p = np.array([np.asarray(seg[0], dtype=float) for seg in world_lines])
        q = np.array([np.asarray(seg[1], dtype=float) for seg in world_lines])
        self._line_p = p
        self._line_d = q - p

        # Ground-truth pose
        self.x = float(x0)
        self.y = float(y0)
        self.theta = radians(theta_deg)
        self.v_mm_s = 0.0
        self.heading_bias_deg = float(
            self.rng.normal(0.0, self.params.heading_bias_sigma_deg)
        )

        # Sim clock (microseconds of simulated time)
        self.t_us: int = 0

        # Latest ToF sample held by the on-board buffer.
        self._latest_tof_t_us: int = 0
        self._latest_tof1_mm: int = 0
        self._latest_tof2_mm: int = 0
        self._latest_tof1_yaw_deg: float = 0.0
        self._latest_tof2_yaw_deg: float = 0.0
        self._next_tof_us: int = self._schedule_next_tof(0)

        # Take an initial reading so the first update() returns something
        # plausible even if BLE rtt < tof period.
        self._sample_tof()

    # ----- public API -----

    @property
    def pose(self) -> tuple[float, float, float]:
        """Ground truth (x_m, y_m, theta_deg)."""
        return self.x, self.y, _wrap_deg(degrees(self.theta))

    @property
    def t_s(self) -> float:
        """Simulated wall-clock seconds since construction."""
        return self.t_us * 1e-6

    async def update(self, target_speed: float, target_heading: float) -> DriveUpdateResponse:
        """Advance one BLE round-trip and return the latest cached ToF.

        Async to match `RealRobot.update`; the body itself is purely
        synchronous CPU work.
        """
        p = self.params

        rtt = max(
            p.physics_dt_s,
            p.ble_rtt_s + self.rng.normal(0.0, p.ble_rtt_jitter_s),
        )
        end_us = self.t_us + int(round(rtt * 1e6))

        # Integrate physics in fine steps; sample ToF at scheduled ticks.
        while self.t_us < end_us:
            step_end = min(end_us, self._next_tof_us)
            dt = (step_end - self.t_us) * 1e-6
            if dt > 0:
                self._integrate(dt, target_speed, target_heading)
                self.t_us = step_end
            if self.t_us >= self._next_tof_us:
                self._sample_tof()
                self._next_tof_us = self._schedule_next_tof(self.t_us)

        yaw_meas = _wrap_deg(degrees(self.theta) + self.heading_bias_deg)
        return DriveUpdateResponse(
            current_us=self.t_us,
            tof1_time_us=self._latest_tof_t_us,
            tof1_dist_mm=self._latest_tof1_mm,
            tof1_yaw_deg=float(self._latest_tof1_yaw_deg),
            tof2_time_us=self._latest_tof_t_us,
            tof2_dist_mm=self._latest_tof2_mm,
            tof2_yaw_deg=float(self._latest_tof2_yaw_deg),
            yaw_deg=float(yaw_meas),
        )

    # ----- internals -----

    def _integrate(self, dt: float, target_speed: float, target_heading: float):
        """Advance ground-truth state by dt seconds."""
        p = self.params

        # Velocity: first-order lag toward PWM-mapped target + noise.
        v_target = float(target_speed) * p.pwm_to_vel_mm_s
        v_target = float(np.clip(v_target, -p.max_vel_mm_s, p.max_vel_mm_s))
        alpha_v = 1.0 - np.exp(-dt / p.vel_tau_s)
        self.v_mm_s += (v_target - self.v_mm_s) * alpha_v
        self.v_mm_s += self.rng.normal(0.0, p.vel_sigma_mm_s) * np.sqrt(dt)

        # Heading: exponential approach with rate cap, plus jitter and bias.
        target = radians(_wrap_deg(float(target_heading)))
        err = (target - self.theta + pi) % (2 * pi) - pi
        alpha_h = 1.0 - np.exp(-dt / p.heading_tau_s)
        d_theta = err * alpha_h
        max_step = radians(p.yaw_rate_max_dps) * dt
        d_theta = float(np.clip(d_theta, -max_step, max_step))
        self.theta += d_theta
        # OU step: b <- b*exp(-dt/tau) + sigma_ss*sqrt(1-exp(-2*dt/tau))*N(0,1)
        decay = np.exp(-dt / p.heading_bias_tau_s)
        self.heading_bias_deg = (
            self.heading_bias_deg * decay
            + self.rng.normal(0.0, p.heading_bias_sigma_deg * np.sqrt(1.0 - decay**2))
        )
        self.theta += radians(
            self.rng.normal(0.0, p.heading_step_sigma_deg) * np.sqrt(dt / 0.1)
        )

        # Position
        v_m_s = self.v_mm_s / 1000.0
        self.x += v_m_s * cos(self.theta) * dt
        self.y += v_m_s * sin(self.theta) * dt

    def _schedule_next_tof(self, now_us: int) -> int:
        p = self.params
        period_s = 1.0 / p.tof_rate_hz
        jitter = self.rng.uniform(-p.tof_jitter_frac, p.tof_jitter_frac)
        return now_us + int(round(period_s * (1.0 + jitter) * 1e6))

    def _sample_tof(self):
        """Take a fresh ToF reading and stash it as the latest cached value."""
        p = self.params
        d1 = self._raycast_mm(p.tof1_angle_deg)
        d2 = self._raycast_mm(p.tof2_angle_deg)
        yaw_now = _wrap_deg(degrees(self.theta) + self.heading_bias_deg)
        self._latest_tof1_mm = d1
        self._latest_tof2_mm = d2
        self._latest_tof1_yaw_deg = yaw_now
        # Firmware reports tof2 yaw as tof1_yaw - 90 (sensor 2 is mounted
        # 90 deg clockwise of sensor 1); mirror that here.
        self._latest_tof2_yaw_deg = _wrap_deg(yaw_now + p.tof2_angle_deg)
        self._latest_tof_t_us = self.t_us

    def _raycast_mm(self, sensor_angle_deg: float) -> int:
        """Distance in mm to the nearest wall along (theta + sensor_angle)."""
        bearing = self.theta + radians(sensor_angle_deg)
        rd = np.array([cos(bearing), sin(bearing)])
        rs = np.array([self.x, self.y])

        ld = self._line_d  # (L, 2)
        diff = self._line_p - rs  # (L, 2)

        denom = rd[0] * ld[:, 1] - rd[1] * ld[:, 0]  # (L,)
        t_num = diff[:, 0] * ld[:, 1] - diff[:, 1] * ld[:, 0]
        u_num = diff[:, 0] * rd[1] - diff[:, 1] * rd[0]

        with np.errstate(divide="ignore", invalid="ignore"):
            t = t_num / denom  # along ray, meters
            u = u_num / denom  # along segment, [0,1]

        valid = np.isfinite(t) & np.isfinite(u) & (t >= 0) & (u >= 0) & (u <= 1)
        if not valid.any():
            d_m = self.params.tof_max_mm / 1000.0
        else:
            d_m = float(np.min(t[valid]))

        d_mm = d_m * 1000.0 + self.rng.normal(0.0, self.params.tof_sigma_mm)
        return int(np.clip(d_mm, 0.0, self.params.tof_max_mm))
