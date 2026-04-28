"""2D Bayes localization conditioned on a trusted gyro.

Old localization.py was a 3D grid (x, y, theta) because the heading
had to be inferred from a 360-deg scan. With the IMU treated as
ground truth, theta is known at update time, so the belief lives on
the 2D (x, y) plane and each ToF reading is a single index lookup
into a per-cell ray cache.

This file owns:
- The world line segments
- The RayCache (precomputed dense rays + circular-Gaussian smoothed
  mean and std distance per angular bin)
- The 2D belief, predict (motion model), and update (sensor) steps

It does NOT own the controller or the visualization; those belong to
the caller (demo_drive.py, a notebook, etc.).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ray_cache import RayCache


@dataclass
class LocParams:
    # Spatial grid: number of cells in each axis. Cell size is derived
    # from the world bounds at construction time.
    n_cells_x: int
    n_cells_y: int
    # Per-cell ray cache resolution
    n_angles: int
    smooth_sigma_deg: float
    max_dist_m: float
    # Sensor model (added in quadrature with the per-angle std from
    # the cache, so corner-buckets are automatically de-weighted)
    sensor_sigma_mm: float
    # Beyond this range the on-board sensor saturates (long-distance
    # mode, ~2 m for the VL53L1X). The update step clamps both the
    # measurement and the model expectation to this value; the
    # corresponding (mean - sat)**2 variance inflation is baked into
    # `cache.std_mm` at build time.
    tof_sat_mm: float
    # Robust likelihood: cap |residual / std| at this value before
    # squaring. Cells that disagree with the measurement by more than
    # `robust_z_max` standard deviations get a fixed penalty instead
    # of an exponentially crushing one, so a single anomalous reading
    # can't kill the right cell.
    robust_z_max: float
    # Motion model: longitudinal sigma combines a base term, a
    # per-meter term, and a per-(pwm * dt) term in quadrature; lateral
    # sigma is just the base (gyro is trusted, so no growth with
    # distance). The per-pwm term captures uncertainty in the
    # pwm_to_vel_mm_s mapping itself: even at fixed commanded distance,
    # the actual speed is unknown, so spread scales with |pwm| * dt.
    motion_long_sigma_per_m: float
    motion_long_sigma_per_pwm: float
    motion_base_sigma_m: float
    # PWM -> mm/s. Calibrate to the step response.
    pwm_to_vel_mm_s: float
    # Body-frame angles of the ToF sensors, used by recommend_flip to
    # compare cache stds at the forward vs reversed sensor bearings.
    tof_sensor_angles_deg: tuple[float, ...]
    # Flip recommended when the reversed-direction mean sensor std is
    # below this fraction of the forward mean std.
    flip_std_ratio: float


class Localization2D:
    """2D Bayes filter on a (x, y) grid."""

    def __init__(
        self,
        world_lines,
        bounds: tuple[float, float, float, float],  # (min_x, max_x, min_y, max_y)
        params: LocParams,
    ):
        self.params = params
        p = self.params

        min_x, max_x, min_y, max_y = bounds
        nx, ny = p.n_cells_x, p.n_cells_y
        self.dx = (max_x - min_x) / nx
        self.dy = (max_y - min_y) / ny
        # Cell centers
        self.xs = min_x + (np.arange(nx) + 0.5) * self.dx
        self.ys = min_y + (np.arange(ny) + 0.5) * self.dy
        self.bounds = bounds
        self.world_lines = world_lines

        self.cache = RayCache(
            world_lines,
            self.xs,
            self.ys,
            n_angles=p.n_angles,
            smooth_sigma_deg=p.smooth_sigma_deg,
            max_dist_m=p.max_dist_m,
            sensor_sigma_mm=p.sensor_sigma_mm,
            tof_sat_mm=p.tof_sat_mm,
        )

        self.valid_mask = self._build_valid_mask()
        self.bel = self.valid_mask / self.valid_mask.sum()

    # ---- validity mask ----

    def _build_valid_mask(self) -> np.ndarray:
        """Mark cells inside the playable area (not inside walls/obstacles).

        Uses an even-odd horizontal raycast against every wall segment.
        Closed loops nest correctly: outer-room cells get one crossing
        (odd -> valid), cells inside an inner obstacle get two crossings
        (even -> invalid), and cells outside the room (e.g. in the L
        notch) get an even count (-> invalid).
        """
        X, Y = np.meshgrid(self.xs, self.ys, indexing="ij")
        inside = np.zeros(X.shape, dtype=bool)
        for (x0, y0), (x1, y1) in self.world_lines:
            if y0 == y1:
                continue
            cond_y = (y0 > Y) != (y1 > Y)
            x_cross = x0 + (Y - y0) * (x1 - x0) / (y1 - y0)
            inside ^= cond_y & (X < x_cross)
        return inside.astype(np.float64)

    def _apply_mask(self):
        self.bel *= self.valid_mask
        s = self.bel.sum()
        if s > 0:
            self.bel /= s

    # ---- belief initialization ----

    def init_uniform(self):
        self.bel = self.valid_mask / self.valid_mask.sum()

    def init_at(self, x: float, y: float, sigma_m: float = 0.10):
        X, Y = np.meshgrid(self.xs, self.ys, indexing="ij")
        b = np.exp(-0.5 * (((X - x) ** 2 + (Y - y) ** 2) / sigma_m**2))
        b *= self.valid_mask
        self.bel = b / b.sum()

    # ---- predict (motion model) ----

    def predict(self, heading_deg: float, speed_pwm: float, dt_s: float):
        """Collapse posterior to a single Gaussian and translate it.

        Moment-matching predict. The robot is only in one place, so
        the prior is intentionally unimodal: take the posterior's
        mean and full 2x2 covariance, add the motion's mean
        displacement and its (rotated) covariance, and rebuild a
        single Gaussian with those moments. Multimodality in the
        posterior is averaged away on purpose.

        Inputs:
            heading_deg : measured world-frame yaw
            speed_pwm   : commanded forward PWM (mapped to mm/s via
                          LocParams.pwm_to_vel_mm_s)
            dt_s        : seconds since the last filter step
        """
        if not (math.isfinite(speed_pwm) and math.isfinite(dt_s)):
            return

        mu_post, cov_post = self.mean_cov_xy()

        v_m_s = (speed_pwm * self.params.pwm_to_vel_mm_s) / 1000.0
        yaw_rad = math.radians(heading_deg)
        c, s = math.cos(yaw_rad), math.sin(yaw_rad)
        d = v_m_s * dt_s
        mu_motion = np.array([d * c, d * s])

        dist_m = abs(d)
        pwm_dt = abs(speed_pwm) * dt_s
        base_var = self.params.motion_base_sigma_m**2
        sigma_long = math.sqrt(
            base_var
            + (self.params.motion_long_sigma_per_m * dist_m) ** 2
            + (self.params.motion_long_sigma_per_pwm * pwm_dt) ** 2
        )
        sigma_lat = self.params.motion_base_sigma_m
        # Heading-frame diagonal cov rotated into the world frame:
        # Sigma = R diag(s_long^2, s_lat^2) R^T
        R = np.array([[c, -s], [s, c]])
        cov_motion = R @ np.diag([sigma_long**2, sigma_lat**2]) @ R.T

        mu_new = mu_post + mu_motion
        cov_new = 0.5 * (cov_post + cov_motion)

        self._set_gaussian(mu_new, cov_new)

    def _set_gaussian(self, mu: np.ndarray, cov: np.ndarray):
        """Replace belief with a single 2D Gaussian on the grid."""
        a, b = float(cov[0, 0]), float(cov[0, 1])
        d = float(cov[1, 1])
        det = max(a * d - b * b, 1e-18)
        inv00, inv11 = d / det, a / det
        inv01 = -b / det

        X, Y = np.meshgrid(self.xs, self.ys, indexing="ij")
        dx = X - float(mu[0])
        dy = Y - float(mu[1])
        log_p = -0.5 * (inv00 * dx * dx + 2.0 * inv01 * dx * dy + inv11 * dy * dy)
        log_p -= log_p.max()
        bel = np.exp(log_p) * self.valid_mask
        total = bel.sum()
        if total > 0:
            self.bel = bel / total

    # ---- update (sensor model) ----

    def update(self, world_bearing_deg: float, measured_mm: float):
        """Multiply belief by p(z | cell) for one ToF reading.

        The per-cell std baked into the cache already grows by
        (mean - sat)**2 for far cells, so the standardized residual
        stays bounded for saturated readings without any clamping.
        Huber-style cap on |residual / std| at `robust_z_max` keeps
        single bad readings from killing the right cell.
        """
        mean_mm_raw, std_mm_raw = self.cache.lookup(world_bearing_deg)
        mean_mm = mean_mm_raw.astype(np.float64)
        std_mm = std_mm_raw.astype(np.float64)

        z = (measured_mm - mean_mm) / std_mm
        z = np.clip(z, -self.params.robust_z_max, self.params.robust_z_max)
        log_lik = -0.5 * z**2
        log_lik -= log_lik.max()
        b = self.bel * np.exp(log_lik) * self.valid_mask
        s = b.sum()
        if s > 0:
            self.bel = b / s

    # ---- accessors ----

    def argmax_xy(self) -> tuple[float, float]:
        ix, iy = np.unravel_index(np.argmax(self.bel), self.bel.shape)
        return float(self.xs[ix]), float(self.ys[iy])

    def mean_xy(self) -> tuple[float, float]:
        """Belief-weighted (x, y); useful when the posterior is unimodal."""
        wx = (self.bel.sum(axis=1) * self.xs).sum()
        wy = (self.bel.sum(axis=0) * self.ys).sum()
        return float(wx), float(wy)

    def std_xy(self) -> tuple[float, float]:
        """Belief-weighted per-axis standard deviation, in meters."""
        _, cov = self.mean_cov_xy()
        return math.sqrt(max(cov[0, 0], 0.0)), math.sqrt(max(cov[1, 1], 0.0))

    def mean_cov_xy(self) -> tuple[np.ndarray, np.ndarray]:
        """Belief-weighted mean and full 2x2 covariance of (x, y)."""
        px = self.bel.sum(axis=1)
        py = self.bel.sum(axis=0)
        cx = float((self.xs * px).sum())
        cy = float((self.ys * py).sum())
        dx = self.xs - cx
        dy = self.ys - cy
        var_x = float((dx * dx * px).sum())
        var_y = float((dy * dy * py).sum())
        # Off-diagonal: E[(x-cx)(y-cy)] over the joint belief.
        cov_xy = float((self.bel * dx[:, None] * dy[None, :]).sum())
        mu = np.array([cx, cy])
        cov = np.array([[var_x, cov_xy], [cov_xy, var_y]])
        return mu, cov

    def recommend_flip(
        self,
        target_dir_deg: float,
        x: float,
        y: float,
        currently_flipped: bool,
    ) -> bool:
        """True if the controller should toggle its flip state.

        Always evaluates the ToF cache stds at the heading pointing
        directly at the goal (`target_dir_deg`) and at its reverse,
        independent of the robot's instantaneous yaw. `flip_std_ratio`
        acts as hysteresis: the decision only switches when the other
        direction's RMS std is below `flip_std_ratio` of the current
        direction's. Using the goal-pointing heading as the reference
        keeps the recommendation stable while the body is turning.
        """
        nx = len(self.xs)
        ny = len(self.ys)
        ix = int(np.clip(round((x - self.xs[0]) / self.dx), 0, nx - 1))
        iy = int(np.clip(round((y - self.ys[0]) / self.dy), 0, ny - 1))

        sensor_angles = np.asarray(self.params.tof_sensor_angles_deg, dtype=float)
        fwd = target_dir_deg + sensor_angles
        rev = target_dir_deg + 180.0 + sensor_angles
        bearings = np.concatenate([fwd, rev])
        _, stds = self.cache.lookup_many(bearings)
        cell_stds = stds[ix, iy].astype(np.float64)
        s = sensor_angles.size
        fwd_rms = math.sqrt(float(np.mean(cell_stds[:s] ** 2)))
        rev_rms = math.sqrt(float(np.mean(cell_stds[s:] ** 2)))
        ratio = self.params.flip_std_ratio
        if currently_flipped:
            return fwd_rms < ratio * rev_rms
        return rev_rms < ratio * fwd_rms

    @property
    def extent(self) -> tuple[float, float, float, float]:
        """imshow extent for self.bel.T with origin='lower'."""
        return (
            float(self.xs[0] - self.dx / 2),
            float(self.xs[-1] + self.dx / 2),
            float(self.ys[0] - self.dy / 2),
            float(self.ys[-1] + self.dy / 2),
        )
