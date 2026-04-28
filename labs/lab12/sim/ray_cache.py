"""Per-cell ray cache for the motion-model Bayes filter.

For every (x, y) cell in a 2D grid we cast a dense set of rays (default
720 rays = 0.5 deg resolution) against the world line segments, then
smooth the resulting distance trace over the angular axis with a
circular Gaussian kernel. We do the same smoothing on the squared
distance to recover a per-angle standard deviation. Discontinuities
(corners, the leading edge of an obstacle) produce buckets where
nearby rays disagree -> high std, which the Bayes update can use to
de-weight unreliable bearings.

Because the gyro is trusted, the robot's heading is known at update
time. For each ToF reading we know the world bearing of the ray, so
the update is a single index lookup per cell rather than a 360-deg
scan correlation.

Usage
-----
    cache = RayCache(world_lines, x_centers, y_centers,
                     n_angles=720, smooth_sigma_deg=5.0)
    # For one ToF reading at world bearing 37.2 deg:
    means, stds = cache.lookup(37.2)            # shape (NX, NY)
    # For both ToF sensors at once:
    means, stds = cache.lookup_many([37.2, 37.2 + 90.0])  # (NX, NY, 2)
"""

from __future__ import annotations

import numpy as np


def _circular_gaussian_smooth(arr: np.ndarray, sigma_bins: float) -> np.ndarray:
    """Convolve along the last axis with a wrap-around Gaussian kernel."""
    n = arr.shape[-1]
    k = np.arange(n)
    k = np.minimum(k, n - k)                       # circular distance from 0
    kernel = np.exp(-0.5 * (k / sigma_bins) ** 2)
    kernel /= kernel.sum()
    # Multiply in the frequency domain; FFT handles the circular wrap.
    return np.fft.irfft(
        np.fft.rfft(arr, axis=-1) * np.fft.rfft(kernel),
        n=n,
        axis=-1,
    )


class RayCache:
    """Pre-computed per-cell ray distances + per-angle uncertainty."""

    def __init__(
        self,
        world_lines,
        x_centers: np.ndarray,
        y_centers: np.ndarray,
        *,
        n_angles: int = 720,
        smooth_sigma_deg: float = 5.0,
        max_dist_m: float = 6.0,
        sensor_sigma_mm: float = 0.0,
        tof_sat_mm: float | None = None,
    ):
        """Pre-build per-cell ray distances and per-angle uncertainty.

        sensor_sigma_mm and tof_sat_mm are baked into `std_mm` at build
        time, so the Bayes update is just (mean, std) lookup -> Gaussian.
        Pass `tof_sat_mm=None` to skip the saturation inflation.
        """
        self.x_centers = np.asarray(x_centers, dtype=float)
        self.y_centers = np.asarray(y_centers, dtype=float)
        self.n_angles = int(n_angles)
        self.angle_step_deg = 360.0 / self.n_angles
        self.smooth_sigma_deg = float(smooth_sigma_deg)
        self.max_dist_m = float(max_dist_m)
        self.sensor_sigma_mm = float(sensor_sigma_mm)
        self.tof_sat_mm = None if tof_sat_mm is None else float(tof_sat_mm)

        # Pack walls (L, 2) start + (L, 2) edge vector.
        p = np.array([np.asarray(seg[0], dtype=float) for seg in world_lines])
        q = np.array([np.asarray(seg[1], dtype=float) for seg in world_lines])
        self._line_p = p
        self._line_d = q - p

        # Build the cache. Distances are stored in mm to match the wire
        # protocol used by the rest of the localization pipeline.
        self.mean_mm, self.std_mm = self._build()

    # --- public lookup API ---

    def lookup(self, world_bearing_deg: float) -> tuple[np.ndarray, np.ndarray]:
        """Return (mean_mm, std_mm) per cell at one world bearing."""
        idx = self._bearing_to_bin(world_bearing_deg)
        return self.mean_mm[:, :, idx], self.std_mm[:, :, idx]

    def lookup_many(self, world_bearings_deg) -> tuple[np.ndarray, np.ndarray]:
        """Return (mean_mm, std_mm) shaped (NX, NY, K) for K bearings."""
        bearings = np.atleast_1d(np.asarray(world_bearings_deg, dtype=float))
        idxs = ((bearings / self.angle_step_deg).round().astype(int)
                % self.n_angles)
        return self.mean_mm[:, :, idxs], self.std_mm[:, :, idxs]

    # --- internals ---

    def _bearing_to_bin(self, world_bearing_deg: float) -> int:
        return int(round(world_bearing_deg / self.angle_step_deg)) % self.n_angles

    def _build(self) -> tuple[np.ndarray, np.ndarray]:
        nx = self.x_centers.size
        ny = self.y_centers.size
        na = self.n_angles

        # Ray directions (na, 2), shared across all cells (world frame).
        bearings_rad = np.radians(np.arange(na) * self.angle_step_deg)
        rd = np.stack([np.cos(bearings_rad), np.sin(bearings_rad)], axis=1)

        ld = self._line_d                     # (L, 2)
        lp = self._line_p                     # (L, 2)

        # Cross products: a x b = a[0]*b[1] - a[1]*b[0]
        # rd x ld -> (na, L)
        denom = rd[:, 0:1] * ld[None, :, 1] - rd[:, 1:2] * ld[None, :, 0]

        # Allocate output: distance in METERS first, convert to mm at end.
        raw_dist = np.empty((nx, ny, na), dtype=np.float64)
        max_d_m = self.max_dist_m

        for ix, x in enumerate(self.x_centers):
            for iy, y in enumerate(self.y_centers):
                rs = np.array([x, y])
                diff = lp - rs                   # (L, 2)
                # (line_p - rs) x ld -> (L,)  (depends only on cell+wall)
                t_num = diff[:, 0] * ld[:, 1] - diff[:, 1] * ld[:, 0]
                # (line_p - rs) x rd -> (na, L)
                u_num = (
                    diff[None, :, 0] * rd[:, 1:2]
                    - diff[None, :, 1] * rd[:, 0:1]
                )

                with np.errstate(divide="ignore", invalid="ignore"):
                    t = t_num[None, :] / denom   # (na, L) ray-axis distance
                    u = u_num / denom            # (na, L) segment param

                bad = (
                    ~np.isfinite(t) | ~np.isfinite(u)
                    | (t < 0) | (u < 0) | (u > 1)
                )
                t = np.where(bad, np.inf, t)
                d = np.min(t, axis=1)            # (na,) nearest hit per ray
                d = np.minimum(d, max_d_m)
                raw_dist[ix, iy] = d

        # Convert to millimeters.
        raw_mm = raw_dist * 1000.0

        # Circular Gaussian smoothing along the angle axis.
        sigma_bins = self.smooth_sigma_deg / self.angle_step_deg
        mean_mm = _circular_gaussian_smooth(raw_mm, sigma_bins)
        mean_sq = _circular_gaussian_smooth(raw_mm ** 2, sigma_bins)
        smooth_var = np.maximum(mean_sq - mean_mm ** 2, 0.0)

        # Bake in sensor noise + saturation inflation so the update step
        # is a pure (mean, std) Gaussian lookup.
        total_var = smooth_var + self.sensor_sigma_mm ** 2
        if self.tof_sat_mm is not None:
            past_sat = np.maximum(mean_mm - self.tof_sat_mm, 0.0)
            total_var = total_var + past_sat ** 2
        std_mm = np.sqrt(total_var)

        return mean_mm.astype(np.float32), std_mm.astype(np.float32)
