"""Reusable test trajectories for the localization demo.

Points come from the lab handout in feet; the simulator/localization
work in meters, so the canonical export is `LAB_PATH_M` (already
converted). `LAB_PATH_FT` is kept for traceability.

Also provides `SplineCurve`, a cubic-spline interpolant of a waypoint
list reparameterized by arclength, used by `SplineController`.
"""

from __future__ import annotations

import numpy as np
from scipy.interpolate import CubicSpline, PchipInterpolator

FT_TO_M = 0.3048

LAB_PATH_FT: list[tuple[float, float]] = [
    (-4, -3),  # 1. start
    (-2, -1),  # 2.
    (1, -1),  # 3.
    (2, -3),  # 4.
    (5, -3),  # 5.
    (5, -2),  # 6.
    (5.5, -0.5),
    (5, 3),  # 7.
    (0, 3),  # 8.
    (0, 0),  # 9. end
]

LAB_PATH_M: list[tuple[float, float]] = [
    (x * FT_TO_M, y * FT_TO_M) for x, y in LAB_PATH_FT
]


# ----------------------------- spline -------------------------------


class SplineCurve:
    """Natural cubic spline through a waypoint list, arclength-parameterized.

    The raw spline is built on a **centripetal** parameter
    `t_i = sum_j ||p_j - p_{j-1}||^0.5`. Centripetal parameterization
    makes the underlying curve shape independent of waypoint density:
    closely-spaced waypoints no longer pull extra curvature into that
    region (vs. uniform-index `t = arange(N)`, which biases curvature
    by point density and tends to overshoot near widely-spaced
    points).

    Arclength `s` is then computed by densely sampling the raw spline
    and accumulating chord lengths; `s_to_t` is a monotone PCHIP
    interpolant from arclength `s` in [0, S] back to the centripetal
    `t`. All public lookups (`xy`, `tangent`, `dense_sample`) take and
    return arclength `s` in meters.
    """

    def __init__(self, waypoints, raw_samples: int = 10_000):
        pts = np.asarray(waypoints, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[1] != 2 or pts.shape[0] < 2:
            raise ValueError(f"need (N, 2) waypoints with N>=2, got {pts.shape}")
        self.waypoints = pts
        # Centripetal parameter: dt_i = ||p_i - p_{i-1}||^0.5. Avoids
        # density-biased curvature and overshoot.
        chord = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        if np.any(chord <= 0):
            raise ValueError("waypoints must be distinct (zero chord between two)")
        dt = np.sqrt(chord)
        t_wp = np.concatenate(([0.0], np.cumsum(dt)))
        self._raw = CubicSpline(t_wp, pts, bc_type="natural")

        ts = np.linspace(0.0, t_wp[-1], raw_samples)
        xy = self._raw(ts)
        seg = np.diff(xy, axis=0)
        chord = np.sqrt((seg * seg).sum(axis=1))
        s = np.concatenate(([0.0], np.cumsum(chord)))
        # Strip duplicates so PCHIP gets a strictly monotone x-axis.
        keep = np.concatenate(([True], np.diff(s) > 0))
        self._s_grid = s[keep]
        self._t_grid = ts[keep]
        self.S = float(self._s_grid[-1])
        self._s_to_t = PchipInterpolator(self._s_grid, self._t_grid)

    def _t_of_s(self, s):
        s = np.clip(np.asarray(s, dtype=np.float64), 0.0, self.S)
        return self._s_to_t(s)

    def xy(self, s) -> np.ndarray:
        """Position(s) on the spline, shape (..., 2)."""
        t = self._t_of_s(s)
        out = self._raw(t)
        return out

    def tangent(self, s) -> np.ndarray:
        """Unit tangent vector(s), shape (..., 2)."""
        t = self._t_of_s(s)
        v = self._raw(t, 1)
        n = np.linalg.norm(v, axis=-1, keepdims=True)
        n = np.where(n > 1e-12, n, 1.0)
        return v / n

    def dense_sample(self, step_m: float) -> tuple[np.ndarray, np.ndarray]:
        """Evenly-spaced samples in arclength: (s_samples, xy_samples)."""
        if step_m <= 0:
            raise ValueError(f"step_m must be > 0, got {step_m}")
        n_samples = max(2, int(np.ceil(self.S / step_m)) + 1)
        s_samples = np.linspace(0.0, self.S, n_samples)
        xy_samples = self.xy(s_samples)
        return s_samples, xy_samples

    def advance(self, s0: float, ds: float) -> float:
        return float(np.clip(s0 + ds, 0.0, self.S))


def build_spline(waypoints) -> SplineCurve:
    return SplineCurve(waypoints)


def project_to_spline(
    xy_query: np.ndarray, xy_samples: np.ndarray, s_samples: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Nearest dense-sample projection. xy_query: (..., 2)."""
    q = np.asarray(xy_query, dtype=np.float64)
    flat = q.reshape(-1, 2)
    d2 = ((flat[:, None, :] - xy_samples[None, :, :]) ** 2).sum(axis=2)
    k = np.argmin(d2, axis=1)
    s_near = s_samples[k]
    d_near = np.sqrt(d2[np.arange(flat.shape[0]), k])
    return s_near.reshape(q.shape[:-1]), d_near.reshape(q.shape[:-1])
