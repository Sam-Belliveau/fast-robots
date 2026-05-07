"""Waypoint controller and a small plotting helper.

The controller is intentionally dumb: from a given (x, y), point at
the next waypoint and drive at a fixed PWM until inside a stop
radius, then advance. Use it open-loop on ground-truth or closed-loop
on the belief argmax by passing the relevant pose into `command()`.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import matplotlib.pyplot as plt
import numpy as np

from localization2d import Localization2D
from paths import SplineCurve, build_spline
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
        self.idx: int = 1  # path[0] is the start pose
        self.flip: bool = False  # True = drive backwards toward the waypoint
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


# --------------------------- spline controller ----------------------


@dataclass
class ViolationReport:
    """A contiguous span of arclength where the spline grazes a wall."""

    s_start: float
    s_end: float
    min_distance_m: float
    wall_index: int
    suggested_midpoint_xy: tuple[float, float]


def _point_segment_distance(
    points: np.ndarray, seg_a: np.ndarray, seg_d: np.ndarray
) -> np.ndarray:
    """Distance from each point in `points` (P,2) to each segment (a, a+d). Returns (P, L)."""
    # ap = points[:,None,:] - seg_a[None,:,:]   shape (P, L, 2)
    ap = points[:, None, :] - seg_a[None, :, :]
    dd = (seg_d * seg_d).sum(axis=1)  # (L,)
    dd = np.where(dd > 1e-18, dd, 1.0)
    t = (ap * seg_d[None, :, :]).sum(axis=2) / dd[None, :]
    t = np.clip(t, 0.0, 1.0)
    closest = seg_a[None, :, :] + t[..., None] * seg_d[None, :, :]
    diff = points[:, None, :] - closest
    return np.sqrt((diff * diff).sum(axis=2))


@dataclass
class SplineController:
    """Belief-weighted heading-field pure-pursuit on a cubic spline.

    Each valid grid cell precomputes up to two unit heading vectors:
    for each local minimum of distance(cell, spline), the heading is a
    Gaussian blend of (a) the spline tangent at the nearest pass and
    (b) the unit vector from the cell toward that nearest spline point.
    Close to the path -> tangent dominates; far -> drive toward the
    path. The same Gaussian (sigma = `heading_sigma_m`) also weights
    the cell's contribution, so distant cells fade out smoothly.

    At each step, eligible (cell, pass) pairs (forward window AND
    cell-to-nearest-spline-point ray clear of walls) are summed as
    weighted unit vectors. `atan2` of the resulting vector is the
    commanded heading. `s_prog` advances with the weighted `pass_s`.
    """

    path: list[tuple[float, float]]
    loc: Localization2D
    map_lines: list
    stop_radius_mm: float
    stop_flip_radius_mm: float
    drive_pwm: float
    align_tol_deg: float
    position_rc_s: float
    tof_long_threshold_mm: float
    tof_short_threshold_mm: float
    look_ahead_m: float
    advance_window_m: float
    heading_sigma_m: float
    wall_clearance_m: float
    spline_sample_step_m: float
    s_prog_backoff_m: float
    heading_lookahead_m: float

    def __post_init__(self):
        # Runtime state
        self.flip: bool = False
        self.long_mode_1: bool = True
        self.long_mode_2: bool = True
        self.filt_x: float | None = None
        self.filt_y: float | None = None
        self._last_filt_t: float = 0.0
        self.s_prog: float = 0.0
        self._last_s_target: float = 0.0
        self._last_aim_xy: tuple[float, float] = (0.0, 0.0)
        self._finished: bool = False
        self.log: list[float] = []  # s_prog history

        self.spline: SplineCurve = build_spline(self.path)
        self.s_samples, self.xy_samples = self.spline.dense_sample(
            self.spline_sample_step_m
        )
        self._N = self.s_samples.shape[0]

        # Wall geometry as (line_p, line_d) arrays.
        if len(self.map_lines) == 0:
            self._seg_a = np.zeros((0, 2))
            self._seg_d = np.zeros((0, 2))
        else:
            a = np.array([p0 for p0, _ in self.map_lines], dtype=np.float64)
            b = np.array([p1 for _, p1 in self.map_lines], dtype=np.float64)
            self._seg_a = a
            self._seg_d = b - a

        # Wall-graze validation up-front.
        violations = self.validate_spline_vs_walls()
        if violations:
            msg = "spline violates wall_clearance_m={:.3f} on {} segment(s):\n".format(
                self.wall_clearance_m, len(violations)
            )
            for v in violations:
                msg += (
                    f"  s=[{v.s_start:.3f},{v.s_end:.3f}] m  min_dist={v.min_distance_m:.3f} m  "
                    f"wall={v.wall_index}  add waypoint near "
                    f"({v.suggested_midpoint_xy[0]:.3f}, {v.suggested_midpoint_xy[1]:.3f})\n"
                )
            raise RuntimeError(msg)

        # Per-cell heading field (s_pass, heading_vec, kernel, visible).
        (
            self._pass_s,
            self._heading_vec,
            self._kernel,
            self._visible,
        ) = self._compute_cell_targets()

    # ---- precompute helpers ----

    def _segments_clear(self, p1: np.ndarray, p2: np.ndarray) -> np.ndarray:
        """Vectorized "is segment p1->p2 free of wall crossings?".

        p1, p2: arrays with trailing dim 2. Returns bool array of shape
        p1.shape[:-1]. Empty wall set returns all True.
        """
        if self._seg_a.shape[0] == 0:
            return np.ones(p1.shape[:-1], dtype=bool)
        A = self._seg_a  # (L, 2)
        D = self._seg_d  # (L, 2)
        rx = p2[..., 0:1] - p1[..., 0:1]  # (..., 1)
        ry = p2[..., 1:2] - p1[..., 1:2]  # (..., 1)
        dx = D[:, 0]  # (L,)
        dy = D[:, 1]  # (L,)
        rxs = rx * dy - ry * dx  # (..., L)
        qmpx = A[:, 0] - p1[..., 0:1]  # (..., L)
        qmpy = A[:, 1] - p1[..., 1:2]  # (..., L)
        safe = np.where(np.abs(rxs) > 1e-18, rxs, 1.0)
        t = (qmpx * dy - qmpy * dx) / safe
        u = (qmpx * ry - qmpy * rx) / safe
        eps = 1e-9
        crosses = (
            (np.abs(rxs) > 1e-18)
            & (t > eps)
            & (t < 1.0 - eps)
            & (u > eps)
            & (u < 1.0 - eps)
        )
        return np.asarray(~crosses.any(axis=-1))

    def _compute_cell_targets(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """For each valid cell, store up to K=2 (pass_s, heading_vec, kernel, visible).

        A "pass" is a local minimum of distance(spline, cell). The
        heading vector is a Gaussian blend (sigma = `heading_sigma_m`)
        of the spline tangent at the pass and the unit vector from
        the cell toward the spline point. Kernel is the gating weight
        used at command time. Visibility: cell -> nearest-spline-point
        ray must not cross a wall.
        """
        K = 2
        loc = self.loc
        nx, ny = loc.bel.shape
        pass_s = np.full((nx, ny, K), np.inf, dtype=np.float64)
        pass_d = np.full((nx, ny, K), np.inf, dtype=np.float64)

        valid = loc.valid_mask > 0
        ix_v, iy_v = np.where(valid)
        cell_xy = np.stack([loc.xs[ix_v], loc.ys[iy_v]], axis=1)  # (V, 2)

        chunk = 1024
        for start in range(0, cell_xy.shape[0], chunk):
            end = min(start + chunk, cell_xy.shape[0])
            cells = cell_xy[start:end]
            dx = cells[:, None, 0] - self.xy_samples[None, :, 0]
            dy = cells[:, None, 1] - self.xy_samples[None, :, 1]
            D = np.sqrt(dx * dx + dy * dy)
            interior = (D[:, 1:-1] <= D[:, :-2]) & (D[:, 1:-1] <= D[:, 2:])
            mins_mask = np.zeros_like(D, dtype=bool)
            mins_mask[:, 1:-1] = interior
            mins_mask[:, 0] = D[:, 0] <= D[:, 1]
            mins_mask[:, -1] = D[:, -1] <= D[:, -2]
            for c_idx in range(cells.shape[0]):
                ks = np.where(mins_mask[c_idx])[0]
                if ks.size > K:
                    order = np.argsort(D[c_idx, ks])[:K]
                    ks = np.sort(ks[order])
                gx = ix_v[start + c_idx]
                gy = iy_v[start + c_idx]
                pass_s[gx, gy, : ks.size] = self.s_samples[ks]
                pass_d[gx, gy, : ks.size] = D[c_idx, ks]

        finite = np.isfinite(pass_s)
        s_lookup = np.where(finite, pass_s, 0.0)
        idx = np.clip(
            np.round(s_lookup / self.spline_sample_step_m).astype(np.int64),
            0,
            self._N - 1,
        )
        # Nearest spline point and tangent for each pass. The tangent is
        # evaluated `heading_lookahead_m` further along the curve to bias
        # the heading toward where the robot will be after a small amount
        # of latency, rather than where it is right now.
        spline_pt = self.xy_samples[idx]  # (nx, ny, K, 2)
        s_tangent = np.clip(
            s_lookup + self.heading_lookahead_m, 0.0, self.spline.S
        )
        tangent = self.spline.tangent(s_tangent.reshape(-1)).reshape(nx, ny, K, 2)

        # Cell xy broadcast against (nx, ny, K, 2).
        cx, cy = np.meshgrid(loc.xs, loc.ys, indexing="ij")
        cell_grid = np.stack([cx, cy], axis=-1)  # (nx, ny, 2)
        cell_grid_b = np.broadcast_to(cell_grid[..., None, :], (nx, ny, K, 2))

        # Toward-path unit vector.
        to_path = spline_pt - cell_grid_b
        n = np.linalg.norm(to_path, axis=-1, keepdims=True)
        to_path_unit = to_path / np.where(n > 1e-12, n, 1.0)

        # Gaussian blend factor. Close => tangent, far => toward path.
        sigma = max(1e-6, self.heading_sigma_m)
        kernel = np.exp(-(pass_d * pass_d) / (2.0 * sigma * sigma))
        kernel = np.where(finite, kernel, 0.0)

        heading_vec = (
            kernel[..., None] * tangent + (1.0 - kernel[..., None]) * to_path_unit
        )
        hn = np.linalg.norm(heading_vec, axis=-1, keepdims=True)
        heading_vec = heading_vec / np.where(hn > 1e-12, hn, 1.0)

        # Visibility: cell -> nearest spline point.
        visible = self._segments_clear(cell_grid_b, spline_pt)
        visible = visible & finite & valid[..., None]

        self._pass_d = pass_d
        return pass_s, heading_vec, kernel, visible

    # ---- public API ----

    def filtered(self, x: float, y: float) -> tuple[float, float]:
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
        return self._finished

    @property
    def current_waypoint(self) -> tuple[float, float] | None:
        if self.done:
            return None
        return self._last_aim_xy

    def nearest_on_spline(self, x: float, y: float) -> tuple[float, float, float]:
        """Project (x, y) onto the dense spline samples.

        Returns (s_nearest, x_nearest, y_nearest) for plotting and
        debugging where on the path the robot is closest to right now.
        """
        dx = self.xy_samples[:, 0] - x
        dy = self.xy_samples[:, 1] - y
        k = int(np.argmin(dx * dx + dy * dy))
        return (
            float(self.s_samples[k]),
            float(self.xy_samples[k, 0]),
            float(self.xy_samples[k, 1]),
        )

    def target_xy(self) -> tuple[float, float]:
        """Most recent belief-weighted aim point (off-spline in general)."""
        return self._last_aim_xy

    def validate_spline_vs_walls(self) -> list[ViolationReport]:
        """Return contiguous arclength runs where wall_distance < clearance."""
        if self._seg_a.shape[0] == 0:
            return []
        dists = _point_segment_distance(self.xy_samples, self._seg_a, self._seg_d)
        per_sample_min = dists.min(axis=1)
        per_sample_argmin = dists.argmin(axis=1)
        violating = per_sample_min < self.wall_clearance_m
        out: list[ViolationReport] = []
        i = 0
        N = self._N
        while i < N:
            if not violating[i]:
                i += 1
                continue
            j = i
            while j + 1 < N and violating[j + 1]:
                j += 1
            seg_min = per_sample_min[i : j + 1].min()
            wall_idx = int(
                per_sample_argmin[i + int(np.argmin(per_sample_min[i : j + 1]))]
            )
            mid_idx = (i + j) // 2
            mid_xy = self.xy_samples[mid_idx]
            out.append(
                ViolationReport(
                    s_start=float(self.s_samples[i]),
                    s_end=float(self.s_samples[j]),
                    min_distance_m=float(seg_min),
                    wall_index=wall_idx,
                    suggested_midpoint_xy=(float(mid_xy[0]), float(mid_xy[1])),
                )
            )
            i = j + 1
        return out

    def command(self, x: float, y: float) -> tuple[float, float]:
        """Return (speed_pwm, target_heading_deg)."""
        if self._finished:
            return 0.0, 0.0

        bel = self.loc.bel  # (nx, ny)
        ps = self._pass_s   # (nx, ny, K)
        eligible = (
            self._visible
            & (ps >= self.s_prog)
            & (ps <= self.s_prog + self.advance_window_m)
        )

        # Per (cell, pass) weight: belief * gaussian kernel on path
        # distance * eligibility. The kernel naturally fades far cells.
        weight = bel[..., None] * self._kernel * eligible  # (nx, ny, K)
        W = float(weight.sum())

        if W > 0:
            head_x = float((weight * self._heading_vec[..., 0]).sum() / W)
            head_y = float((weight * self._heading_vec[..., 1]).sum() / W)
            ps_safe = np.where(eligible, ps, 0.0)
            s_pass_avg = float((weight * ps_safe).sum() / W)
        else:
            # Fallback: aim along the spline tangent at the nearest
            # forward sample to (x, y).
            forward_mask = self.s_samples >= self.s_prog
            fwd_xy = self.xy_samples[forward_mask]
            fwd_s = self.s_samples[forward_mask]
            dxq = fwd_xy[:, 0] - x
            dyq = fwd_xy[:, 1] - y
            k = int(np.argmin(dxq * dxq + dyq * dyq))
            s_pass_avg = float(fwd_s[k])
            s_tan = min(self.spline.S, s_pass_avg + self.heading_lookahead_m)
            tan = self.spline.tangent(s_tan)
            head_x, head_y = float(tan[0]), float(tan[1])

        # Pre-normalization magnitude is the agreement of the
        # contributing unit headings: ~1 when they all point the same
        # way, smaller when they conflict. Used to modulate speed.
        h_mag = math.hypot(head_x, head_y)
        coherence = max(0.0, min(1.0, h_mag))
        if h_mag > 1e-12:
            head_x /= h_mag
            head_y /= h_mag
        target_dir_deg = math.degrees(math.atan2(head_y, head_x))

        # Synthetic aim point look_ahead_m ahead of the planner along
        # the commanded heading, used for plotting and as the slack in
        # the end-of-path termination test below.
        aim_x = x + self.look_ahead_m * head_x
        aim_y = y + self.look_ahead_m * head_y
        self._last_aim_xy = (aim_x, aim_y)
        self._last_s_target = s_pass_avg

        # Monotone progress.
        self.s_prog = max(self.s_prog, s_pass_avg - self.s_prog_backoff_m)
        self.log.append(self.s_prog)

        # Termination: progress reached the end and the planner is
        # geometrically within `stop_radius_mm` of the spline endpoint.
        end_xy = self.spline.xy(self.spline.S)
        end_dx = float(end_xy[0]) - x
        end_dy = float(end_xy[1]) - y
        if (
            self.s_prog >= self.spline.S - self.look_ahead_m
            and math.hypot(end_dx, end_dy) * 1000.0 < self.stop_radius_mm
        ):
            self._finished = True
            self.s_prog = self.spline.S
            return 0.0, 0.0

        # Speed scales with the agreement of the contributing headings:
        # full speed when they align, zero when they cancel.
        return self.drive_pwm * coherence, target_dir_deg


# ------------------------------ plotting ----------------------------


class BeliefPlot:
    """Heatmap + ground truth + belief argmax + ToF rays + path overlay."""

    def __init__(
        self,
        loc: Localization2D,
        map_lines,
        path: list[tuple[float, float]] | None = None,
        spline_xy: np.ndarray | None = None,
        figsize: tuple[float, float] = (10.0, 7.5),
    ):
        self.loc = loc
        self.fig, self.ax = plt.subplots(figsize=figsize)

        self.im = self.ax.imshow(
            loc.bel.T,
            origin="lower",
            extent=loc.extent,
            cmap="hot",
            vmin=0.0,
            vmax=loc.bel.max() + 1e-12,
            alpha=0.85,
        )
        for (x0, y0), (x1, y1) in map_lines:
            self.ax.plot([x0, x1], [y0, y1], "k-", lw=2)

        if path:
            px = [p[0] for p in path]
            py = [p[1] for p in path]
            self.ax.plot(px, py, "c*", ms=12, alpha=0.7, label="waypoints")
        if spline_xy is not None:
            self.ax.plot(
                spline_xy[:, 0],
                spline_xy[:, 1],
                "c-",
                lw=1.5,
                alpha=0.7,
                label="spline",
            )
        elif path:
            px = [p[0] for p in path]
            py = [p[1] for p in path]
            self.ax.plot(px, py, "c--", lw=1.5, alpha=0.7, label="path")

        (self.gt_trail,) = self.ax.plot([], [], "g-", lw=1.0, alpha=0.7)
        (self.bel_trail,) = self.ax.plot([], [], "r-", lw=1.0, alpha=0.7)
        (self.gt_pt,) = self.ax.plot([], [], "go", ms=10, label="ground truth")
        (self.bel_pt,) = self.ax.plot([], [], "rx", ms=12, mew=3, label="belief mean")
        (self.filt_pt,) = self.ax.plot(
            [], [], "y+", ms=14, mew=3, label="planner pos (filtered)"
        )
        self._gt_xs: list[float] = []
        self._gt_ys: list[float] = []
        self._bel_xs: list[float] = []
        self._bel_ys: list[float] = []
        (self.ray1,) = self.ax.plot(
            [], [], color="lime", lw=1.5, alpha=0.9, label="tof1"
        )
        (self.ray1_hit,) = self.ax.plot(
            [], [], "o", mfc="none", mec="lime", ms=8, mew=1.5
        )
        (self.ray2,) = self.ax.plot(
            [], [], color="deepskyblue", lw=1.5, alpha=0.9, label="tof2"
        )
        (self.ray2_hit,) = self.ax.plot(
            [], [], "o", mfc="none", mec="deepskyblue", ms=8, mew=1.5
        )
        # Nearest projection on spline (where the robot is right now) and
        # current target on spline (where the controller is aiming).
        (self.proj_pt,) = self.ax.plot(
            [],
            [],
            "o",
            mfc="none",
            mec="magenta",
            ms=14,
            mew=2.0,
            label="spline nearest",
        )
        (self.target_pt,) = self.ax.plot(
            [],
            [],
            "*",
            color="orange",
            ms=18,
            mec="black",
            mew=0.8,
            label="spline target",
        )
        (self.aim_seg,) = self.ax.plot(
            [],
            [],
            "-",
            color="orange",
            lw=1.5,
            alpha=0.6,
        )
        (self.proj_seg,) = self.ax.plot(
            [],
            [],
            "-",
            color="magenta",
            lw=1.0,
            alpha=0.5,
        )

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
        nearest_xy: tuple[float, float] | None = None,
        target_xy: tuple[float, float] | None = None,
        anchor_xy: tuple[float, float] | None = None,
    ):
        """Refresh all artists. Pass gt_xy=None on the real robot
        (no ground truth) -- ToF rays are then drawn from the belief
        mean instead, and the green GT trail/marker stays empty."""
        bx, by = self.loc.mean_xy()
        self.im.set_data(self.loc.bel.T)
        self.im.set_clim(vmin=0.0, vmax=self.loc.bel.max() + 1e-12)

        self._bel_xs.append(bx)
        self._bel_ys.append(by)
        self.bel_trail.set_data(self._bel_xs, self._bel_ys)
        self.bel_pt.set_data([bx], [by])

        if filtered_xy is not None:
            self.filt_pt.set_data([filtered_xy[0]], [filtered_xy[1]])

        if gt_xy is None:
            ox, oy = bx, by  # rays anchored at belief mean
        else:
            ox, oy = gt_xy
            self._gt_xs.append(ox)
            self._gt_ys.append(oy)
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

        # Spline overlays. `anchor_xy` is the source point for the
        # connecting segments (defaults to the planner pos / belief mean
        # / GT, in that order). `nearest_xy` is where the robot projects
        # onto the spline; `target_xy` is where the controller is
        # currently aiming.
        if anchor_xy is None:
            anchor_xy = filtered_xy if filtered_xy is not None else (ox, oy)
        ax_, ay_ = anchor_xy
        if nearest_xy is not None:
            self.proj_pt.set_data([nearest_xy[0]], [nearest_xy[1]])
            self.proj_seg.set_data([ax_, nearest_xy[0]], [ay_, nearest_xy[1]])
        else:
            self.proj_pt.set_data([], [])
            self.proj_seg.set_data([], [])
        if target_xy is not None:
            self.target_pt.set_data([target_xy[0]], [target_xy[1]])
            self.aim_seg.set_data([ax_, target_xy[0]], [ay_, target_xy[1]])
        else:
            self.target_pt.set_data([], [])
            self.aim_seg.set_data([], [])

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
        align_tol_deg=controller.align_tol_deg,
    )
    # Predict from the robot-side integrated PWM*s displacement; no
    # speed_factor or dt approximation needed on the host.
    filter_step(loc, resp)
    return resp, resp.current_us
