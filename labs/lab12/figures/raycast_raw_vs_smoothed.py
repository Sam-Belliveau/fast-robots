"""Top-down raw vs smoothed ray cast at a fixed pose in the arena.

Casts 720 rays from a pose inside the L-shaped room, plots each raw
hit, and overlays the smoothed mean range from the same RayCache the
Bayes filter uses at runtime. The smoothed trace traces a slightly
shrunken version of the room outline, with corners pulled inward.
"""

from __future__ import annotations

from pathlib import Path

import _common  # noqa: F401  -- adds sim/ to sys.path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

from ray_cache import RayCache
from world import load_world

OUT = Path(__file__).resolve().parent / "raycast_raw_vs_smoothed.svg"

POSE_M = (1.0, -0.75)
N_ANGLES = 720


def raw_rays(pose, lines, n_angles, max_d_m):
    x, y = pose
    bearings = np.radians(np.arange(n_angles) * (360.0 / n_angles))
    rd = np.stack([np.cos(bearings), np.sin(bearings)], axis=1)
    lp = np.array([seg[0] for seg in lines], dtype=float)
    lq = np.array([seg[1] for seg in lines], dtype=float)
    ld = lq - lp
    diff = lp - np.array([x, y])
    denom = rd[:, 0:1] * ld[None, :, 1] - rd[:, 1:2] * ld[None, :, 0]
    t_num = diff[:, 0] * ld[:, 1] - diff[:, 1] * ld[:, 0]
    u_num = diff[None, :, 0] * rd[:, 1:2] - diff[None, :, 1] * rd[:, 0:1]
    with np.errstate(divide="ignore", invalid="ignore"):
        t = t_num[None, :] / denom
        u = u_num / denom
    bad = ~np.isfinite(t) | ~np.isfinite(u) | (t < 0) | (u < 0) | (u > 1)
    t = np.where(bad, np.inf, t)
    d = np.minimum(np.min(t, axis=1), max_d_m)
    return bearings, d


def main():
    lines, bounds, loc_params, _, _ = load_world()
    px, py = POSE_M

    bearings, d_raw = raw_rays(POSE_M, lines, N_ANGLES, loc_params.max_dist_m)
    ex_raw = px + d_raw * np.cos(bearings)
    ey_raw = py + d_raw * np.sin(bearings)

    # Build the cache with the same parameters used at runtime, but on a
    # tiny grid that contains our pose so the lookup is cheap.
    pad = 0.05
    xs = np.array([px - pad, px, px + pad])
    ys = np.array([py - pad, py, py + pad])
    cache = RayCache(
        lines,
        xs,
        ys,
        n_angles=loc_params.n_angles,
        smooth_sigma_deg=loc_params.smooth_sigma_deg,
        max_dist_m=loc_params.max_dist_m,
        sensor_sigma_mm=loc_params.sensor_sigma_mm,
        tof_sat_mm=loc_params.tof_sat_mm,
    )
    mean_mm = cache.mean_mm[1, 1]  # center cell
    std_mm = cache.std_mm[1, 1]
    s_bearings = np.radians(np.arange(loc_params.n_angles) * cache.angle_step_deg)
    d_smooth = mean_mm / 1000.0
    d_lo = np.maximum(0.0, (mean_mm - std_mm) / 1000.0)
    d_hi = (mean_mm + std_mm) / 1000.0
    ex_s = px + d_smooth * np.cos(s_bearings)
    ey_s = py + d_smooth * np.sin(s_bearings)
    ex_lo = px + d_lo * np.cos(s_bearings)
    ey_lo = py + d_lo * np.sin(s_bearings)
    ex_hi = px + d_hi * np.cos(s_bearings)
    ey_hi = py + d_hi * np.sin(s_bearings)

    fig, ax = plt.subplots(figsize=(9.0, 7.2))

    # Walls
    for (x0, y0), (x1, y1) in lines:
        ax.plot([x0, x1], [y0, y1], "k-", lw=2.0)

    # Raw 720-ray cast: red closed outline of every endpoint.
    ax.plot(
        np.append(ex_raw, ex_raw[:1]),
        np.append(ey_raw, ey_raw[:1]),
        color="tab:red",
        lw=1.0,
        label="raw 720-ray cast",
    )

    # Smoothed outline (closed polygon)
    ax.plot(
        np.append(ex_s, ex_s[:1]),
        np.append(ey_s, ey_s[:1]),
        color="tab:blue",
        lw=2.0,
        label="smoothed mean range",
    )

    # Smoothed mean +/- std band: fill between the inner and outer
    # closed polygons so a thick ring marks high-variance bearings
    # (corners, obstacle edges) and a thin ring marks flat walls.
    poly_x = np.concatenate([ex_hi, ex_lo[::-1], ex_hi[:1]])
    poly_y = np.concatenate([ey_hi, ey_lo[::-1], ey_hi[:1]])
    ax.fill(
        poly_x,
        poly_y,
        color="tab:blue",
        alpha=0.22,
        label="smoothed mean +/- std",
        linewidth=0,
    )

    # Radial tick marks every 5 deg, from (mean - std) to (mean + std).
    # Each tick points back at the pose, so the visual length is the
    # local 2*sigma of the per-bearing distance distribution.
    tick_step_deg = 5.0
    tick_idxs = np.round(
        np.arange(0.0, 360.0, tick_step_deg) / cache.angle_step_deg
    ).astype(int) % loc_params.n_angles
    tick_segs = np.stack(
        [
            np.stack([ex_lo[tick_idxs], ey_lo[tick_idxs]], axis=1),
            np.stack([ex_hi[tick_idxs], ey_hi[tick_idxs]], axis=1),
        ],
        axis=1,
    )  # (N, 2, 2): each row is [(x_lo,y_lo), (x_hi,y_hi)]
    tick_lc = LineCollection(
        tick_segs, colors="tab:blue", linewidths=1.0, alpha=0.5
    )
    ax.add_collection(tick_lc)

    # Pose marker
    ax.plot([px], [py], "k+", ms=14, mew=2.5)
    ax.text(
        px + 0.04, py - 0.06, f"pose ({px:.2f}, {py:.2f}) m", fontsize=9, color="black"
    )

    ax.set_aspect("equal")
    ax.set_xlim(bounds[0] - 0.05, bounds[1] + 0.05)
    ax.set_ylim(bounds[2] - 0.05, bounds[3] + 0.05)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(
        f"Raw ray cast vs smoothed mean range "
        f"(smooth_sigma={loc_params.smooth_sigma_deg:.0f} deg)"
    )
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(OUT, format="svg", bbox_inches="tight")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
