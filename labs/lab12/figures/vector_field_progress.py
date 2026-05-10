"""Belief-weighted heading vector field at successive values of s_prog.

The SplineController precomputes, for every valid grid cell, up to two
(spline tangent, toward-path) Gaussian blends. At command time it
keeps only the passes whose arclength sits inside the forward window
[s_prog, s_prog + advance_window_m], sums those unit vectors weighted
by belief and a Gaussian kernel on cell-to-spline distance, and
returns atan2 of the result.

This figure visualizes that field directly: each arrow is the
controller's commanded heading at that cell, treating the cell itself
as the belief mean. Arrow color = coherence (how strongly the
contributing passes agree, which also scales the speed). The
translucent magenta band marks the active forward window.

One SVG is written per progress value (vector_field_progress_1.svg
through vector_field_progress_N.svg). Arrows are drawn at every grid
cell in the controller's belief grid.
"""

from __future__ import annotations

from pathlib import Path

import _common  # noqa: F401

import numpy as np
import matplotlib.pyplot as plt

from controller import SplineController
from localization2d import Localization2D
from paths import LAB_PATH_M
from world import load_world

OUT_DIR = Path(__file__).resolve().parent

# Step between drawn arrows in grid cells. 1 = every cell. The
# belief grid is 192x144, so step=1 is ~27k arrows. SVG handles it.
ARROW_STEP = 6

FRACTIONS = (0.05, 0.25, 0.45, 0.65, 0.85)
LABELS = ("start", "lower corridor", "right loop", "upper corridor", "return")


def cell_heading_field(ctrl, loc, s_prog):
    """Per-cell heading direction across the full valid grid.

    Progress restricts which parts of the spline are valid targets:
    only samples in [s_prog, s_prog + advance_window_m] are eligible.
    For every valid cell we find the geometrically nearest in-window
    sample and blend the spline tangent at that sample with the unit
    vector pointing from the cell toward it. Close cells get the
    tangent (smooth flow along the path); far cells get the toward-
    path direction (rejoin the path).

    Every valid cell gets a unit-length arrow. Coherence is a belief-
    average concept, not a cell-level one, so there is no length or
    color scaling here.
    """
    s_samples = ctrl.s_samples
    xy_samples = ctrl.xy_samples
    s_end = min(ctrl.spline.S, s_prog + ctrl.advance_window_m)
    in_win = (s_samples >= s_prog) & (s_samples <= s_end)
    if not in_win.any():
        in_win = np.zeros_like(in_win)
        in_win[-1] = True
    win_xy = xy_samples[in_win]                       # (M, 2)
    win_s = s_samples[in_win]                         # (M,)

    xs = loc.xs
    ys = loc.ys
    nx, ny = xs.size, ys.size
    CX, CY = np.meshgrid(xs, ys, indexing="ij")
    cell_xy = np.stack([CX, CY], axis=-1).reshape(-1, 2)  # (V, 2)

    # Nearest in-window sample per cell.
    dx = cell_xy[:, None, 0] - win_xy[None, :, 0]
    dy = cell_xy[:, None, 1] - win_xy[None, :, 1]
    d = np.sqrt(dx * dx + dy * dy)                   # (V, M)
    k = np.argmin(d, axis=1)
    nearest_xy = win_xy[k]
    nearest_s = win_s[k]
    nearest_d = d[np.arange(d.shape[0]), k]

    # Tangent slightly ahead of the nearest sample, matching the
    # controller's heading_lookahead_m.
    s_tan = np.clip(nearest_s + ctrl.heading_lookahead_m, 0.0, ctrl.spline.S)
    tangent = ctrl.spline.tangent(s_tan)

    # Toward-path unit vector.
    to_path = nearest_xy - cell_xy
    n = np.linalg.norm(to_path, axis=-1, keepdims=True)
    to_path_unit = to_path / np.where(n > 1e-12, n, 1.0)

    # Gaussian blend (close -> tangent, far -> toward path).
    sigma = max(1e-6, ctrl.heading_sigma_m)
    blend = np.exp(-(nearest_d ** 2) / (2.0 * sigma * sigma))[:, None]
    heading = blend * tangent + (1.0 - blend) * to_path_unit
    hn = np.linalg.norm(heading, axis=-1, keepdims=True)
    heading = heading / np.where(hn > 1e-12, hn, 1.0)

    hx = heading[:, 0].reshape(nx, ny)
    hy = heading[:, 1].reshape(nx, ny)
    L = (loc.valid_mask > 0).astype(np.float64)
    return hx, hy, L


def render(ctrl, loc, lines, bounds, s_prog, label, idx):
    fig, ax = plt.subplots(figsize=(10.0, 8.0))

    # Walls
    for (x0, y0), (x1, y1) in lines:
        ax.plot([x0, x1], [y0, y1], "k-", lw=2.0, zorder=5)

    # Spline and waypoints
    ax.plot(ctrl.xy_samples[:, 0], ctrl.xy_samples[:, 1],
            color="tab:cyan", lw=1.4, alpha=0.85, zorder=3)
    px = [p[0] for p in ctrl.path]
    py = [p[1] for p in ctrl.path]
    ax.plot(px, py, "*", color="tab:cyan", ms=12,
            mec="black", mew=0.5, zorder=6)

    # Forward window on the spline
    s = ctrl.s_samples
    in_win = (s >= s_prog) & (s <= s_prog + ctrl.advance_window_m)
    if in_win.any():
        ax.plot(
            ctrl.xy_samples[in_win, 0], ctrl.xy_samples[in_win, 1],
            color="magenta", lw=6.0, alpha=0.55,
            solid_capstyle="round", zorder=4,
        )

    # Vector field at every (ARROW_STEP-th) grid cell
    hx, hy, L = cell_heading_field(ctrl, loc, s_prog)
    xs = loc.xs
    ys = loc.ys
    ix = np.arange(0, xs.size, ARROW_STEP)
    iy = np.arange(0, ys.size, ARROW_STEP)
    IX, IY = np.meshgrid(ix, iy, indexing="ij")
    X = xs[IX]
    Y = ys[IY]
    U = hx[IX, IY] * L[IX, IY]
    V = hy[IX, IY] * L[IX, IY]

    keep = L[IX, IY] > 0
    ax.quiver(
        X[keep], Y[keep], U[keep], V[keep],
        color="tab:purple",
        pivot="middle",
        angles="xy", scale_units="xy", scale=12.0,
        width=0.0042,
        headwidth=3.4, headlength=3.4, headaxislength=3.0,
        alpha=0.9, zorder=2,
    )

    # Spline position at s_prog
    sp_xy = ctrl.spline.xy(s_prog)
    ax.plot([sp_xy[0]], [sp_xy[1]], "o",
            mfc="magenta", mec="black", ms=12, mew=0.8, zorder=7)

    ax.set_aspect("equal")
    ax.set_xlim(bounds[0] - 0.02, bounds[1] + 0.02)
    ax.set_ylim(bounds[2] - 0.02, bounds[3] + 0.02)
    ax.set_title(
        f"Vector field at s_prog = {s_prog:.2f} m / {ctrl.spline.S:.2f} m  "
        f"({label})"
    )
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

    fig.tight_layout()
    out = OUT_DIR / f"vector_field_progress_{idx}.svg"
    fig.savefig(out, format="svg", bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out}")


def main():
    lines, bounds, loc_params, _, ctl_kwargs = load_world()
    loc = Localization2D(lines, bounds, loc_params)

    ctrl = SplineController(
        path=LAB_PATH_M,
        loc=loc,
        map_lines=lines,
        **ctl_kwargs,
    )

    S = ctrl.spline.S
    for i, (frac, lbl) in enumerate(zip(FRACTIONS, LABELS), start=1):
        render(ctrl, loc, lines, bounds, frac * S, lbl, i)


if __name__ == "__main__":
    main()
