#!/usr/bin/env python3
"""Plot ToF characterization results from tof_results.csv → PDF."""

import csv
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import os

os.chdir(os.path.dirname(os.path.abspath(__file__)))

# ── Load data ────────────────────────────────────────────────────────
rows = []
with open("tof_results.csv") as f:
    reader = csv.DictReader(f)
    for r in reader:
        if r["mean_mm"].strip() in ("", "—"):
            continue
        try:
            rows.append(
                {
                    "sensor": r["sensor"].strip(),
                    "mode": r["mode"].strip(),
                    "mean": float(r["mean_mm"]),
                    "std": float(r["std_mm"]),
                }
            )
        except (ValueError, KeyError):
            pass

# Group by (sensor, mode)
groups = {}
for r in rows:
    key = (r["sensor"], r["mode"])
    groups.setdefault(key, {"mean": [], "std": []})
    groups[key]["mean"].append(r["mean"])
    groups[key]["std"].append(r["std"])

# ── Style ────────────────────────────────────────────────────────────
plt.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.size": 11,
        "axes.grid": True,
        "grid.alpha": 0.3,
    }
)

colors = {
    ("S1", "short"): "#2196F3",
    ("S1", "long"): "#1565C0",
    ("S2", "short"): "#FF9800",
    ("S2", "long"): "#E65100",
}
markers = {"short": "o", "long": "s"}

# ── Single combined plot ─────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(12, 7))

# Stagger annotations: each series gets a different vertical offset direction
# so labels at similar distances don't pile up
annotation_offsets = {
    ("S1", "short"): (0, 30),
    ("S1", "long"): (0, -35),
    ("S2", "short"): (0, 50),
    ("S2", "long"): (0, -55),
}

x_offsets = {
    ("S1", "short"): -0.12,
    ("S1", "long"): -0.04,
    ("S2", "short"): 0.04,
    ("S2", "long"): 0.12,
}

for key in sorted(groups.keys()):
    sensor, mode = key
    data = groups[key]
    paired = sorted(zip(data["mean"], data["std"]))
    means = [m for m, _ in paired]
    stds = [s for _, s in paired]
    x = np.arange(len(means)) + x_offsets[key]

    ax.errorbar(
        x,
        means,
        yerr=stds,
        marker=markers[mode],
        color=colors[key],
        label=f"{sensor} ({mode})",
        linewidth=1.2,
        markersize=3,
        capsize=3,
        capthick=1,
    )

    # Annotate each point with mean ± std
    off = annotation_offsets[key]
    for i, (m, s) in enumerate(zip(means, stds)):
        ax.annotate(
            f"{m:.0f} ± {s:.1f} mm",
            xy=(x[i], m + (s if off[1] > 0 else -s)),
            xytext=off,
            textcoords="offset points",
            fontsize=9,
            color=colors[key],
            fontweight="bold",
            ha="center",
            va="bottom" if off[1] > 0 else "top",
            arrowprops=dict(arrowstyle="-", color=colors[key], alpha=0.4, lw=0.8),
        )

ax.set_xlabel("Measurement (sorted by distance)")
ax.set_ylabel("Distance (mm)")
ax.set_title("ToF Short vs Long Mode — Both Sensors (mean ± 1σ)")
ax.legend(fontsize=10, loc="upper left")
fig.tight_layout()

out = "../../images/tof_mode_comparison.png"
os.makedirs(os.path.dirname(out), exist_ok=True)
fig.savefig(out, bbox_inches="tight", dpi=200)
print(f"Saved: {os.path.abspath(out)}")
print("Done!")
