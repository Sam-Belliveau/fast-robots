"""Reusable test trajectories for the localization demo.

Points come from the lab handout in feet; the simulator/localization
work in meters, so the canonical export is `LAB_PATH_M` (already
converted). `LAB_PATH_FT` is kept for traceability.
"""

from __future__ import annotations

FT_TO_M = 0.3048

LAB_PATH_FT: list[tuple[float, float]] = [
    (-4, -3),  # 1. start
    (-2, -1),  # 2.
    (1, -1),  # 3.
    (2, -3),  # 4.
    (5, -3),  # 5.
    (5, -2),  # 6.
    (5.5, -1),
    (5.5, 2),
    (5, 3),  # 7.
    (0, 3),  # 8.
    (0, 0),  # 9. end
]

LAB_PATH_M: list[tuple[float, float]] = [
    (x * FT_TO_M, y * FT_TO_M) for x, y in LAB_PATH_FT
]
