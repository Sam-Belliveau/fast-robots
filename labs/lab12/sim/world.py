"""Load the lab world (line segments + bounds + filter params) from world.yaml.

Single entry point for everything the demo / notebook needs to know
about the environment and the localization tuning.
"""

from __future__ import annotations

import re
from pathlib import Path

import yaml

from localization2d import LocParams
from sim_robot import SimParams

DEFAULT_CONFIG = Path(__file__).resolve().parent / "config" / "world.yaml"
_PAIR_RE = re.compile(r"\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)")


def load_world(path: str | Path = DEFAULT_CONFIG):
    """Return (map_lines, bounds, loc_params, sim_params, ctl_kwargs).

    map_lines   : list of ((x0, y0), (x1, y1)) wall segments, meters.
    bounds      : (min_x, max_x, min_y, max_y), meters.
    loc_params  : LocParams populated from the `localization:` section.
    sim_params  : SimParams populated from the `simulation:` section.
    ctl_kwargs  : dict of SplineController fields from `controller:`.
    """
    cfg = yaml.safe_load(Path(path).read_text())

    lines = []
    for entry in cfg["world"]["lines"]:
        pts = _PAIR_RE.findall(entry)
        if len(pts) != 2:
            raise ValueError(f"expected 2 (x,y) pairs in {entry!r}, got {len(pts)}")
        (a, b), (c, d) = pts
        lines.append(((float(a), float(b)), (float(c), float(d))))

    b = cfg["world"]["bounds"]
    bounds = (b["min_x"], b["max_x"], b["min_y"], b["max_y"])

    loc_cfg = dict(cfg["localization"])
    loc_cfg["tof_sensor_angles_deg"] = tuple(loc_cfg["tof_sensor_angles_deg"])
    loc_params = LocParams(**loc_cfg)

    sim_params = SimParams(**cfg["simulation"])
    ctl_kwargs = dict(cfg["controller"])

    return lines, bounds, loc_params, sim_params, ctl_kwargs
