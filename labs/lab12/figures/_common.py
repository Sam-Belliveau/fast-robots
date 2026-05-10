"""Shared helpers for the lab 12 figure scripts.

Adds `labs/lab12/sim/` to sys.path so the figures can import the
exact RayCache / SplineController / world loader that the runtime
uses.
"""

from __future__ import annotations

import sys
from pathlib import Path

LAB_DIR = Path(__file__).resolve().parent.parent
SIM_DIR = LAB_DIR / "sim"
if str(SIM_DIR) not in sys.path:
    sys.path.insert(0, str(SIM_DIR))
