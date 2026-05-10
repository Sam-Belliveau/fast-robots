"""Regenerate every figure under labs/lab12/figures/.

Run from anywhere:
    python labs/lab12/figures/build_all.py
"""

from __future__ import annotations

import runpy
from pathlib import Path

HERE = Path(__file__).resolve().parent

SCRIPTS = [
    "protocol_diagram.py",
    "raycast_raw_vs_smoothed.py",
    "vector_field_progress.py",
]


def main():
    for name in SCRIPTS:
        path = HERE / name
        print(f"\n=== running {name} ===")
        runpy.run_path(str(path), run_name="__main__")


if __name__ == "__main__":
    main()
