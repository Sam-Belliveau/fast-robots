"""Build protocol_diagram.svg from protocol_diagram.tex.

Compiles the TikZ source with pdflatex, then converts the resulting
PDF to SVG with pdftocairo. Intermediate .pdf / .aux / .log are
cleaned up afterwards.

Run from anywhere:
    python labs/lab12/figures/protocol_diagram.py
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
TEX = HERE / "protocol_diagram.tex"
PDF = HERE / "protocol_diagram.pdf"
SVG = HERE / "protocol_diagram.svg"


def run(cmd, cwd):
    print("$ " + " ".join(cmd))
    result = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    if result.returncode != 0:
        sys.stdout.write(result.stdout)
        sys.stderr.write(result.stderr)
        raise SystemExit(
            f"command failed ({result.returncode}): {' '.join(cmd)}"
        )


def main():
    run(
        ["pdflatex", "-interaction=nonstopmode", "-halt-on-error",
         TEX.name],
        cwd=HERE,
    )
    run(
        ["pdftocairo", "-svg", PDF.name, SVG.name],
        cwd=HERE,
    )
    for ext in (".aux", ".log", ".pdf"):
        p = HERE / f"protocol_diagram{ext}"
        if p.exists():
            p.unlink()
    print(f"wrote {SVG}")


if __name__ == "__main__":
    main()
