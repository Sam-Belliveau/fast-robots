"""Parse CommandTypes enum from subsystem_ble.h at import time."""

import re
from pathlib import Path

_HEADER = Path(__file__).resolve().parent.parent.parent / "main" / "subsystem_ble.h"


def _parse(path: Path = _HEADER) -> dict[str, int]:
    text = path.read_text()
    m = re.search(r"enum\s+CommandTypes\s*\{([^}]+)\}", text)
    if not m:
        raise RuntimeError(f"CommandTypes enum not found in {path}")
    return {
        name: int(val)
        for name, val in re.findall(r"(\w+)\s*=\s*(\d+)", m.group(1))
    }


COMMAND_TYPES: dict[str, int] = _parse()
