"""BLE adapter that mirrors `SimRobot.update` for the real Artemis.

`RealRobot.update(target_speed, target_heading)` issues one
`DriveUpdate` command over BLE and returns the same
`DriveUpdateResponse` named tuple that `SimRobot.update` returns, so
`controller.step_once` works against either backend unchanged.
"""

from __future__ import annotations

from main_python.ble.connection import BLEConnection
from main_python.commands import DriveSetAlignTol, DriveUpdate
from main_python.commands.drive import DriveUpdateResponse


class RealRobot:
    """Thin async wrapper around `BLEConnection` exposing `update`."""

    def __init__(self, conn: BLEConnection, timeout_s: float = 5.0):
        self._conn = conn
        self.timeout_s = timeout_s
        self._last_align_tol_deg: float | None = None

    async def update(
        self,
        target_speed: float,
        target_heading: float,
        long_mode_1: bool = True,
        long_mode_2: bool = True,
        align_tol_deg: float = 30.0,
    ) -> DriveUpdateResponse:
        # Push align_tol_deg to firmware only when the value changes so
        # we don't pay a BLE round-trip every step. The firmware caches
        # it across DriveUpdate calls.
        if self._last_align_tol_deg != align_tol_deg:
            await self._conn.execute(
                DriveSetAlignTol(float(align_tol_deg)),
                timeout=self.timeout_s,
            )
            self._last_align_tol_deg = align_tol_deg
        return await self._conn.execute(
            DriveUpdate(
                float(target_speed),
                float(target_heading),
                bool(long_mode_1),
                bool(long_mode_2),
            ),
            timeout=self.timeout_s,
        )
