"""BLE adapter that mirrors `SimRobot.update` for the real Artemis.

`RealRobot.update(target_speed, target_heading)` issues one
`DriveUpdate` command over BLE and returns the same
`DriveUpdateResponse` named tuple that `SimRobot.update` returns, so
`controller.step_once` works against either backend unchanged.
"""

from __future__ import annotations

from main_python.ble.connection import BLEConnection
from main_python.commands import DriveUpdate
from main_python.commands.drive import DriveUpdateResponse


class RealRobot:
    """Thin async wrapper around `BLEConnection` exposing `update`."""

    def __init__(self, conn: BLEConnection, timeout_s: float = 5.0):
        self._conn = conn
        self.timeout_s = timeout_s

    async def update(
        self,
        target_speed: float,
        target_heading: float,
        long_mode_1: bool = True,
        long_mode_2: bool = True,
    ) -> DriveUpdateResponse:
        return await self._conn.execute(
            DriveUpdate(
                float(target_speed),
                float(target_heading),
                bool(long_mode_1),
                bool(long_mode_2),
            ),
            timeout=self.timeout_s,
        )
