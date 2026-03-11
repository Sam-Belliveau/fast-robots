"""Async bridge between the GUI and BLEConnection."""

import asyncio
import dataclasses
import time
from typing import Any

from main_python.ble import BLEConnection

from .discovery import CommandInfo


class BLEBridge:
    def __init__(self):
        self._conn: BLEConnection | None = None
        self._lock = asyncio.Lock()

    @property
    def connected(self) -> bool:
        return self._conn is not None

    async def connect(self) -> None:
        async with self._lock:
            if self._conn is not None:
                return
            conn = BLEConnection()
            await conn.connect()
            self._conn = conn

    async def disconnect(self) -> None:
        async with self._lock:
            if self._conn is not None:
                await self._conn.disconnect()
                self._conn = None

    async def execute(
        self, info: CommandInfo, params: dict[str, Any]
    ) -> tuple[Any, float]:
        async with self._lock:
            if self._conn is None:
                raise ConnectionError("Not connected to robot")
            coerced: dict[str, Any] = {}
            for field in dataclasses.fields(info.cmd_class):
                raw = params.get(field.name)
                if raw is None and field.default is not dataclasses.MISSING:
                    continue
                coerced[field.name] = _coerce(raw, field.type)
            cmd = info.cmd_class(**coerced)
            t0 = time.monotonic()
            result = await self._conn.execute(cmd)
            elapsed = (time.monotonic() - t0) * 1000
            return result, elapsed


def _coerce(value: Any, target_type: type) -> Any:
    if target_type is int:
        return int(value)
    if target_type is float:
        return float(value)
    if target_type is str:
        return str(value)
    return value
