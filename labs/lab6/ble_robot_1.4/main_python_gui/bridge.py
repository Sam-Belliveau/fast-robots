"""Async bridge between the GUI and BLEConnection."""

import asyncio
import dataclasses
import sys
import time
from typing import Any

from aiohttp import web

from main_python.ble import BLEConnection

from .discovery import CommandInfo

SCAN_INTERVAL = 3.0  # seconds between reconnect attempts


def _log(msg: str) -> None:
    print(f"[GUI] {msg}", file=sys.stderr)


def _error(msg: str) -> None:
    print(f"[GUI ERROR] {msg}", file=sys.stderr)


class BLEBridge:
    def __init__(self):
        self._conn: BLEConnection | None = None
        self._lock = asyncio.Lock()
        self._status: str = "disconnected"
        self._ws_clients: set[web.WebSocketResponse] = set()
        self._reconnect_task: asyncio.Task | None = None

    @property
    def status(self) -> str:
        return self._status

    @property
    def connected(self) -> bool:
        return self._status == "connected"

    def register_ws(self, ws: web.WebSocketResponse) -> None:
        self._ws_clients.add(ws)

    def unregister_ws(self, ws: web.WebSocketResponse) -> None:
        self._ws_clients.discard(ws)

    async def _set_status(self, status: str) -> None:
        if status == self._status:
            return
        self._status = status
        _log(f"Status: {status}")
        msg = {"type": "status", "status": status}
        dead = []
        for ws in self._ws_clients:
            try:
                await ws.send_json(msg)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self._ws_clients.discard(ws)

    async def _try_connect(self) -> bool:
        """Attempt a single connection. Returns True on success."""
        try:
            conn = BLEConnection()
            await conn.connect(timeout=5.0)
            self._conn = conn
            await self._set_status("connected")
            return True
        except Exception as e:
            _log(f"Connect failed: {e}")
            return False

    async def _reconnect_loop(self) -> None:
        """Background loop: scan and reconnect whenever disconnected."""
        while True:
            if self._conn is None:
                await self._set_status("scanning")
                async with self._lock:
                    if self._conn is None:
                        await self._try_connect()
            else:
                # Verify the connection is still alive
                if (
                    self._conn._client is None
                    or not self._conn._client.is_connected
                ):
                    async with self._lock:
                        self._conn = None
                    await self._set_status("disconnected")
                    continue
            await asyncio.sleep(SCAN_INTERVAL)

    def start(self) -> None:
        """Start the background reconnect loop."""
        self._reconnect_task = asyncio.ensure_future(
            self._reconnect_loop()
        )

    async def disconnect(self) -> None:
        if self._reconnect_task:
            self._reconnect_task.cancel()
            try:
                await self._reconnect_task
            except asyncio.CancelledError:
                pass
        async with self._lock:
            if self._conn is not None:
                await self._conn.disconnect()
                self._conn = None
        await self._set_status("disconnected")

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
                if raw is None:
                    _error(
                        f"{info.cmd_class.__name__}: "
                        f"missing required param '{field.name}'"
                    )
                    raise ValueError(
                        f"Missing required parameter: {field.name}"
                    )
                try:
                    coerced[field.name] = _coerce(raw, field.type)
                except (ValueError, TypeError) as e:
                    _error(
                        f"{info.cmd_class.__name__}: "
                        f"failed to coerce '{field.name}'={raw!r} "
                        f"to {field.type}: {e}"
                    )
                    raise
            cmd = info.cmd_class(**coerced)
            t0 = time.monotonic()
            try:
                result = await self._conn.execute(cmd)
            except Exception:
                # Connection may have died mid-command
                if (
                    self._conn._client is None
                    or not self._conn._client.is_connected
                ):
                    self._conn = None
                    asyncio.ensure_future(
                        self._set_status("disconnected")
                    )
                raise
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
