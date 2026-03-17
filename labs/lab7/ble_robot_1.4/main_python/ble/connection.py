"""Async BLE connection manager using bleak."""

import asyncio
import platform
import sys
from pathlib import Path
from typing import Any

import yaml
from bleak import BleakClient, BleakScanner

from ..commands import BLECommand
from .packet import PacketReader, PacketWriter


def _log(msg: str) -> None:
    """Print a BLE log message to stderr."""
    print(f"[BLE] {msg}", file=sys.stderr)


def _error(msg: str) -> None:
    """Print a BLE error to stderr."""
    print(f"[BLE ERROR] {msg}", file=sys.stderr)


class BLEConnection:
    def __init__(self, config_path: str | Path | None = None):
        if config_path is None:
            config_path = Path(__file__).parent / "connection.yaml"
        with open(config_path) as f:
            cfg = yaml.safe_load(f)

        self._address: str = cfg["artemis_address"]
        self._service_uuid: str = cfg["ble_service"]
        self._rx_cmd_uuid: str = cfg["rx_cmd"]
        self._tx_data_uuid: str = cfg["tx_data"]

        self._client: BleakClient | None = None
        self._next_req_id: int = 1
        self._pending: dict[int, tuple[BLECommand, asyncio.Future, list]] = {}

    async def connect(self, timeout: float = 10.0) -> None:
        if platform.system() == "Darwin":
            # macOS CoreBluetooth doesn't expose real MAC addresses;
            # scan by service UUID instead.
            devices = await BleakScanner.discover(
                timeout=timeout,
                service_uuids=[self._service_uuid],
            )
            device = devices[0] if devices else None
        else:
            device = await BleakScanner.find_device_by_address(
                self._address, timeout=timeout
            )
        if device is None:
            raise ConnectionError(
                f"Could not find device with address {self._address}"
            )

        self._client = BleakClient(device)
        await self._client.connect()
        await self._client.start_notify(
            self._tx_data_uuid, self._notification_handler
        )

    async def disconnect(self) -> None:
        if self._client and self._client.is_connected:
            await self._client.stop_notify(self._tx_data_uuid)
            await self._client.disconnect()
        self._client = None

    async def execute(
        self, command: BLECommand[Any], timeout: float = 30.0
    ) -> Any:
        if self._client is None or not self._client.is_connected:
            raise ConnectionError("Not connected")

        req_id = self._next_req_id
        self._next_req_id = (self._next_req_id + 1) & 0xFFFF
        if self._next_req_id == 0:
            self._next_req_id = 1

        writer = PacketWriter(req_id, command.cmd_id())
        command.write_params(writer)

        loop = asyncio.get_running_loop()
        future: asyncio.Future = loop.create_future()
        self._pending[req_id] = (command, future, [])

        payload = writer.to_bytes()
        _log(
            f"TX {type(command).__name__} req_id={req_id} "
            f"cmd_id={command.cmd_id()} ({len(payload)} bytes)"
        )
        await self._client.write_gatt_char(
            self._rx_cmd_uuid, payload, response=False
        )

        try:
            return await asyncio.wait_for(future, timeout=timeout)
        except asyncio.TimeoutError:
            self._pending.pop(req_id, None)
            _error(
                f"{type(command).__name__} (req_id={req_id}) timed out "
                f"after {timeout}s"
            )
            raise TimeoutError(
                f"Command {type(command).__name__} (req_id={req_id}) timed out"
            )

    def _notification_handler(self, _sender: Any, data: bytearray) -> None:
        reader = PacketReader(data)
        req_id = reader.req_id

        entry = self._pending.get(req_id)
        if entry is None:
            _error(
                f"Received notification for unknown req_id={req_id} "
                f"(data={data.hex()})"
            )
            return

        command, future, accumulated = entry
        try:
            fields = reader.read_all()
        except Exception as e:
            self._pending.pop(req_id, None)
            _error(
                f"{type(command).__name__} (req_id={req_id}): {e}"
            )
            if not future.done():
                future.get_loop().call_soon_threadsafe(
                    future.set_exception, e
                )
            return
        accumulated.extend(fields)

        if reader.has_end:
            self._pending.pop(req_id, None)
            try:
                result = command.parse_response(accumulated)
            except Exception as e:
                _error(
                    f"{type(command).__name__} (req_id={req_id}): "
                    f"parse_response failed: {e} (fields={accumulated})"
                )
                if not future.done():
                    future.get_loop().call_soon_threadsafe(
                        future.set_exception, e
                    )
                return
            if not future.done():
                future.get_loop().call_soon_threadsafe(future.set_result, result)
        else:
            self._pending[req_id] = (command, future, accumulated)

    async def __aenter__(self):
        await self.connect()
        return self

    async def __aexit__(self, *exc):
        await self.disconnect()
