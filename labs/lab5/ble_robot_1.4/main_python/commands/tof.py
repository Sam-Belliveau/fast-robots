"""ToF commands from subsystem_tof.h."""

from dataclasses import dataclass
from typing import Any, NamedTuple

from . import BLECommand, _chunk
from ..ble.packet import PacketWriter


class ToFSample(NamedTuple):
    time: int
    distance: int
    sensor_id: int


class ToFStatsSample(NamedTuple):
    sensor_id: int
    n: int
    mean: float
    std_dev: float


@dataclass
class SendToFData(BLECommand[list[ToFSample]]):
    @staticmethod
    def cmd_id() -> int: return 10
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> list[ToFSample]:
        return _chunk(fields, 3, ToFSample)


@dataclass
class ToFShort(BLECommand[None]):
    @staticmethod
    def cmd_id() -> int: return 11
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class ToFLong(BLECommand[None]):
    @staticmethod
    def cmd_id() -> int: return 12
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class ToFStats(BLECommand[list[ToFStatsSample]]):
    @staticmethod
    def cmd_id() -> int: return 13
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> list[ToFStatsSample]:
        return _chunk(fields, 4, ToFStatsSample)
