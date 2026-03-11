"""Core BLE commands from subsystem_ble.h."""

from dataclasses import dataclass
from typing import Any, NamedTuple

from . import BLECommand, _chunk
from ..ble.packet import PacketWriter


class TimeSample(NamedTuple):
    time: int
    temp: float


@dataclass
class Ping(BLECommand[str]):
    @staticmethod
    def cmd_id() -> int: return 0
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> str:
        return fields[0] if fields else ""


@dataclass
class SendTwoInts(BLECommand[None]):
    a: int
    b: int
    @staticmethod
    def cmd_id() -> int: return 1
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.a)
        w.write(self.b)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class SendThreeFloats(BLECommand[None]):
    a: float
    b: float
    c: float
    @staticmethod
    def cmd_id() -> int: return 2
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.a)
        w.write(self.b)
        w.write(self.c)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class GetTimeMillis(BLECommand[TimeSample]):
    @staticmethod
    def cmd_id() -> int: return 3
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> TimeSample:
        return TimeSample(fields[0], fields[1])


@dataclass
class Echo(BLECommand[str]):
    msg: str
    @staticmethod
    def cmd_id() -> int: return 4
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.msg)
    def parse_response(self, fields: list[Any]) -> str:
        return "".join(f for f in fields if isinstance(f, str))


@dataclass
class Dance(BLECommand[None]):
    @staticmethod
    def cmd_id() -> int: return 5
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class SetVel(BLECommand[None]):
    @staticmethod
    def cmd_id() -> int: return 6
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class StoreTimeMillis(BLECommand[None]):
    count: int
    @staticmethod
    def cmd_id() -> int: return 7
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.count)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class SendTimeMillis(BLECommand[list[TimeSample]]):
    @staticmethod
    def cmd_id() -> int: return 8
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> list[TimeSample]:
        return _chunk(fields, 2, TimeSample)


@dataclass
class StartRecording(BLECommand[None]):
    @staticmethod
    def cmd_id() -> int: return 14
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class StopRecording(BLECommand[None]):
    @staticmethod
    def cmd_id() -> int: return 15
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> None:
        return None
