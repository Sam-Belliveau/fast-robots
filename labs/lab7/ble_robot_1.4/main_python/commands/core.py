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
    cmd_name = "PING"
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> str:
        return fields[0] if fields else ""


@dataclass
class SendTwoInts(BLECommand[None]):
    a: int
    b: int
    cmd_name = "SEND_TWO_INTS"
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
    cmd_name = "SEND_THREE_FLOATS"
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.a)
        w.write(self.b)
        w.write(self.c)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class GetTimeMillis(BLECommand[TimeSample]):
    cmd_name = "GET_TIME_MILLIS"
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> TimeSample:
        return TimeSample(fields[0], fields[1])


@dataclass
class Echo(BLECommand[str]):
    msg: str
    cmd_name = "ECHO"
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.msg)
    def parse_response(self, fields: list[Any]) -> str:
        return "".join(f for f in fields if isinstance(f, str))


@dataclass
class Dance(BLECommand[None]):
    cmd_name = "DANCE"
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class SetVel(BLECommand[None]):
    cmd_name = "SET_VEL"
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class StoreTimeMillis(BLECommand[None]):
    count: int
    cmd_name = "STORE_TIME_MILLIS"
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.count)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class SendTimeMillis(BLECommand[list[TimeSample]]):
    cmd_name = "SEND_TIME_MILLIS"
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> list[TimeSample]:
        return _chunk(fields, 2, TimeSample)


@dataclass
class StartRecording(BLECommand[None]):
    cmd_name = "START_RECORDING"
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class StopRecording(BLECommand[None]):
    cmd_name = "STOP_RECORDING"
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class GetAvgHz(BLECommand[float]):
    cmd_name = "GET_AVG_HZ"
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> float:
        return fields[0] if fields else 0.0
