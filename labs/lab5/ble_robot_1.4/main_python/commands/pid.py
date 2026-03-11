"""PID commands from subsystem_pid.h."""

from dataclasses import dataclass
from typing import Any, NamedTuple

from . import BLECommand, _chunk
from ..ble.packet import PacketWriter


class PIDSample(NamedTuple):
    time: int
    measurement: int
    pwm: int
    p: int
    i: int
    d: int


@dataclass
class PIDStart(BLECommand[None]):
    duration_ms: int
    @staticmethod
    def cmd_id() -> int: return 20
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.duration_ms)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class PIDStop(BLECommand[None]):
    @staticmethod
    def cmd_id() -> int: return 21
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class PIDSetpoint(BLECommand[None]):
    setpoint: float
    @staticmethod
    def cmd_id() -> int: return 22
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.setpoint)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class PIDGains(BLECommand[None]):
    kp: float
    ki: float
    kd: float
    @staticmethod
    def cmd_id() -> int: return 23
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.kp)
        w.write(self.ki)
        w.write(self.kd)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class PIDParams(BLECommand[None]):
    cap: float
    range: float
    rc: float
    deadband: int
    @staticmethod
    def cmd_id() -> int: return 24
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.cap)
        w.write(self.range)
        w.write(self.rc)
        w.write(self.deadband)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class SendPIDData(BLECommand[list[PIDSample]]):
    @staticmethod
    def cmd_id() -> int: return 25
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> list[PIDSample]:
        return _chunk(fields, 6, PIDSample)
