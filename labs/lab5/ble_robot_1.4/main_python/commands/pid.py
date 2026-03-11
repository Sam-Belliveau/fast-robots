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
    cmd_name = "PID_START"
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.duration_ms)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class PIDStop(BLECommand[None]):
    cmd_name = "PID_STOP"
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class PIDSetpoint(BLECommand[None]):
    setpoint: float
    cmd_name = "PID_SETPOINT"
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.setpoint)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class PIDGains(BLECommand[None]):
    kp: float
    ki: float
    kd: float
    cmd_name = "PID_GAINS"
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
    cmd_name = "PID_PARAMS"
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.cap)
        w.write(self.range)
        w.write(self.rc)
        w.write(self.deadband)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class SendPIDData(BLECommand[list[PIDSample]]):
    cmd_name = "SEND_PID_DATA"
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> list[PIDSample]:
        return _chunk(fields, 6, PIDSample)
