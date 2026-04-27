"""Angle PID commands from subsystem_angle_pid.h."""

from dataclasses import dataclass
from typing import Any, NamedTuple

from . import BLECommand, _chunk
from ..ble.packet import PacketWriter


class AnglePIDSample(NamedTuple):
    time: int
    angle: int  # yaw * 10 (0.1 deg resolution)
    error: int  # error * 10
    pwm: int
    motor_left: int
    motor_right: int
    p: int
    d: int


@dataclass
class AnglePIDStart(BLECommand[None]):
    duration_ms: int
    cmd_name = "ANGLE_PID_START"
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.duration_ms)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class AnglePIDStop(BLECommand[None]):
    cmd_name = "ANGLE_PID_STOP"
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class AnglePIDSetpoint(BLECommand[None]):
    setpoint: float
    cmd_name = "ANGLE_PID_SETPOINT"
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.setpoint)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class AnglePIDSetpointRel(BLECommand[None]):
    delta: float
    cmd_name = "ANGLE_PID_SETPOINT_REL"
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.delta)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class AnglePIDGains(BLECommand[None]):
    kp: float
    kd: float
    cmd_name = "ANGLE_PID_GAINS"
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.kp)
        w.write(self.kd)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class AnglePIDParams(BLECommand[None]):
    rc: float
    deadband: int
    cmd_name = "ANGLE_PID_PARAMS"
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.rc)
        w.write(self.deadband)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class SendAnglePIDData(BLECommand[list[AnglePIDSample]]):
    cmd_name = "SEND_ANGLE_PID_DATA"
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> list[AnglePIDSample]:
        return _chunk(fields, 8, AnglePIDSample)
