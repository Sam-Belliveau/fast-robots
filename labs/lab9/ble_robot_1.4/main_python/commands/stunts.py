"""Stunt commands from subsystem_stunts.h."""

from dataclasses import dataclass
from typing import Any, NamedTuple

from . import BLECommand, _chunk
from ..ble.packet import PacketWriter


class StuntSample(NamedTuple):
    time: int
    distance: int
    angle: int
    orientation: int  # 0=right_side_up, 1=in_between, 2=upside_down
    motor_left: int
    motor_right: int


@dataclass
class StuntDrift(BLECommand[None]):
    duration_ms: int
    cmd_name = "STUNT_DRIFT"
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.duration_ms)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class StuntFlip(BLECommand[None]):
    duration_ms: int
    cmd_name = "STUNT_FLIP"
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.duration_ms)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class StuntStop(BLECommand[None]):
    cmd_name = "STUNT_STOP"
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class StuntParams(BLECommand[None]):
    speed: float
    trigger_distance: float
    return_speed: float
    cmd_name = "STUNT_PARAMS"
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.speed)
        w.write(self.trigger_distance)
        w.write(self.return_speed)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class StuntYawGains(BLECommand[None]):
    kp: float
    kd: float
    cmd_name = "STUNT_YAW_GAINS"
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.kp)
        w.write(self.kd)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class SendStuntData(BLECommand[list[StuntSample]]):
    cmd_name = "SEND_STUNT_DATA"
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> list[StuntSample]:
        return _chunk(fields, 6, StuntSample)
