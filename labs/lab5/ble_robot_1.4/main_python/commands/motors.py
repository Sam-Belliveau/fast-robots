"""Motor commands from subsystem_motors.h."""

from dataclasses import dataclass
from typing import Any

from . import BLECommand
from ..ble.packet import PacketWriter


@dataclass
class MotorCmd(BLECommand[None]):
    left: int
    right: int
    @staticmethod
    def cmd_id() -> int: return 16
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.left)
        w.write(self.right)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class MotorStop(BLECommand[None]):
    @staticmethod
    def cmd_id() -> int: return 17
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class MotorCal(BLECommand[float]):
    cal: float
    @staticmethod
    def cmd_id() -> int: return 18
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.cal)
    def parse_response(self, fields: list[Any]) -> float:
        return fields[0]


@dataclass
class MotorTimeout(BLECommand[None]):
    timeout_ms: int
    @staticmethod
    def cmd_id() -> int: return 19
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.timeout_ms)
    def parse_response(self, fields: list[Any]) -> None:
        return None
