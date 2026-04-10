"""Mapping commands from subsystem_mapping.h."""

from dataclasses import dataclass
from typing import Any, NamedTuple

from . import BLECommand, _chunk
from ..ble.packet import PacketWriter


class MapSample(NamedTuple):
    time: int
    angle: int  # yaw * 10 (0.1 deg resolution)
    tof1: int   # front sensor distance (mm)
    tof2: int   # side sensor distance (mm)


@dataclass
class MapStart(BLECommand[None]):
    num_steps: int = 18
    cmd_name = "MAP_START"
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.num_steps)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class MapStop(BLECommand[None]):
    cmd_name = "MAP_STOP"
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class MapParams(BLECommand[None]):
    settle_threshold: float = 5.0
    settle_time_ms: int = 500
    step_timeout_ms: int = 5000
    cmd_name = "MAP_PARAMS"
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.settle_threshold)
        w.write(self.settle_time_ms)
        w.write(self.step_timeout_ms)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class SendMapData(BLECommand[list[MapSample]]):
    cmd_name = "SEND_MAP_DATA"
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> list[MapSample]:
        return _chunk(fields, 4, MapSample)
