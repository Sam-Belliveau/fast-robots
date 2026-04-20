"""Mapping commands from subsystem_mapping.h."""

from dataclasses import dataclass
from typing import Any, NamedTuple

from . import BLECommand, _chunk
from ..ble.packet import PacketWriter


class MapBucket(NamedTuple):
    index: int          # bucket index [0, N)
    angle_deg: float    # bucket center angle in degrees
    distance: float     # weighted mean distance in mm, -1 if no samples


@dataclass
class MapStart(BLECommand[None]):
    num_steps: int = 36
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
    settle_time_ms: int = 300
    step_timeout_ms: int = 4000
    samples_per_step: int = 5
    angle_std_deg: float = 5.0
    cmd_name = "MAP_PARAMS"
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.settle_threshold)
        w.write(self.settle_time_ms)
        w.write(self.step_timeout_ms)
        w.write(self.samples_per_step)
        w.write(self.angle_std_deg)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class SendMapData(BLECommand[list[MapBucket]]):
    cmd_name = "SEND_MAP_DATA"
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> list[MapBucket]:
        return _chunk(fields, 3, MapBucket)
