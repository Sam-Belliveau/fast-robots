"""ToF commands from subsystem_tof.h."""

from dataclasses import dataclass
from typing import Any, NamedTuple

from . import BLECommand, _chunk
from ..ble.packet import PacketWriter


class ToFSample(NamedTuple):
    time: int
    distance: int
    extrapolated: int
    sensor_id: int


class ToFStatsSample(NamedTuple):
    sensor_id: int
    n: int
    mean: float
    std_dev: float


@dataclass
class SendToFData(BLECommand[list[ToFSample]]):
    cmd_name = "SEND_TOF_DATA"
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> list[ToFSample]:
        return _chunk(fields, 4, ToFSample)


class ToFModeSample(NamedTuple):
    mode_s1: int  # 0=SHORT, 1=LONG
    mode_s2: int


@dataclass
class ToFMode(BLECommand[ToFModeSample]):
    cmd_name = "TOF_MODE"
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> ToFModeSample:
        return ToFModeSample(fields[0], fields[1])


@dataclass
class ToFStats(BLECommand[list[ToFStatsSample]]):
    cmd_name = "TOF_STATS"
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> list[ToFStatsSample]:
        return _chunk(fields, 4, ToFStatsSample)
