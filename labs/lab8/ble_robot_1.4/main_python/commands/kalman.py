"""Kalman filter commands from subsystem_kalman.h."""

from dataclasses import dataclass
from typing import Any, NamedTuple

from . import BLECommand, _chunk
from ..ble.packet import PacketWriter


class KFSample(NamedTuple):
    time: int
    dist: int   # mm
    vel: int    # mm/s


@dataclass
class KFParams(BLECommand[None]):
    drag: float
    momentum: float
    sigma_1: float
    sigma_2: float
    sigma_3: float
    cmd_name = "KF_PARAMS"
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.drag)
        w.write(self.momentum)
        w.write(self.sigma_1)
        w.write(self.sigma_2)
        w.write(self.sigma_3)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class KFReset(BLECommand[None]):
    cmd_name = "KF_RESET"
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class SendKFData(BLECommand[list[KFSample]]):
    cmd_name = "SEND_KF_DATA"
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> list[KFSample]:
        return _chunk(fields, 3, KFSample)
