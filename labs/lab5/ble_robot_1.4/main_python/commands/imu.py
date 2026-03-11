"""IMU commands from subsystem_imu.h."""

from dataclasses import dataclass
from typing import Any, NamedTuple

from . import BLECommand, _chunk
from ..ble.packet import PacketWriter


class IMUSample(NamedTuple):
    time: int
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    mx: float
    my: float
    mz: float


@dataclass
class SendIMUData(BLECommand[list[IMUSample]]):
    @staticmethod
    def cmd_id() -> int: return 9
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> list[IMUSample]:
        return _chunk(fields, 10, IMUSample)
