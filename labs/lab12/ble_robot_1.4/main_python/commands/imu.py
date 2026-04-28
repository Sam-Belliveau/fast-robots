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
    yaw: float


@dataclass
class SendIMUData(BLECommand[list[IMUSample]]):
    cmd_name = "SEND_IMU_DATA"
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> list[IMUSample]:
        return _chunk(fields, 11, IMUSample)


@dataclass
class SetHeading(BLECommand[None]):
    """Snap imu::yaw to `heading` (degrees) by adjusting the DMP offset."""
    heading: float
    cmd_name = "SET_HEADING"
    def write_params(self, w: PacketWriter) -> None:
        w.write(float(self.heading))
    def parse_response(self, fields: list[Any]) -> None:
        return None
