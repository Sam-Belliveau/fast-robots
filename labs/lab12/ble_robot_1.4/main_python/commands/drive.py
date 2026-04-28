"""Drive command from subsystem_drive.h."""

from dataclasses import dataclass
from typing import Any, NamedTuple

from . import BLECommand
from ..ble.packet import PacketWriter


class DriveUpdateResponse(NamedTuple):
    current_us: int
    tof1_time_us: int
    tof1_dist_mm: int
    tof1_yaw_deg: float
    tof2_time_us: int
    tof2_dist_mm: int
    tof2_yaw_deg: float
    yaw_deg: float


@dataclass
class DriveUpdate(BLECommand[DriveUpdateResponse]):
    target_speed: float   # PWM units, [-255, 255]
    target_heading: float # absolute yaw in degrees
    cmd_name = "DRIVE_UPDATE"

    def write_params(self, w: PacketWriter) -> None:
        w.write(float(self.target_speed))
        w.write(float(self.target_heading))

    def parse_response(self, fields: list[Any]) -> DriveUpdateResponse:
        return DriveUpdateResponse(*fields)
