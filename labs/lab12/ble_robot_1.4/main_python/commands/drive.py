"""Drive command from subsystem_drive.h."""

from dataclasses import dataclass
from typing import Any, NamedTuple

from . import BLECommand
from ..ble.packet import DataType, PacketWriter


class DriveUpdateResponse(NamedTuple):
    current_us: int
    tof1_time_us: int
    tof1_dist_mm: int
    tof1_yaw_deg: float
    tof2_time_us: int
    tof2_dist_mm: int
    tof2_yaw_deg: float
    yaw_deg: float
    # Robot-side integrated odometry, in PWM*seconds. Multiply by
    # LocParams.pwm_to_vel_mm_s to convert to mm of displacement.
    # abs_pwm_s tracks integral(|forward|*dt) for noise scaling.
    dx_pwm_s: float
    dy_pwm_s: float
    abs_pwm_s: float


@dataclass
class DriveUpdate(BLECommand[DriveUpdateResponse]):
    target_speed: float   # PWM units, [-255, 255]
    target_heading: float # absolute yaw in degrees
    long_mode_1: bool = True  # ToF sensor 1 distance mode
    long_mode_2: bool = True  # ToF sensor 2 distance mode
    cmd_name = "DRIVE_UPDATE"

    def write_params(self, w: PacketWriter) -> None:
        w.write(float(self.target_speed))
        w.write(float(self.target_heading))
        w.write_typed(DataType.U8, 1 if self.long_mode_1 else 0)
        w.write_typed(DataType.U8, 1 if self.long_mode_2 else 0)

    def parse_response(self, fields: list[Any]) -> DriveUpdateResponse:
        return DriveUpdateResponse(*fields)


@dataclass
class DriveSetAlignTol(BLECommand[None]):
    """Set the heading-error speed-rolloff sigma (degrees) used by the
    on-board drive subsystem. Must match the host's predict-step value
    so estimated and actual displacement agree."""
    align_tol_deg: float
    cmd_name = "DRIVE_SET_ALIGN_TOL"

    def write_params(self, w: PacketWriter) -> None:
        w.write(float(self.align_tol_deg))

    def parse_response(self, fields: list[Any]) -> None:
        return None
