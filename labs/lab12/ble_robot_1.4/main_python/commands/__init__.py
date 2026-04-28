"""BLE command base class and all command implementations."""

from abc import ABC, abstractmethod
from typing import Any

from ..ble.packet import PacketWriter
from .cmd_types import COMMAND_TYPES


class BLECommand[T](ABC):
    """Base class for all BLE commands.

    Subclasses set cmd_name to the CommandTypes enum name from
    subsystem_ble.h. The cmd_id is resolved automatically at runtime.
    """

    cmd_name: str

    @classmethod
    def cmd_id(cls) -> int:
        return COMMAND_TYPES[cls.cmd_name]

    @abstractmethod
    def write_params(self, w: PacketWriter) -> None: ...

    @abstractmethod
    def parse_response(self, fields: list[Any]) -> T: ...


def _chunk(fields: list, size: int, cls: type):
    """Split a flat field list into typed tuples of the given size."""
    return [cls(*fields[i : i + size]) for i in range(0, len(fields), size)]


# Re-export all commands and data types from submodules.
from .core import (
    Ping as Ping,
    SendTwoInts as SendTwoInts,
    SendThreeFloats as SendThreeFloats,
    GetTimeMillis as GetTimeMillis,
    Echo as Echo,
    Dance as Dance,
    SetVel as SetVel,
    StoreTimeMillis as StoreTimeMillis,
    SendTimeMillis as SendTimeMillis,
    StartRecording as StartRecording,
    StopRecording as StopRecording,
    TimeSample as TimeSample,
    GetAvgHz as GetAvgHz,
)
from .imu import (
    SendIMUData as SendIMUData,
    IMUSample as IMUSample,
)
from .tof import (
    SendToFData as SendToFData,
    ToFMode as ToFMode,
    ToFStats as ToFStats,
    ToFSample as ToFSample,
    ToFModeSample as ToFModeSample,
    ToFStatsSample as ToFStatsSample,
)
from .motors import (
    MotorCmd as MotorCmd,
    MotorStop as MotorStop,
    MotorTest as MotorTest,
    MotorCal as MotorCal,
    MotorTimeout as MotorTimeout,
)
from .pid import (
    PIDStart as PIDStart,
    PIDStop as PIDStop,
    PIDSetpoint as PIDSetpoint,
    PIDGains as PIDGains,
    PIDParams as PIDParams,
    SendPIDData as SendPIDData,
    PIDSample as PIDSample,
)
from .kalman import (
    KFParams as KFParams,
    KFReset as KFReset,
    SendKFData as SendKFData,
    KFSample as KFSample,
)
from .step import (
    StepResponse as StepResponse,
    SendStepData as SendStepData,
    StepSample as StepSample,
)
from .angle_pid import (
    AnglePIDStart as AnglePIDStart,
    AnglePIDStop as AnglePIDStop,
    AnglePIDSetpoint as AnglePIDSetpoint,
    AnglePIDSetpointRel as AnglePIDSetpointRel,
    AnglePIDGains as AnglePIDGains,
    AnglePIDParams as AnglePIDParams,
    SendAnglePIDData as SendAnglePIDData,
    AnglePIDSample as AnglePIDSample,
)
from .stunts import (
    StuntDrift as StuntDrift,
    StuntFlip as StuntFlip,
    StuntStop as StuntStop,
    StuntParams as StuntParams,
    StuntYawGains as StuntYawGains,
    SendStuntData as SendStuntData,
    StuntSample as StuntSample,
)
from .mapping import (
    MapStart as MapStart,
    MapStop as MapStop,
    MapParams as MapParams,
    SendMapData as SendMapData,
    MapStatus as MapStatus,
    MapBucket as MapBucket,
)
from .drive import (
    DriveUpdate as DriveUpdate,
    DriveUpdateResponse as DriveUpdateResponse,
)
