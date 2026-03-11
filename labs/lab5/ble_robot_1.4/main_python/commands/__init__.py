"""BLE command base class and all command implementations."""

from abc import ABC, abstractmethod
from typing import Any

from ..ble.packet import PacketWriter


class BLECommand[T](ABC):
    """Base class for all BLE commands.

    Subclasses define cmd_id, how to serialize parameters, and how to
    parse the flat list of response values into a typed result T.
    """

    @staticmethod
    @abstractmethod
    def cmd_id() -> int: ...

    @abstractmethod
    def write_params(self, w: PacketWriter) -> None: ...

    @abstractmethod
    def parse_response(self, fields: list[Any]) -> T: ...


def _chunk(fields: list, size: int, cls: type):
    """Split a flat field list into typed tuples of the given size."""
    return [cls(*fields[i : i + size]) for i in range(0, len(fields), size)]


# Re-export all commands and data types from submodules.
from .core import (
    Ping,
    SendTwoInts,
    SendThreeFloats,
    GetTimeMillis,
    Echo,
    Dance,
    SetVel,
    StoreTimeMillis,
    SendTimeMillis,
    StartRecording,
    StopRecording,
    TimeSample,
)
from .imu import SendIMUData, IMUSample
from .tof import SendToFData, ToFShort, ToFLong, ToFStats, ToFSample, ToFStatsSample
from .motors import MotorCmd, MotorStop, MotorCal, MotorTimeout
from .pid import (
    PIDStart,
    PIDStop,
    PIDSetpoint,
    PIDGains,
    PIDParams,
    SendPIDData,
    PIDSample,
)

__all__ = [
    "BLECommand",
    "_chunk",
    "Ping",
    "SendTwoInts",
    "SendThreeFloats",
    "GetTimeMillis",
    "Echo",
    "Dance",
    "SetVel",
    "StoreTimeMillis",
    "SendTimeMillis",
    "StartRecording",
    "StopRecording",
    "TimeSample",
    "SendIMUData",
    "IMUSample",
    "SendToFData",
    "ToFShort",
    "ToFLong",
    "ToFStats",
    "ToFSample",
    "ToFStatsSample",
    "MotorCmd",
    "MotorStop",
    "MotorCal",
    "MotorTimeout",
    "PIDStart",
    "PIDStop",
    "PIDSetpoint",
    "PIDGains",
    "PIDParams",
    "SendPIDData",
    "PIDSample",
]
