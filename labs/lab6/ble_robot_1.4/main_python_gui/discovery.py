"""Command introspection for the GUI."""

import dataclasses
from dataclasses import fields as dc_fields
from typing import Any, get_args

from main_python.commands import (
    BLECommand,
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
    SendIMUData,
    SendToFData,
    ToFShort,
    ToFLong,
    ToFStats,
    MotorCmd,
    MotorStop,
    MotorTest,
    MotorCal,
    MotorTimeout,
    PIDStart,
    PIDStop,
    PIDSetpoint,
    PIDGains,
    PIDParams,
    SendPIDData,
)

ALL_COMMANDS: list[type[BLECommand]] = [
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
    SendIMUData,
    SendToFData,
    ToFShort,
    ToFLong,
    ToFStats,
    MotorCmd,
    MotorStop,
    MotorTest,
    MotorCal,
    MotorTimeout,
    PIDStart,
    PIDStop,
    PIDSetpoint,
    PIDGains,
    PIDParams,
    SendPIDData,
]


@dataclasses.dataclass
class FieldInfo:
    name: str
    type_name: str
    default: Any | None


@dataclasses.dataclass
class CommandInfo:
    name: str
    cmd_class: type[BLECommand]
    fields: list[FieldInfo]
    return_type_str: str


def _type_short_name(t: Any) -> str:
    if t is type(None):
        return "None"
    if isinstance(t, type):
        return t.__name__
    origin = getattr(t, "__origin__", None)
    if origin is not None:
        inner = ", ".join(_type_short_name(a) for a in get_args(t))
        return f"{getattr(origin, '__name__', str(origin))}[{inner}]"
    return str(t)


def _extract_return_type(cls: type) -> str:
    for base in getattr(cls, "__orig_bases__", ()):
        origin = getattr(base, "__origin__", None)
        if origin is BLECommand:
            args = get_args(base)
            if args:
                return _type_short_name(args[0])
    return "unknown"


def discover_commands() -> dict[str, CommandInfo]:
    registry: dict[str, CommandInfo] = {}
    for cls in ALL_COMMANDS:
        name = cls.__name__
        fields: list[FieldInfo] = []
        for f in dc_fields(cls):
            default = (
                f.default
                if f.default is not dataclasses.MISSING
                else None
            )
            type_name = (
                f.type.__name__
                if isinstance(f.type, type)
                else str(f.type)
            )
            fields.append(FieldInfo(
                name=f.name, type_name=type_name, default=default
            ))
        registry[name] = CommandInfo(
            name=name,
            cmd_class=cls,
            fields=fields,
            return_type_str=_extract_return_type(cls),
        )
    return registry


def to_json_schema(info: CommandInfo) -> dict:
    return {
        "name": info.name,
        "fields": [
            {"name": f.name, "type": f.type_name, "default": f.default}
            for f in info.fields
        ],
        "return_type": info.return_type_str,
    }
