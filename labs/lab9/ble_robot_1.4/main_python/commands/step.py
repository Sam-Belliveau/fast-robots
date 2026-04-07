"""Step response commands from subsystem_step.h."""

from dataclasses import dataclass
from typing import Any, NamedTuple

from . import BLECommand, _chunk
from ..ble.packet import PacketWriter


class StepSample(NamedTuple):
    time: int
    dist: int   # mm
    motor: int  # PWM


@dataclass
class StepResponse(BLECommand[None]):
    duration_ms: int = 3000
    pwm: int = 100
    cmd_name = "STEP_RESPONSE"
    def write_params(self, w: PacketWriter) -> None:
        w.write(self.duration_ms)
        w.write(self.pwm)
    def parse_response(self, fields: list[Any]) -> None:
        return None


@dataclass
class SendStepData(BLECommand[list[StepSample]]):
    cmd_name = "SEND_STEP_DATA"
    def write_params(self, w: PacketWriter) -> None: pass
    def parse_response(self, fields: list[Any]) -> list[StepSample]:
        return _chunk(fields, 3, StepSample)
