"""Custom visualization registry for command results."""

import base64
import io
from typing import Any, Callable

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

_renderers: dict[str, Callable[[Any], str]] = {}


def register(command_name: str):
    """Decorator to register a custom visualization for a command."""
    def decorator(fn: Callable[[Any], str]):
        _renderers[command_name] = fn
        return fn
    return decorator


def render(command_name: str, result: Any) -> str | None:
    fn = _renderers.get(command_name)
    if fn is None:
        return None
    return fn(result)


def _fig_to_base64(fig: plt.Figure) -> str:
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=100, bbox_inches="tight")
    plt.close(fig)
    buf.seek(0)
    b64 = base64.b64encode(buf.read()).decode("ascii")
    return f'<img src="data:image/png;base64,{b64}" style="max-width:100%">'


@register("SendPIDData")
def _render_pid(samples: list) -> str:
    if not samples:
        return "<em>No PID data</em>"
    times = [s.time for s in samples]
    t0 = times[0]
    t_sec = [(t - t0) / 1000.0 for t in times]
    fig, ax1 = plt.subplots(figsize=(8, 4))
    ax1.plot(t_sec, [s.measurement for s in samples], label="measurement")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Measurement")
    ax2 = ax1.twinx()
    ax2.plot(
        t_sec, [s.pwm for s in samples],
        color="orange", alpha=0.6, label="PWM"
    )
    ax2.set_ylabel("PWM")
    fig.legend(loc="upper right")
    fig.tight_layout()
    return _fig_to_base64(fig)


@register("SendIMUData")
def _render_imu(samples: list) -> str:
    if not samples:
        return "<em>No IMU data</em>"
    times = [s.time for s in samples]
    t0 = times[0]
    t_sec = [(t - t0) / 1000.0 for t in times]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    ax1.plot(t_sec, [s.ax for s in samples], label="ax")
    ax1.plot(t_sec, [s.ay for s in samples], label="ay")
    ax1.plot(t_sec, [s.az for s in samples], label="az")
    ax1.set_ylabel("Accel (mg)")
    ax1.legend()
    ax2.plot(t_sec, [s.gx for s in samples], label="gx")
    ax2.plot(t_sec, [s.gy for s in samples], label="gy")
    ax2.plot(t_sec, [s.gz for s in samples], label="gz")
    ax2.set_ylabel("Gyro (dps)")
    ax2.set_xlabel("Time (s)")
    ax2.legend()
    fig.tight_layout()
    return _fig_to_base64(fig)


@register("SendToFData")
def _render_tof(samples: list) -> str:
    if not samples:
        return "<em>No ToF data</em>"
    by_sensor: dict[int, tuple[list, list]] = {}
    t0 = samples[0].time
    for s in samples:
        if s.sensor_id not in by_sensor:
            by_sensor[s.sensor_id] = ([], [])
        by_sensor[s.sensor_id][0].append((s.time - t0) / 1000.0)
        by_sensor[s.sensor_id][1].append(s.distance)
    fig, ax = plt.subplots(figsize=(8, 4))
    for sid, (ts, ds) in sorted(by_sensor.items()):
        ax.plot(ts, ds, label=f"Sensor {sid}")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance (mm)")
    ax.legend()
    fig.tight_layout()
    return _fig_to_base64(fig)
