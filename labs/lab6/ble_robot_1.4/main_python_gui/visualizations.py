"""Custom visualization registry for command results."""

import base64
import io
from typing import Any, Callable

import numpy as np
from scipy.interpolate import CubicSpline

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
    t_sec = [(t - t0) / 1e6 for t in times]

    fig, (ax_dist, ax_err, ax_motor) = plt.subplots(
        3, 1, figsize=(8, 8), sharex=True
    )

    # Distance / measurement
    ax_dist.plot(t_sec, [s.measurement for s in samples], label="measurement")
    ax_dist.set_ylabel("Distance (mm)")
    ax_dist.legend()

    # Error
    ax_err.plot(t_sec, [s.error for s in samples], label="error", color="red")
    ax_err.axhline(0, color="gray", linewidth=0.5)
    ax_err.set_ylabel("Error (mm)")
    ax_err.legend()

    # Motor output (target PWM and smoothed)
    ax_motor.plot(
        t_sec, [s.pwm for s in samples],
        label="PWM target", color="orange", alpha=0.5
    )
    ax_motor.plot(
        t_sec, [s.motor_left for s in samples],
        label="motor L", color="blue"
    )
    ax_motor.plot(
        t_sec, [s.motor_right for s in samples],
        label="motor R", color="green"
    )
    ax_motor.set_ylabel("PWM")
    ax_motor.set_xlabel("Time (s)")
    ax_motor.legend()

    fig.tight_layout()
    return _fig_to_base64(fig)


@register("SendKFData")
def _render_kf(samples: list) -> str:
    if not samples:
        return "<em>No KF data</em>"
    times = [s.time for s in samples]
    t0 = times[0]
    t_sec = [(t - t0) / 1e6 for t in times]

    fig, (ax_dist, ax_vel) = plt.subplots(
        2, 1, figsize=(8, 5), sharex=True
    )

    ax_dist.plot(t_sec, [s.dist for s in samples])
    ax_dist.set_ylabel("Distance (mm)")

    ax_vel.plot(t_sec, [s.vel for s in samples])
    ax_vel.axhline(0, color="gray", linewidth=0.5)
    ax_vel.set_ylabel("Velocity (mm/s)")
    ax_vel.set_xlabel("Time (s)")

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

    by_sensor: dict[int, dict[str, list]] = {}
    t0 = samples[0].time
    for s in samples:
        if s.sensor_id not in by_sensor:
            by_sensor[s.sensor_id] = {
                "t": [], "raw": [], "extrap": [],
                "t_raw": [], "raw_actual": [],
            }
        d = by_sensor[s.sensor_id]
        t_sec = (s.time - t0) / 1e6
        d["t"].append(t_sec)
        d["extrap"].append(s.extrapolated)
        # Only keep raw readings that are valid and represent new readings
        if s.distance >= 0:
            if not d["raw_actual"] or s.distance != d["raw_actual"][-1]:
                d["t_raw"].append(t_sec)
                d["raw_actual"].append(s.distance)

    parts: list[str] = []
    colors = {1: "tab:blue", 2: "tab:orange"}
    for sid, d in sorted(by_sensor.items()):
        c = colors.get(sid, None)
        fig, (ax_dist, ax_speed) = plt.subplots(
            2, 1, figsize=(8, 5), sharex=True
        )
        fig.suptitle(f"Sensor {sid}")

        ax_dist.plot(
            d["t_raw"], d["raw_actual"], "o",
            label="raw", color=c, markersize=4,
        )
        ax_dist.plot(
            d["t"], d["extrap"], "-",
            label="extrap", color=c, alpha=0.7,
        )
        ax_dist.set_ylabel("Distance (mm)")
        ax_dist.legend()

        # Speed: derivative of cubic spline fit to raw readings
        t_raw = d["t_raw"]
        raw = d["raw_actual"]
        if len(t_raw) >= 4:
            cs = CubicSpline(t_raw, raw)
            t_eval = np.linspace(t_raw[0], t_raw[-1], 200)
            ax_speed.plot(
                t_eval, cs(t_eval, 1),
                color=c,
            )

        ax_speed.set_ylabel("Speed (mm/s)")
        ax_speed.set_xlabel("Time (s)")
        ax_speed.axhline(0, color="gray", linewidth=0.5)

        fig.tight_layout()
        parts.append(_fig_to_base64(fig))

    return "\n".join(parts)
