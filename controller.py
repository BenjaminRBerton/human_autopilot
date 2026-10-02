"""Attitude control with heading guidance and latched altitude capture."""

from dataclasses import dataclass, field
import logging
import math

log = logging.getLogger(__name__)
MEASUREMENTS = ("heading", "airspeed", "altitude", "verticalSpeed", "roll", "pitch", "slip")
TARGETS = tuple(name + "Target" for name in MEASUREMENTS if name != "slip")
OUTPUTS = ("controlPitch", "controlRoll", "controlYaw")


def clamp(value, low, high):
    return max(low, min(high, value))


@dataclass(frozen=True)
class Gains:
    kp: float
    ki: float = 0.0
    kd: float = 0.0
    integral_limit: float = 20.0

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.kp, self.ki, self.kd, self.integral_limit)):
            raise ValueError("PID parameters must be finite")
        if self.integral_limit < 0:
            raise ValueError("integral_limit must be nonnegative")


@dataclass(frozen=True)
class Config:
    interval: float = 0.025
    roll: Gains = field(default_factory=lambda: Gains(0.01, 0.001))
    pitch: Gains = field(default_factory=lambda: Gains(0.07, 0.007))
    yaw: Gains = field(default_factory=lambda: Gains(0.025, 0.0025))
    altitude: Gains = field(default_factory=lambda: Gains(0.07, 0.007))
    bank_angle: float = 15.0
    heading_tolerance: float = 2.0
    altitude_tolerance: float = 10.0
    level_pitch: float = 2.0
    min_pitch: float = -15.0
    max_pitch: float = 10.0

    def __post_init__(self):
        if not math.isfinite(self.interval) or self.interval <= 0:
            raise ValueError("interval must be finite and positive")
        if not all(math.isfinite(v) for v in (self.bank_angle, self.heading_tolerance,
                   self.altitude_tolerance, self.level_pitch, self.min_pitch, self.max_pitch)):
            raise ValueError("Guidance parameters must be finite")
        if not (0 < self.bank_angle < 90 and 0 <= self.heading_tolerance < 180
                and self.altitude_tolerance >= 0
                and self.min_pitch < self.level_pitch < self.max_pitch):
            raise ValueError("Invalid guidance limits or tolerances")


class PID:
    """PID on error, with bounded integration and conditional anti-windup."""

    def __init__(self, gains):
        self.gains = gains
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous_error = None

    def update(self, error, dt, low=-1.0, high=1.0):
        if not math.isfinite(error) or not math.isfinite(dt) or dt <= 0:
            raise ValueError("PID requires finite error and positive dt")
        g = self.gains
        derivative = 0.0 if self.previous_error is None else (error - self.previous_error) / dt
        integral = clamp(self.integral + error * dt, -g.integral_limit, g.integral_limit)
        p, d = g.kp * error, g.kd * derivative
        raw = p + g.ki * integral + d
        # Reject integration only when it pushes further into saturation.
        change = g.ki * (integral - self.integral)
        if not ((raw > high and change > 0) or (raw < low and change < 0)):
            self.integral = integral
        self.previous_error = error
        output = clamp(p + g.ki * self.integral + d, low, high)
        log.debug("PID error=%g dt=%g P=%g I=%g D=%g output=%g",
                  error, dt, p, g.ki * self.integral, d, output)
        return output


class Controller:
    """OFF emits no commands. Each axis waits for its required input pair.

    Reset clears measurements, targets, commands, PID history, and enable state.
    Unused inputs are retained for compatibility, with no control effect.
    """

    def __init__(self, config=None):
        self.config = config or Config()
        self.enabled = False
        self.inputs = {}
        self.outputs = {}
        self.pids = {name: PID(getattr(self.config, name)) for name in ("roll", "pitch", "yaw", "altitude")}
        self.altitude_hold = False
        self.previous_altitude_error = None

    def _clear_altitude_capture(self):
        self.altitude_hold = False
        self.previous_altitude_error = None
        self.pids["altitude"].reset()
        self.pids["pitch"].reset()

    def _clear_history(self):
        self.outputs.clear()
        self._clear_altitude_capture()
        for pid in self.pids.values():
            pid.reset()

    def reset(self):
        self.enabled = False
        self.inputs.clear()
        self._clear_history()
        log.info("Reset: switched OFF and cleared inputs, commands, and PID history")

    def set_input(self, name, value):
        if name == "reset":
            self.reset()
            return
        if name == "on_off":
            if not isinstance(value, bool):
                raise ValueError("on_off must be a boolean")
            if value != self.enabled:
                self._clear_history()
                log.info("on_off=%s", value)
            self.enabled = value
            return
        if name not in MEASUREMENTS + TARGETS:
            raise ValueError(f"Unknown input: {name}")
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
            self.inputs.pop(name, None)
            self._clear_history()
            raise ValueError(f"{name} must be a finite number; cached value discarded")
        if name == "altitudeTarget" and self.inputs.get(name) != value:
            self._clear_altitude_capture()
        self.inputs[name] = float(value)
        log.debug("Input %s=%g", name, value)

    def _roll_target(self):
        if "headingTarget" not in self.inputs:
            return self.inputs.get("rollTarget", 0.0)
        if "heading" not in self.inputs:
            return None
        error = (self.inputs["headingTarget"] - self.inputs["heading"] + 180) % 360 - 180
        if abs(error) <= self.config.heading_tolerance:
            return 0.0
        return math.copysign(self.config.bank_angle, error)

    def _pitch_target(self, dt):
        values, c = self.inputs, self.config
        if "altitudeTarget" not in values:
            return values.get("pitchTarget")
        if "altitude" not in values:
            return None
        error = values["altitudeTarget"] - values["altitude"]
        crossed = (self.previous_altitude_error is not None
                   and ((self.previous_altitude_error > 0 and error <= 0)
                        or (self.previous_altitude_error < 0 and error >= 0)))
        if not self.altitude_hold and (abs(error) <= c.altitude_tolerance or crossed):
            self.altitude_hold = True
            self.pids["pitch"].reset()
            log.info("Altitude captured: target=%g measured=%g; entering altitude hold",
                     values["altitudeTarget"], values["altitude"])
        self.previous_altitude_error = error
        if not self.altitude_hold:
            return values.get("pitchTarget")
        correction = self.pids["altitude"].update(
            error, dt, c.min_pitch - c.level_pitch, c.max_pitch - c.level_pitch)
        return c.level_pitch + correction

    def step(self, dt):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be finite and positive")
        if not self.enabled:
            return {}
        result = {}
        for axis, measurement, desired, output in (
            ("pitch", "pitch", self._pitch_target(dt) if "pitch" in self.inputs else None, "controlPitch"),
            ("roll", "roll", self._roll_target(), "controlRoll"),
            ("yaw", "slip", 0.0, "controlYaw"),
        ):
            if measurement not in self.inputs or desired is None:
                log.debug("Waiting for %s inputs", axis)
                continue
            error = desired - self.inputs[measurement]
            log.debug("Axis=%s measured=%g target=%g", axis, self.inputs[measurement], desired)
            result[output] = self.pids[axis].update(error, dt)
        self.outputs = result.copy()
        return result
