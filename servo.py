"""
Servo drivers for the Raspberry Pi, with a simulation fallback.

Backends, in order of preference:

  pigpio    hardware-timed pulses from the pigpio daemon — rock-steady, no
            jitter. Recommended. Install: ``sudo apt install pigpio
            python3-pigpio`` then ``sudo systemctl enable --now pigpiod``.
  RPi.GPIO  software PWM — works everywhere but servos twitch slightly, so
            pulses are released after each move by default.
  sim       no hardware; commands are logged. Lets you use the GUI on any PC.

All backends take angles in degrees and convert them with each servo's pulse
range (``min_pulse_us`` .. ``max_pulse_us``), which lives in config.json.
"""

from __future__ import annotations

import atexit
import logging
from typing import Dict, Optional, Sequence

log = logging.getLogger("servo")

PWM_FREQUENCY_HZ = 50  # standard hobby-servo frame rate (20 ms)


def angle_to_pulse_us(angle: float, min_us: float, max_us: float) -> float:
    angle = max(0.0, min(180.0, float(angle)))
    return min_us + (max_us - min_us) * angle / 180.0


class ServoBank:
    """Base class / simulator. Subclasses talk to real hardware."""

    name = "sim"
    jitter_free = True  # simulation never jitters

    def __init__(self, pins: Sequence[int], pulse_ranges: Sequence[Sequence[float]]):
        if len(pins) != len(pulse_ranges):
            raise ValueError("one pulse range per pin is required")
        self.pins = list(pins)
        self.ranges = [tuple(r) for r in pulse_ranges]
        self.angles: Dict[int, float] = {}
        self._closed = False

    # --- public API -------------------------------------------------------
    def write(self, joint: int, angle: float) -> None:
        """Command joint ``joint`` (0-based) to ``angle`` degrees. Non-blocking."""
        pin = self.pins[joint]
        lo, hi = self.ranges[joint]
        angle = max(0.0, min(180.0, float(angle)))
        self.angles[pin] = angle
        self._set_pulse(pin, angle_to_pulse_us(angle, lo, hi))

    def write_all(self, angles: Sequence[float]) -> None:
        for joint, angle in enumerate(angles):
            self.write(joint, angle)

    def release(self) -> None:
        """Stop sending pulses: the servos go limp (no jitter, no current)."""
        for pin in self.pins:
            self._off(pin)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self.release()
        finally:
            self._shutdown()

    # --- backend hooks ----------------------------------------------------
    def _set_pulse(self, pin: int, pulse_us: float) -> None:
        log.debug("[sim] GPIO%-2d -> %.0f us", pin, pulse_us)

    def _off(self, pin: int) -> None:
        pass

    def _shutdown(self) -> None:
        pass


class PigpioServoBank(ServoBank):
    name = "pigpio"
    jitter_free = True

    def __init__(self, pins, pulse_ranges, pi):
        super().__init__(pins, pulse_ranges)
        self._pi = pi
        atexit.register(self.close)

    def _set_pulse(self, pin, pulse_us):
        self._pi.set_servo_pulsewidth(pin, int(round(pulse_us)))

    def _off(self, pin):
        self._pi.set_servo_pulsewidth(pin, 0)

    def _shutdown(self):
        self._pi.stop()


class RPiGPIOServoBank(ServoBank):
    name = "RPi.GPIO"
    jitter_free = False

    def __init__(self, pins, pulse_ranges, gpio):
        super().__init__(pins, pulse_ranges)
        self._gpio = gpio
        gpio.setwarnings(False)
        gpio.setmode(gpio.BCM)
        self._pwm = {}
        for pin in self.pins:
            gpio.setup(pin, gpio.OUT)
            pwm = gpio.PWM(pin, PWM_FREQUENCY_HZ)
            pwm.start(0)
            self._pwm[pin] = pwm
        atexit.register(self.close)

    def _set_pulse(self, pin, pulse_us):
        period_us = 1_000_000 / PWM_FREQUENCY_HZ
        self._pwm[pin].ChangeDutyCycle(100.0 * pulse_us / period_us)

    def _off(self, pin):
        self._pwm[pin].ChangeDutyCycle(0)

    def _shutdown(self):
        for pwm in self._pwm.values():
            pwm.stop()
        self._gpio.cleanup(self.pins)


def open_servos(pins: Sequence[int], pulse_ranges, backend: str = "auto") -> ServoBank:
    """Open the best available backend.

    ``backend`` is "auto", "pigpio", "rpi" or "sim".
    """
    backend = backend.lower()
    if backend in ("auto", "pigpio"):
        pi = _connect_pigpio()
        if pi is not None:
            return PigpioServoBank(pins, pulse_ranges, pi)
        if backend == "pigpio":
            raise RuntimeError("pigpio daemon not reachable — run: sudo systemctl enable --now pigpiod")
    if backend in ("auto", "rpi"):
        gpio = _import_rpi_gpio()
        if gpio is not None:
            return RPiGPIOServoBank(pins, pulse_ranges, gpio)
        if backend == "rpi":
            raise RuntimeError("RPi.GPIO is not available on this machine")
    return ServoBank(pins, pulse_ranges)


def _connect_pigpio() -> Optional[object]:
    try:
        import pigpio  # type: ignore
    except ImportError:
        return None
    pi = pigpio.pi()
    if not pi.connected:
        return None
    return pi


def _import_rpi_gpio():
    try:
        import RPi.GPIO as GPIO  # type: ignore

        return GPIO
    except (ImportError, RuntimeError):
        return None
