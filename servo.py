"""
Servo driver for the Raspberry Pi, with a simulation fallback.

Improvements over the original ``set_servo``:
  * one persistent PWM object per pin (the original created a new one and
    slept 1 s on *every* call, freezing the GUI while a dial was dragged)
  * moves are non-blocking; the caller decides when to release the servo
  * angles are clamped to the safe 0–180° range
  * GPIO is cleaned up on exit
  * runs without a Pi (prints the commands instead), so the GUI and the
    kinematics can be explored on any PC
"""

from __future__ import annotations

import atexit
from typing import Dict, Iterable

try:  # pragma: no cover - only importable on a Raspberry Pi
    import RPi.GPIO as GPIO  # type: ignore

    ON_PI = True
except (ImportError, RuntimeError):
    GPIO = None
    ON_PI = False

PWM_FREQUENCY_HZ = 50       # standard hobby servo frame rate
MIN_DUTY = 2.0              # duty cycle (%) at 0°   (~0.4 ms pulse)
MAX_DUTY = 12.0             # duty cycle (%) at 180° (~2.4 ms pulse)


def angle_to_duty(angle: float) -> float:
    angle = max(0.0, min(180.0, angle))
    return MIN_DUTY + (MAX_DUTY - MIN_DUTY) * angle / 180.0


class ServoBank:
    """Controls several servos by BCM pin number."""

    def __init__(self, pins: Iterable[int], simulate: bool = not ON_PI):
        self.pins = list(pins)
        self.simulate = simulate or not ON_PI
        self._pwm: Dict[int, object] = {}
        self.angles: Dict[int, float] = {}

        if not self.simulate:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            for pin in self.pins:
                GPIO.setup(pin, GPIO.OUT)
                pwm = GPIO.PWM(pin, PWM_FREQUENCY_HZ)
                pwm.start(0)  # 0% = no pulse, servo idle
                self._pwm[pin] = pwm
            atexit.register(self.close)

    def move(self, pin: int, angle: float) -> None:
        """Start driving ``pin`` towards ``angle`` (returns immediately)."""
        angle = max(0.0, min(180.0, float(angle)))
        self.angles[pin] = angle
        if self.simulate:
            print(f"[sim] servo GPIO{pin:<2} -> {angle:6.1f}°  (duty {angle_to_duty(angle):.2f}%)")
            return
        self._pwm[pin].ChangeDutyCycle(angle_to_duty(angle))

    def release(self, pin: int | None = None) -> None:
        """Stop sending pulses (removes jitter / saves power; the servo goes limp)."""
        if self.simulate:
            return
        for p in [pin] if pin is not None else self.pins:
            self._pwm[p].ChangeDutyCycle(0)

    def close(self) -> None:
        if self.simulate or not self._pwm:
            return
        for pwm in self._pwm.values():
            pwm.stop()
        self._pwm.clear()
        GPIO.cleanup(self.pins)
