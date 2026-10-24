"""Tests for motion planning, servo backends and config. Run: python -m pytest -q"""

import json
import math
import sys
import types

import pytest

import config
import servo
from motion import Move


# ---------------- motion ----------------
def test_move_endpoints_and_sync():
    m = Move.plan((0, 0, 0, 0), (90, 45, 0, -30), max_speed=90)
    assert m.sample(0) == (0, 0, 0, 0)
    assert m.sample(m.duration) == (90, 45, 0, -30)
    mid = m.sample(m.duration / 2)
    # every joint is exactly half way at the midpoint -> synchronised
    assert mid == pytest.approx((45, 22.5, 0, -15))


def test_move_peak_speed_limited():
    m = Move.plan((0,), (180,), max_speed=90)
    dt = 0.001
    peak = max(abs(m.sample(t + dt)[0] - m.sample(t)[0]) / dt for t in [i * dt for i in range(int(m.duration / dt))])
    assert peak <= 90 * 1.01


def test_move_monotonic_no_overshoot():
    m = Move.plan((10,), (100,), max_speed=120)
    samples = [m.sample(i * m.duration / 50)[0] for i in range(51)]
    assert all(b >= a for a, b in zip(samples, samples[1:]))
    assert max(samples) <= 100


# ---------------- servo backends ----------------
def test_pulse_mapping():
    assert servo.angle_to_pulse_us(0, 400, 2400) == 400
    assert servo.angle_to_pulse_us(90, 400, 2400) == 1400
    assert servo.angle_to_pulse_us(500, 400, 2400) == 2400  # clamped


class FakePi:
    def __init__(self):
        self.connected = True
        self.pulses = {}
        self.stopped = False

    def set_servo_pulsewidth(self, pin, us):
        self.pulses[pin] = us

    def stop(self):
        self.stopped = True


def test_pigpio_backend(monkeypatch):
    pi = FakePi()
    monkeypatch.setitem(sys.modules, "pigpio", types.SimpleNamespace(pi=lambda: pi))
    bank = servo.open_servos([17, 18], [(400, 2400), (500, 2500)])
    assert bank.name == "pigpio" and bank.jitter_free
    bank.write_all([90, 180])
    assert pi.pulses == {17: 1400, 18: 2500}
    bank.close()
    assert pi.pulses == {17: 0, 18: 0} and pi.stopped


def test_rpi_gpio_backend_when_no_pigpio(monkeypatch):
    calls = []

    class PWM:
        def __init__(self, pin, freq):
            self.pin = pin
            assert freq == 50

        def start(self, d):
            calls.append(("start", self.pin, d))

        def ChangeDutyCycle(self, d):
            calls.append(("duty", self.pin, round(d, 3)))

        def stop(self):
            calls.append(("stop", self.pin))

    gpio = types.SimpleNamespace(
        BCM="BCM", OUT="OUT", setwarnings=lambda f: None, setmode=lambda m: None,
        setup=lambda p, m: None, PWM=PWM, cleanup=lambda pins: calls.append(("cleanup", tuple(pins))),
    )
    rpi = types.ModuleType("RPi")
    rpi.GPIO = gpio
    monkeypatch.setitem(sys.modules, "pigpio", None)  # import fails
    monkeypatch.setitem(sys.modules, "RPi", rpi)
    monkeypatch.setitem(sys.modules, "RPi.GPIO", gpio)
    bank = servo.open_servos([17], [(400, 2400)])
    assert bank.name == "RPi.GPIO" and not bank.jitter_free
    bank.write(0, 0)
    assert ("duty", 17, 2.0) in calls  # 400 us / 20 ms = 2 % (original calibration)
    bank.write(0, 180)
    assert ("duty", 17, 12.0) in calls
    bank.close()
    assert ("cleanup", (17,)) in calls


def test_falls_back_to_simulation(monkeypatch):
    monkeypatch.setitem(sys.modules, "pigpio", None)
    monkeypatch.setitem(sys.modules, "RPi", None)
    bank = servo.open_servos([17, 18, 22, 27], [(400, 2400)] * 4)
    assert bank.name == "sim"
    bank.write_all([0, 45, 90, 180])
    assert bank.angles == {17: 0, 18: 45, 22: 90, 27: 180}


# ---------------- config ----------------
def test_default_config_loads():
    cfg = config.ArmConfig.from_dict({})
    assert cfg.pins == [17, 18, 22, 27]
    assert cfg.links.upper == 10.0


def test_shipped_config_and_points_are_valid():
    cfg = config.load_config()
    pts = config.load_points()
    for name, angles in pts.items():
        assert cfg.calibration.within_limits(angles), name


def test_bad_config_rejected():
    with pytest.raises(ValueError):
        config.ArmConfig.from_dict({"directions": [1, 2, 1, 1]})
    with pytest.raises(ValueError):
        config.ArmConfig.from_dict({"min_deg": [0, 100, 0, 0], "max_deg": [180, 90, 180, 180]})


def test_points_round_trip(tmp_path):
    p = tmp_path / "points.json"
    config.save_points({"X": [1.04, 2, 3, 4]}, p)
    assert config.load_points(p) == {"X": [1.0, 2.0, 3.0, 4.0]}
    json.loads(p.read_text())
