"""GUI smoke test in simulation mode (skipped if PyQt5 is not installed)."""

import importlib.util
import os
import time

import pytest

pytest.importorskip("PyQt5")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5 import QtWidgets  # noqa: E402

import config  # noqa: E402
import kinematics as k  # noqa: E402

spec = importlib.util.spec_from_file_location("gui", os.path.join(os.path.dirname(__file__), "4R_Manipulator_GUI.py"))
gui = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gui)

app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def run_until(cond, timeout=6.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        app.processEvents()
        if cond():
            return True
        time.sleep(0.005)
    return False


@pytest.fixture
def window(tmp_path, monkeypatch):
    monkeypatch.setattr(config, "POINTS_PATH", tmp_path / "points.json")
    cfg = config.ArmConfig.from_dict({"max_speed_deg_s": 360})
    w = gui.MainWindow(cfg, "sim")
    yield w
    w.close()


def test_starts_at_home(window):
    assert window.cmd == (90, 90, 90, 90)
    assert window.servos.name == "sim"


def test_ik_moves_smoothly_to_target(window):
    for key, v in (("X", 12), ("Y", 8), ("Z", 16), ("Pitch", -15)):
        window.ik_spins[key].setValue(v)
    window._on_ik()
    assert window.move is not None, window.ik_status.text()
    seen = []
    assert run_until(lambda: (seen.append(window.cmd) or window.move is None))
    pose = k.forward(window.cmd, window.links(), window.cal)
    assert abs(pose.x - 12) < 1e-6 and abs(pose.y - 8) < 1e-6 and abs(pose.z - 16) < 1e-6
    assert len(seen) > 3  # passed through intermediate positions, no jump


def test_stop_freezes_mid_move(window):
    window.move_to((0, 0, 0, 0))
    run_until(lambda: window.cmd[0] < 80)
    window.stop()
    frozen = window.cmd
    run_until(lambda: False, timeout=0.2)
    assert window.cmd == frozen and window.move is None
    assert 0 < frozen[0] < 90


def test_dial_jog(window):
    window.dials[0].setValue(120)
    assert run_until(lambda: window.move is None)
    assert window.cmd[0] == 120
    assert window.servos.angles[17] == 120


def test_teach_and_play_sequence(window):
    window.points = {"one": [30.0, 80.0, 90.0, 90.0], "two": [150.0, 100.0, 90.0, 90.0]}
    window._reload_point_list()
    window.play_sequence()
    assert run_until(lambda: window.cmd == (30, 80, 90, 90))
    assert run_until(lambda: window.cmd == (150, 100, 90, 90))
    assert run_until(lambda: not window.sequence and window.move is None)


def test_unreachable_ik_reports_error(window):
    for key, v in (("X", 150), ("Y", 0), ("Z", 0), ("Pitch", 0)):
        window.ik_spins[key].setValue(v)
    window._on_ik()
    assert window.move is None
    assert "Unreachable" in window.ik_status.text()
