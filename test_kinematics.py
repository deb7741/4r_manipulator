"""Run with:  python -m pytest -q   (or simply:  python test_kinematics.py)"""

import itertools
import math

import kinematics as k

LINKS = k.Links(base=8.0, upper=10.0, fore=8.0, tool=4.0)


def close(a, b, tol=1e-6):
    return abs(a - b) < tol


def test_straight_up():
    # Shoulder 90° (vertical), elbow and wrist straight -> tip directly above the base.
    pose = k.forward((0, 90, 90, 90), LINKS)
    assert close(pose.x, 0) and close(pose.y, 0)
    assert close(pose.z, LINKS.base + LINKS.upper + LINKS.fore + LINKS.tool)
    assert close(pose.pitch, 90)


def test_straight_out_along_y():
    # Base 90° points the arm along +Y; everything horizontal.
    pose = k.forward((90, 0, 90, 90), LINKS)
    assert close(pose.x, 0)
    assert close(pose.y, LINKS.upper + LINKS.fore + LINKS.tool)
    assert close(pose.z, LINKS.base)


def test_original_bug_regression():
    # The original code passed degrees to math.cos and reported the base angle as Z.
    pose = k.forward((116, 56, 66, 90), LINKS)
    assert not close(pose.z, 116)
    reach = math.hypot(pose.x, pose.y)
    assert reach <= LINKS.upper + LINKS.fore + LINKS.tool + 1e-9


def test_inverse_round_trip():
    # For a grid of reachable servo settings, IK must reproduce the same tip pose.
    tested = 0
    for servo in itertools.product(range(10, 171, 40), range(20, 161, 35), range(30, 151, 40), range(40, 141, 50)):
        pose = k.forward(servo, LINKS)
        sol = k.inverse(pose, LINKS)
        assert sol is not None, f"no IK solution for {servo}"
        back = k.forward(sol, LINKS)
        assert close(back.x, pose.x, 1e-6) and close(back.y, pose.y, 1e-6) and close(back.z, pose.z, 1e-6)
        assert close(back.pitch, pose.pitch, 1e-6)
        tested += 1
    assert tested > 50


def test_unreachable_returns_none():
    assert k.inverse(k.Pose(100, 0, 0, 0), LINKS) is None


def test_presets_are_valid_servo_angles():
    import importlib.util

    if importlib.util.find_spec("PyQt5") is None:
        return  # GUI not installed; presets live in the GUI module
    spec = importlib.util.spec_from_file_location("gui", "4R_Manipulator_GUI.py")
    gui = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gui)
    for angles in gui.PRESETS.values():
        assert len(angles) == 4 and all(0 <= a <= 180 for a in angles)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok ", name)
