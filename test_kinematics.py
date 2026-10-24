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


def test_inverse_respects_soft_limits():
    pose = k.forward((10, 55, 150, 140), LINKS)
    tight = k.Calibration(max_angles=(180, 180, 120, 180))
    sol = k.inverse(pose, LINKS, tight)
    assert sol is None or sol[2] <= 120


def test_inverse_prefers_least_motion():
    pose = k.forward((60, 70, 50, 100), LINKS)
    current = (60, 70, 50, 100)
    sol = k.inverse(pose, LINKS, prefer=current)
    assert sol is not None
    assert sum(abs(a - b) for a, b in zip(sol, current)) < 1e-6


def test_reversed_servo_direction_round_trip():
    cal = k.Calibration(directions=(1, -1, 1, -1), offsets=(0, 180, 90, 90))
    pose = k.forward((30, 120, 80, 100), LINKS, cal)
    sol = k.inverse(pose, LINKS, cal)
    back = k.forward(sol, LINKS, cal)
    assert close(back.x, pose.x) and close(back.z, pose.z)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok ", name)
