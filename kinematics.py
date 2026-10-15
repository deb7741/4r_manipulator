"""
Kinematics of the 4R manipulator (base yaw + three pitch joints).

Geometry
--------
            L3        L4
       o---------o---------> end effector
      /  joint 3   joint 4
  L2 /
    /  joint 2 (shoulder)
   o
   |
   | L1  (base column, rotates about the vertical axis = joint 1)
  ===

Joint angles q1..q4 (degrees):
  q1  base yaw, measured from the +X axis in the horizontal plane
  q2  shoulder pitch, measured up from the horizontal
  q3  elbow pitch, relative to link 2 (0 = straight)
  q4  wrist pitch, relative to link 3 (0 = straight)

Servo angles are converted to joint angles with a per-joint offset and
direction (``Calibration``), so the maths is the same however the servos are
mounted. Calibration normally comes from ``config.json``.

Pure Python: no GPIO, no Qt — safe to unit-test anywhere.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

Angles = Tuple[float, float, float, float]


@dataclass(frozen=True)
class Links:
    """Link lengths in any consistent unit (the GUI uses cm)."""

    base: float      # L1, vertical column height
    upper: float     # L2, shoulder -> elbow
    fore: float      # L3, elbow -> wrist
    tool: float      # L4, wrist -> end effector


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    # End-effector angle in the arm's vertical plane, degrees from horizontal
    # (equal to q2 + q3 + q4). 0 = pointing straight out, -90 = pointing down.
    pitch: float


@dataclass(frozen=True)
class Calibration:
    # Servo angle at which each joint angle is zero. Defaults: base 0° = +X,
    # shoulder 0° = horizontal, elbow and wrist straight at 90°.
    offsets: Angles = (0.0, 0.0, 90.0, 90.0)
    # +1 if increasing the servo angle increases the joint angle, else -1.
    directions: Tuple[int, int, int, int] = (1, 1, 1, 1)
    # Soft limits in servo degrees — keep the arm off the table / its own frame.
    min_angles: Angles = (0.0, 0.0, 0.0, 0.0)
    max_angles: Angles = (180.0, 180.0, 180.0, 180.0)

    def clamp(self, servo: Sequence[float]) -> Angles:
        return tuple(  # type: ignore[return-value]
            min(hi, max(lo, s)) for s, lo, hi in zip(servo, self.min_angles, self.max_angles)
        )

    def within_limits(self, servo: Sequence[float], tol: float = 1e-6) -> bool:
        return all(lo - tol <= s <= hi + tol for s, lo, hi in zip(servo, self.min_angles, self.max_angles))


DEFAULT_CAL = Calibration()


def servo_to_joint(servo_deg: Sequence[float], cal: Calibration = DEFAULT_CAL) -> Angles:
    return tuple(  # type: ignore[return-value]
        d * (s - o) for s, o, d in zip(servo_deg, cal.offsets, cal.directions)
    )


def joint_to_servo(joint_deg: Sequence[float], cal: Calibration = DEFAULT_CAL) -> Angles:
    return tuple(  # type: ignore[return-value]
        o + q / d for q, o, d in zip(joint_deg, cal.offsets, cal.directions)
    )


def joint_positions(joint_deg: Sequence[float], links: Links) -> List[Tuple[float, float, float]]:
    """3-D positions of base, shoulder, elbow, wrist and tip (for drawing)."""
    q1, q2, q3, q4 = (math.radians(a) for a in joint_deg)
    planar = [(0.0, 0.0), (0.0, links.base)]
    r, z = 0.0, links.base
    for length, angle in ((links.upper, q2), (links.fore, q2 + q3), (links.tool, q2 + q3 + q4)):
        r += length * math.cos(angle)
        z += length * math.sin(angle)
        planar.append((r, z))
    c, s = math.cos(q1), math.sin(q1)
    return [(pr * c, pr * s, pz) for pr, pz in planar]


def forward(servo_deg: Sequence[float], links: Links, cal: Calibration = DEFAULT_CAL) -> Pose:
    """End-effector pose for the given servo angles (degrees)."""
    q = servo_to_joint(servo_deg, cal)
    x, y, z = joint_positions(q, links)[-1]
    return Pose(x, y, z, q[1] + q[2] + q[3])


def inverse(
    target: Pose,
    links: Links,
    cal: Calibration = DEFAULT_CAL,
    elbow_up: Optional[bool] = None,
    prefer: Optional[Sequence[float]] = None,
) -> Optional[Angles]:
    """Servo angles that put the tip at ``target`` with the requested pitch.

    Tries facing the target and reaching back over the top, with both elbow
    configurations. Returns ``None`` if out of reach or outside the soft
    limits. If ``prefer`` (current servo angles) is given, the valid solution
    needing the least total joint movement is returned.
    """
    l2, l3 = links.upper, links.fore
    if l2 <= 0 or l3 <= 0:
        return None

    yaw = math.degrees(math.atan2(target.y, target.x))
    dist = math.hypot(target.x, target.y)
    phi = math.radians(target.pitch)

    solutions: List[Angles] = []
    for q1, r in ((yaw, dist), (yaw + 180.0, -dist), (yaw - 180.0, -dist)):
        # Wrist centre in the arm's vertical plane.
        rw = r - links.tool * math.cos(phi)
        zw = target.z - links.base - links.tool * math.sin(phi)
        d = (rw * rw + zw * zw - l2 * l2 - l3 * l3) / (2 * l2 * l3)
        if d < -1 - 1e-9 or d > 1 + 1e-9:
            continue  # wrist centre out of reach for this branch
        d = max(-1.0, min(1.0, d))

        for up in ([True, False] if elbow_up is None else [elbow_up]):
            q3 = -math.acos(d) if up else math.acos(d)
            q2 = math.atan2(zw, rw) - math.atan2(l3 * math.sin(q3), l2 + l3 * math.cos(q3))
            q4 = phi - q2 - q3
            servo = joint_to_servo((q1, math.degrees(q2), math.degrees(q3), math.degrees(q4)), cal)
            servo = tuple(_wrap(s) for s in servo)
            if cal.within_limits(servo):
                solutions.append(cal.clamp(servo))

    if not solutions:
        return None
    if prefer is None:
        return solutions[0]
    return min(solutions, key=lambda s: sum(abs(a - b) for a, b in zip(s, prefer)))


def _wrap(angle: float) -> float:
    """Shift by whole turns into (-90, 270] without changing the direction."""
    while angle > 270:
        angle -= 360
    while angle <= -90:
        angle += 360
    return angle
