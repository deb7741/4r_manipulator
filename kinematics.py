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

Servo angles (0–180°) are converted to joint angles with a per-joint offset
and direction, so the maths stays the same whatever way the servos are
mounted. Adjust ``SERVO_OFFSETS`` / ``SERVO_DIRECTIONS`` to match your arm.

This module is pure Python (no GPIO / Qt), so it can be unit-tested and
reused anywhere.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

# Servo angle that corresponds to joint angle 0 for each joint.
# Defaults: base 0° = +X axis, shoulder 0° = horizontal,
# elbow 90° = straight, wrist 90° = straight.
SERVO_OFFSETS: Tuple[float, float, float, float] = (0.0, 0.0, 90.0, 90.0)

# +1 if increasing the servo angle increases the joint angle, -1 otherwise.
SERVO_DIRECTIONS: Tuple[int, int, int, int] = (1, 1, 1, 1)

SERVO_MIN = 0.0
SERVO_MAX = 180.0


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


def servo_to_joint(servo_deg: Sequence[float]) -> Tuple[float, float, float, float]:
    return tuple(  # type: ignore[return-value]
        d * (s - o) for s, o, d in zip(servo_deg, SERVO_OFFSETS, SERVO_DIRECTIONS)
    )


def joint_to_servo(joint_deg: Sequence[float]) -> Tuple[float, float, float, float]:
    return tuple(  # type: ignore[return-value]
        o + q / d for q, o, d in zip(joint_deg, SERVO_OFFSETS, SERVO_DIRECTIONS)
    )


def joint_positions(joint_deg: Sequence[float], links: Links):
    """Return the 3-D positions of base, shoulder, elbow, wrist and tip.

    Useful for drawing the arm. Angles in degrees.
    """
    q1, q2, q3, q4 = (math.radians(a) for a in joint_deg)
    a2 = q2
    a3 = q2 + q3
    a4 = q2 + q3 + q4

    # Radial distance and height of each joint in the arm's vertical plane.
    planar = [(0.0, 0.0), (0.0, links.base)]
    r, z = 0.0, links.base
    for length, angle in ((links.upper, a2), (links.fore, a3), (links.tool, a4)):
        r += length * math.cos(angle)
        z += length * math.sin(angle)
        planar.append((r, z))

    c, s = math.cos(q1), math.sin(q1)
    return [(pr * c, pr * s, pz) for pr, pz in planar]


def forward(servo_deg: Sequence[float], links: Links) -> Pose:
    """End-effector pose for the given servo angles (degrees)."""
    q = servo_to_joint(servo_deg)
    x, y, z = joint_positions(q, links)[-1]
    return Pose(x, y, z, q[1] + q[2] + q[3])


def inverse(
    target: Pose, links: Links, elbow_up: Optional[bool] = None
) -> Optional[Tuple[float, float, float, float]]:
    """Servo angles that place the tip at ``target`` with the given pitch.

    Returns ``None`` if the point is out of reach or needs a servo outside
    0–180°. With ``elbow_up=None`` both elbow configurations are tried.
    """
    l2, l3 = links.upper, links.fore
    if l2 <= 0 or l3 <= 0:
        return None

    yaw = math.degrees(math.atan2(target.y, target.x))
    dist = math.hypot(target.x, target.y)
    phi = math.radians(target.pitch)

    # Either face the target (radial distance +dist) or face away from it and
    # reach back over the top (base turned 180°, radial distance -dist).
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
            servo = joint_to_servo((q1, math.degrees(q2), math.degrees(q3), math.degrees(q4)))
            servo = tuple(_wrap_servo(s) for s in servo)
            if all(SERVO_MIN - 1e-6 <= s <= SERVO_MAX + 1e-6 for s in servo):
                return tuple(min(SERVO_MAX, max(SERVO_MIN, s)) for s in servo)  # type: ignore[return-value]
    return None


def _wrap_servo(angle: float) -> float:
    """Bring an angle into (-180, 180] + offsets without changing its meaning."""
    while angle > 270:
        angle -= 360
    while angle < -90:
        angle += 360
    return angle
