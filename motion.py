"""
Smooth, synchronised joint motion.

Instead of snapping every servo to its target (which jerks the arm and can
tip it over), a move is planned so that all joints start and finish together,
the slowest joint moves at ``max_speed`` deg/s, and velocity eases in and out
(cosine profile). The planner is pure Python; the GUI samples it on a timer.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

Angles = Tuple[float, ...]

MIN_DURATION_S = 0.15


@dataclass(frozen=True)
class Move:
    start: Angles
    goal: Angles
    duration: float  # seconds

    @classmethod
    def plan(cls, start: Sequence[float], goal: Sequence[float], max_speed: float) -> "Move":
        """Plan a move whose fastest joint averages ``max_speed`` deg/s.

        The cosine profile's peak speed is pi/2 x its average, so the average
        is scaled down to keep the *peak* at ``max_speed``.
        """
        largest = max((abs(g - s) for s, g in zip(start, goal)), default=0.0)
        if max_speed <= 0:
            duration = MIN_DURATION_S
        else:
            duration = max(MIN_DURATION_S, largest * (math.pi / 2) / max_speed)
        return cls(tuple(start), tuple(goal), duration)

    def sample(self, t: float) -> Angles:
        """Joint angles ``t`` seconds after the move started."""
        if t >= self.duration:
            return self.goal
        if t <= 0:
            return self.start
        s = 0.5 - 0.5 * math.cos(math.pi * t / self.duration)  # 0 -> 1, eased
        return tuple(a + (b - a) * s for a, b in zip(self.start, self.goal))

    def done(self, t: float) -> bool:
        return t >= self.duration
