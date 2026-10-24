"""Load and save the arm's settings (config.json) and taught points (points.json)."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Tuple

from kinematics import Calibration, Links

HERE = Path(__file__).resolve().parent
CONFIG_PATH = HERE / "config.json"
POINTS_PATH = HERE / "points.json"

JOINT_NAMES = ("Base", "Shoulder", "Elbow", "Wrist")

DEFAULTS = {
    "pins_bcm": [17, 18, 22, 27],
    "pulse_us": [[400, 2400], [400, 2400], [400, 2400], [400, 2400]],
    "offsets_deg": [0, 0, 90, 90],
    "directions": [1, 1, 1, 1],
    "min_deg": [0, 0, 0, 0],
    "max_deg": [180, 180, 180, 180],
    "links_cm": {"base": 8.0, "upper": 10.0, "fore": 8.0, "tool": 4.0},
    "home_deg": [90, 90, 90, 90],
    "max_speed_deg_s": 90,
    "release_after_move": "auto",
    "backend": "auto",
}

DEFAULT_POINTS = {
    "A": [116, 56, 66, 90],
    "B": [69, 56, 66, 90],
    "C": [11, 56, 66, 90],
    "D": [40, 56, 66, 90],
    "E": [56, 180, 77, 90],
}


@dataclass
class ArmConfig:
    pins: List[int]
    pulse_us: List[Tuple[float, float]]
    calibration: Calibration
    links: Links
    home: Tuple[float, float, float, float]
    max_speed: float
    release_after_move: str  # "auto" | "always" | "never"
    backend: str
    raw: Dict = field(default_factory=dict)

    @classmethod
    def from_dict(cls, data: Dict) -> "ArmConfig":
        d = {**DEFAULTS, **data}
        cal = Calibration(
            offsets=tuple(float(v) for v in d["offsets_deg"]),
            directions=tuple(int(v) for v in d["directions"]),
            min_angles=tuple(float(v) for v in d["min_deg"]),
            max_angles=tuple(float(v) for v in d["max_deg"]),
        )
        _check(cal, d)
        return cls(
            pins=[int(p) for p in d["pins_bcm"]],
            pulse_us=[(float(a), float(b)) for a, b in d["pulse_us"]],
            calibration=cal,
            links=Links(**{k: float(v) for k, v in d["links_cm"].items()}),
            home=tuple(float(v) for v in d["home_deg"]),  # type: ignore[arg-type]
            max_speed=float(d["max_speed_deg_s"]),
            release_after_move=str(d["release_after_move"]),
            backend=str(d["backend"]),
            raw=d,
        )

    def to_dict(self) -> Dict:
        d = dict(self.raw)
        d["links_cm"] = {
            "base": self.links.base,
            "upper": self.links.upper,
            "fore": self.links.fore,
            "tool": self.links.tool,
        }
        return d


def _check(cal: Calibration, d: Dict) -> None:
    for key in ("pins_bcm", "pulse_us", "offsets_deg", "directions", "min_deg", "max_deg", "home_deg"):
        if len(d[key]) != 4:
            raise ValueError(f"config.json: '{key}' needs 4 values (base, shoulder, elbow, wrist)")
    if any(v not in (1, -1) for v in cal.directions):
        raise ValueError("config.json: 'directions' must be 1 or -1")
    for lo, hi in zip(cal.min_angles, cal.max_angles):
        if not 0 <= lo < hi <= 180:
            raise ValueError("config.json: need 0 <= min_deg < max_deg <= 180 for every joint")


def _dump(obj: Dict) -> str:
    """JSON with each value on one line — easy to hand-edit."""
    lines = [f"  {json.dumps(k)}: {json.dumps(v)}" for k, v in obj.items()]
    return "{\n" + ",\n".join(lines) + "\n}\n"


def load_config(path: Path = CONFIG_PATH) -> ArmConfig:
    data = json.loads(path.read_text()) if path.exists() else {}
    return ArmConfig.from_dict(data)


def save_config(cfg: ArmConfig, path: Path = CONFIG_PATH) -> None:
    path.write_text(_dump(cfg.to_dict()))


def load_points(path: Path = POINTS_PATH) -> Dict[str, List[float]]:
    if path.exists():
        return {k: [float(a) for a in v] for k, v in json.loads(path.read_text()).items()}
    return {k: list(map(float, v)) for k, v in DEFAULT_POINTS.items()}


def save_points(points: Dict[str, List[float]], path: Path = POINTS_PATH) -> None:
    path.write_text(_dump({k: [round(a, 1) for a in v] for k, v in points.items()}))
