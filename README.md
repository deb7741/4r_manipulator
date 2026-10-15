# 4R Manipulator

An educational 4R spherical manipulator for learning **forward and inverse kinematics**, driven by a Raspberry Pi and a PyQt5 GUI.

![joints](https://img.shields.io/badge/joints-4%20servos-blue) ![python](https://img.shields.io/badge/python-3.8%2B-green)

## Features

- **Forward kinematics** — turn the four joint dials and watch the tip's X, Y, Z and pitch update live.
- **Inverse kinematics** — type a target X/Y/Z and tool pitch, press **Go**, and the arm solves for the joint angles (both elbow configurations and reach-over-the-top are tried).
- **Live side and top views** of the arm.
- **Taught points A–E** for repeatable demos.
- **Simulation mode** — runs on any PC without a Pi; servo commands are printed instead of sent.

## Hardware

| Joint | Servo GPIO (BCM) | Physical pin |
|---|---|---|
| Base (yaw) | 17 | 11 |
| Shoulder | 18 | 12 |
| Elbow | 22 | 15 |
| Wrist | 27 | 13 |

Power the servos from a separate 5–6 V supply sized for all four motors, and connect its ground to the Pi's ground. Don't power servos from the Pi's 5 V pin.

## Run

```bash
pip install -r requirements.txt
python3 4R_Manipulator_GUI.py              # on a Pi: drives the servos
python3 4R_Manipulator_GUI.py --simulate   # anywhere: no GPIO
```

## Calibrating to your arm

Joint angles are computed from servo angles using the offsets and directions at the top of `kinematics.py`:

```python
SERVO_OFFSETS    = (0.0, 0.0, 90.0, 90.0)  # servo angle at which each joint is "zero"
SERVO_DIRECTIONS = (1, 1, 1, 1)            # flip to -1 if a servo is mounted reversed
```

Defaults: base 0° points along +X, shoulder 0° is horizontal, elbow and wrist are straight at 90°. Set the shoulder to 90° and elbow/wrist to 90° — the arm should point straight up. If it doesn't, adjust the offsets until it does.

Set the four link lengths (in cm) in the GUI to match your build.

## Files

| File | Purpose |
|---|---|
| `4R_Manipulator_GUI.py` | PyQt5 interface |
| `kinematics.py` | Forward / inverse kinematics (pure Python, no hardware) |
| `servo.py` | Raspberry Pi PWM servo driver with simulation fallback |
| `test_kinematics.py` | Unit tests — run `python -m pytest -q` |

## The maths

With base yaw θ₁ and pitch joints θ₂, θ₃, θ₄ (each measured relative to the previous link):

```
r = L2·cos θ2 + L3·cos(θ2+θ3) + L4·cos(θ2+θ3+θ4)
z = L1 + L2·sin θ2 + L3·sin(θ2+θ3) + L4·sin(θ2+θ3+θ4)
x = r·cos θ1,   y = r·sin θ1,   pitch = θ2+θ3+θ4
```

Inverse kinematics subtracts the tool link to find the wrist centre, solves the two-link (L2, L3) triangle with the law of cosines, and sets θ₄ from the requested pitch.

## License

MIT
