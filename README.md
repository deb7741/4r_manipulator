# 4R Manipulator

An educational 4R spherical manipulator for learning **forward and inverse kinematics**, driven by a Raspberry Pi and a PyQt5 GUI.

![joints](https://img.shields.io/badge/joints-4%20servos-blue) ![python](https://img.shields.io/badge/python-3.8%2B-green)

## Features

- **Jog** each joint with its dial or by typing an exact angle.
- **Inverse kinematics** — type a target X/Y/Z and tool pitch, press **Go**. All solutions are tried (elbow up/down, reaching over the top) and the one needing the least movement within the joint limits is used.
- **Smooth motion** — every move is speed-limited and synchronised so all joints start and finish together, with eased acceleration. No more slamming to a preset.
- **Teach & replay** — save the current pose as a named point, drag to reorder, and **Play sequence** (optionally looping). Points are stored in `points.json`.
- **STOP (Esc)**, **Home (H)** and **Release servos** buttons.
- **Live side and top views** of the arm, with the IK target marked.
- **Simulation mode** — runs on any PC without a Pi.

## Hardware

| Joint | GPIO (BCM) | Physical pin |
|---|---|---|
| Base (yaw) | 17 | 11 |
| Shoulder | 18 | 12 |
| Elbow | 22 | 15 |
| Wrist | 27 | 13 |

Power the servos from a separate 5–6 V supply that can deliver the stall current of all four motors (≈1 A each for MG996R-class servos), and **connect its ground to the Pi's ground**. Never power servos from the Pi's 5 V pin.

## Install

On the Raspberry Pi:

```bash
sudo apt install python3-pyqt5 pigpio python3-pigpio
sudo systemctl enable --now pigpiod      # jitter-free servo pulses
git clone https://github.com/deb7741/4r_manipulator && cd 4r_manipulator
python3 4R_Manipulator_GUI.py
```

On a PC (simulation):

```bash
pip install PyQt5 pytest
python3 4R_Manipulator_GUI.py --simulate
```

The driver is picked automatically: **pigpio** (best) → **RPi.GPIO** (works, but servos twitch, so they are released after each move) → **simulation**. Force one with `--backend pigpio|rpi|sim`. Add `-v` to log every pulse.

> At start-up the arm moves straight to the home pose (the servos' real position is unknown at power-up), so keep the work area clear.

## Calibration — `config.json`

All settings live in `config.json` (edit by hand; the GUI's **Save settings** writes link lengths and speed):

| Key | Meaning |
|---|---|
| `pins_bcm` | GPIO pin per joint |
| `pulse_us` | pulse width (µs) at 0° and 180° for each servo. Defaults `[400, 2400]` match the original code's 2–12 % duty cycle. Many SG90/MG90S servos want about `[500, 2400]` |
| `offsets_deg` | servo angle at which each joint is "zero" (base along +X, shoulder horizontal, elbow and wrist straight) |
| `directions` | `1`, or `-1` if a servo is mounted reversed |
| `min_deg` / `max_deg` | soft limits — keep the arm off the table and its own frame. IK and jogging never exceed them |
| `links_cm` | link lengths L1 (base column) … L4 (tool) |
| `home_deg` | Home pose |
| `max_speed_deg_s` | fastest joint speed |
| `release_after_move` | `auto` (release only on RPi.GPIO), `always` or `never` |
| `backend` | `auto`, `pigpio`, `rpi` or `sim` |

**Quick calibration**

1. Type `90` into Shoulder, Elbow and Wrist. The arm should point straight up. If a joint is off, adjust its entry in `offsets_deg` until it is.
2. Increase the Shoulder angle a little: the arm should tilt *forward* (towards +X when the base is at 0°). If it tilts back, set that joint's `directions` entry to `-1` and its offset to `180 - offset`.
3. Jog each joint slowly to where it would hit something and set `min_deg` / `max_deg` a few degrees inside that.
4. Measure your links pivot-to-pivot and enter them under **Settings**, then **Save settings**.
5. Check: put the tip on a ruler mark, press **Use current** and compare X/Y/Z.

## Files

| File | Purpose |
|---|---|
| `4R_Manipulator_GUI.py` | PyQt5 interface |
| `kinematics.py` | Forward / inverse kinematics (pure Python) |
| `motion.py` | Smooth, synchronised move planner |
| `servo.py` | pigpio / RPi.GPIO / simulation servo drivers |
| `config.py`, `config.json` | Settings and calibration |
| `points.json` | Taught points |
| `test_*.py` | Tests — run `python3 -m pytest -q` (25 tests, no hardware needed) |

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
