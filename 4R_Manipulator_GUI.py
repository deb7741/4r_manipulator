"""
4R Manipulator control GUI (PyQt5).

  * Jog each joint with its dial or by typing an angle.
  * Inverse kinematics: type a target X/Y/Z + tool pitch and press Go.
  * Teach & replay: save the current pose as a named point, play a sequence.
  * Smooth, speed-limited, synchronised motion; Home and STOP (Esc).
  * Live side and top view of the arm.

Runs on a Raspberry Pi (drives the servos) or on any PC in simulation mode.

    python3 4R_Manipulator_GUI.py                  # best available backend
    python3 4R_Manipulator_GUI.py --simulate       # never touch GPIO
    python3 4R_Manipulator_GUI.py --backend rpi    # force RPi.GPIO
"""

from __future__ import annotations

import argparse
import logging
import math
import sys
import time

from PyQt5 import QtCore, QtGui, QtWidgets

import config as cfgmod
import kinematics as kin
from config import JOINT_NAMES
from motion import Move
from servo import open_servos

TICK_MS = 20            # motion update period (50 Hz = one servo frame)
RELEASE_DELAY_MS = 600  # how long to hold before releasing (jittery backends)
SEQUENCE_DWELL_MS = 600 # pause at each point while playing a sequence


class ArmView(QtWidgets.QWidget):
    """Side view (radial distance vs height) and top view (X vs Y)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(340, 520)
        self.points = []
        self.links = kin.Links(8, 10, 8, 4)
        self.yaw = 0.0
        self.target = None  # optional (x, y, z) marker

    def update_arm(self, joints, links, target=None):
        self.yaw, self.links, self.target = joints[0], links, target
        self.points = kin.joint_positions(joints, links)
        self.update()

    def paintEvent(self, _event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor("#14161c"))
        if not self.points:
            return
        half = self.height() // 2
        reach = max(1e-6, self.links.base + self.links.upper + self.links.fore + self.links.tool)
        yaw = math.radians(self.yaw)
        planar = [(x * math.cos(yaw) + y * math.sin(yaw), z) for x, y, z in self.points]
        tgt_side = tgt_top = None
        if self.target:
            tx, ty, tz = self.target
            tgt_side = (tx * math.cos(yaw) + ty * math.sin(yaw), tz)
            tgt_top = (tx, ty)
        self._draw(p, QtCore.QRect(0, 0, self.width(), half), "Side view (r, z)", planar, reach, True, tgt_side)
        top = [(x, y) for x, y, _ in self.points]
        self._draw(p, QtCore.QRect(0, half, self.width(), half), "Top view (x, y)", top, reach, False, tgt_top)

    def _draw(self, p, rect, title, pts, reach, ground, target):
        margin = 24
        scale = (min(rect.width(), rect.height()) - 2 * margin) / (2.2 * reach)
        cx = rect.center().x()
        cy = rect.bottom() - margin - 0.1 * reach * scale if ground else rect.center().y()

        def px(u, v):
            return QtCore.QPointF(cx + u * scale, cy - v * scale)

        p.setBrush(QtCore.Qt.NoBrush)
        p.setPen(QtGui.QPen(QtGui.QColor("#2a2e38"), 1))
        p.drawRect(rect.adjusted(4, 4, -4, -4))
        p.setPen(QtGui.QColor("#8b93a7"))
        p.drawText(rect.adjusted(12, 8, 0, 0), QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop, title)

        p.setPen(QtGui.QPen(QtGui.QColor("#2f3542"), 1, QtCore.Qt.DashLine))
        if ground:
            p.drawLine(QtCore.QPointF(rect.left() + 8, cy), QtCore.QPointF(rect.right() - 8, cy))
        else:
            r = (reach - self.links.base) * scale
            p.drawEllipse(QtCore.QPointF(cx, cy), r, r)

        if target is not None:
            p.setPen(QtGui.QPen(QtGui.QColor("#22c55e"), 1.5))
            c = px(*target)
            p.drawLine(c + QtCore.QPointF(-7, 0), c + QtCore.QPointF(7, 0))
            p.drawLine(c + QtCore.QPointF(0, -7), c + QtCore.QPointF(0, 7))

        colors = ["#6b7280", "#60a5fa", "#a78bfa", "#f472b6"]
        for i in range(len(pts) - 1):
            p.setPen(QtGui.QPen(QtGui.QColor(colors[i % 4]), 6, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
            p.drawLine(px(*pts[i]), px(*pts[i + 1]))
        p.setPen(QtCore.Qt.NoPen)
        for i, pt in enumerate(pts):
            p.setBrush(QtGui.QColor("#facc15" if i == len(pts) - 1 else "#e5e7eb"))
            p.drawEllipse(px(*pt), 4.5, 4.5)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, cfg: cfgmod.ArmConfig, backend: str):
        super().__init__()
        self.cfg = cfg
        self.cal = cfg.calibration
        self.points = cfgmod.load_points()
        self.servos = open_servos(cfg.pins, cfg.pulse_us, backend)

        self.cmd = tuple(cfg.home)     # angles currently commanded to the servos
        self.target = tuple(cfg.home)  # where the arm is heading
        self.move: Move | None = None
        self.move_t0 = 0.0
        self.ik_marker = None
        self.sequence: list[str] = []

        self.tick = QtCore.QTimer(self, interval=TICK_MS)
        self.tick.timeout.connect(self._on_tick)
        self.release_timer = QtCore.QTimer(self, singleShot=True, interval=RELEASE_DELAY_MS)
        self.release_timer.timeout.connect(self.servos.release)
        self.dwell_timer = QtCore.QTimer(self, singleShot=True, interval=SEQUENCE_DWELL_MS)
        self.dwell_timer.timeout.connect(self._next_in_sequence)

        self.view = ArmView()
        left = QtWidgets.QVBoxLayout()
        left.addWidget(self.view, 1)
        left.addLayout(self._build_actions())

        right = QtWidgets.QVBoxLayout()
        right.addWidget(self._build_joints())
        right.addWidget(self._build_readout())
        right.addWidget(self._build_ik())
        right.addWidget(self._build_points())
        right.addWidget(self._build_settings())
        right.addStretch(1)

        root = QtWidgets.QHBoxLayout()
        root.addLayout(left, 5)
        root.addLayout(right, 6)
        central = QtWidgets.QWidget()
        central.setLayout(root)
        self.setCentralWidget(central)

        QtWidgets.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Escape), self, activated=self.stop)
        QtWidgets.QShortcut(QtGui.QKeySequence("H"), self, activated=self.go_home)

        name = self.servos.name
        self.setWindowTitle(f"4R Manipulator — {name}")
        note = {
            "pigpio": "pigpio: hardware-timed pulses (jitter-free)",
            "RPi.GPIO": "RPi.GPIO: software PWM — install pigpio for jitter-free servos",
            "sim": "Simulation — no servos connected",
        }[name]
        self.statusBar().showMessage(f"{note}   •   Esc = STOP, H = Home")

        # Start at home. The servos' real position is unknown at power-up, so
        # this first command is a direct jump — keep the arm clear.
        self.servos.write_all(self.cmd)
        self._schedule_release()
        self._sync_dials(self.target)
        self._refresh()

    # ---------------- UI builders ----------------
    def _build_actions(self):
        row = QtWidgets.QHBoxLayout()
        self.stop_btn = QtWidgets.QPushButton("STOP  (Esc)")
        self.stop_btn.setStyleSheet("background:#dc2626;color:white;font-weight:bold;padding:8px;")
        self.stop_btn.clicked.connect(self.stop)
        home = QtWidgets.QPushButton("Home")
        home.clicked.connect(self.go_home)
        limp = QtWidgets.QPushButton("Release servos")
        limp.setToolTip("Stop sending pulses — the arm goes limp")
        limp.clicked.connect(self.release_now)
        for w in (self.stop_btn, home, limp):
            row.addWidget(w)
        return row

    def _build_joints(self):
        box = QtWidgets.QGroupBox("Joints — servo angles (°)")
        grid = QtWidgets.QGridLayout(box)
        self.dials, self.spins = [], []
        for i, name in enumerate(JOINT_NAMES):
            lo, hi = int(self.cal.min_angles[i]), int(self.cal.max_angles[i])
            dial = QtWidgets.QDial()
            dial.setRange(lo, hi)
            dial.setNotchesVisible(True)
            dial.setWrapping(False)
            dial.setFixedSize(84, 84)
            dial.valueChanged.connect(lambda v, j=i: self._on_joint_input(j, v))
            spin = QtWidgets.QSpinBox()
            spin.setRange(lo, hi)
            spin.setSuffix("°")
            spin.setKeyboardTracking(False)  # only react when Enter is pressed / focus leaves
            spin.valueChanged.connect(lambda v, j=i: self._on_joint_input(j, v))
            grid.addWidget(QtWidgets.QLabel(name, alignment=QtCore.Qt.AlignCenter), 0, i)
            grid.addWidget(dial, 1, i, alignment=QtCore.Qt.AlignCenter)
            grid.addWidget(spin, 2, i)
            self.dials.append(dial)
            self.spins.append(spin)
        return box

    def _build_readout(self):
        box = QtWidgets.QGroupBox("End effector (live)")
        row = QtWidgets.QHBoxLayout(box)
        self.readout = {}
        for key in ("X", "Y", "Z", "Pitch"):
            lab = QtWidgets.QLabel()
            lab.setStyleSheet("font: 600 13pt monospace;")
            unit = "°" if key == "Pitch" else " cm"
            row.addWidget(QtWidgets.QLabel(f"{key}:"))
            row.addWidget(lab)
            self.readout[key] = (lab, unit)
        return box

    def _build_ik(self):
        box = QtWidgets.QGroupBox("Inverse kinematics — go to a point")
        grid = QtWidgets.QGridLayout(box)
        self.ik_spins = {}
        defaults = {"X": 12.0, "Y": 8.0, "Z": 16.0, "Pitch": -15.0}
        for col, key in enumerate(("X", "Y", "Z", "Pitch")):
            spin = QtWidgets.QDoubleSpinBox()
            spin.setDecimals(1)
            spin.setRange(-180 if key == "Pitch" else -200, 180 if key == "Pitch" else 200)
            spin.setSingleStep(0.5)
            spin.setValue(defaults[key])
            grid.addWidget(QtWidgets.QLabel(key + (" (°)" if key == "Pitch" else " (cm)")), 0, col)
            grid.addWidget(spin, 1, col)
            self.ik_spins[key] = spin
        here = QtWidgets.QPushButton("Use current")
        here.clicked.connect(self._copy_current_pose)
        go = QtWidgets.QPushButton("Go")
        go.setDefault(True)
        go.clicked.connect(self._on_ik)
        grid.addWidget(here, 0, 4)
        grid.addWidget(go, 1, 4)
        self.ik_status = QtWidgets.QLabel()
        self.ik_status.setWordWrap(True)
        grid.addWidget(self.ik_status, 2, 0, 1, 5)
        return box

    def _build_points(self):
        box = QtWidgets.QGroupBox("Taught points")
        lay = QtWidgets.QHBoxLayout(box)
        self.point_list = QtWidgets.QListWidget()
        self.point_list.setDragDropMode(QtWidgets.QAbstractItemView.InternalMove)  # reorder = sequence order
        self.point_list.itemDoubleClicked.connect(lambda item: self.go_to_point(item.text()))
        self.point_list.model().rowsMoved.connect(self._save_points_from_list)
        self._reload_point_list()
        lay.addWidget(self.point_list, 1)

        col = QtWidgets.QVBoxLayout()
        buttons = [
            ("Go", lambda: self._selected_point() and self.go_to_point(self._selected_point())),
            ("Save current…", self._save_current_point),
            ("Delete", self._delete_point),
            ("▶ Play sequence", self.play_sequence),
        ]
        for text, fn in buttons:
            b = QtWidgets.QPushButton(text)
            b.clicked.connect(fn)
            col.addWidget(b)
        self.loop_box = QtWidgets.QCheckBox("Loop")
        col.addWidget(self.loop_box)
        col.addStretch(1)
        lay.addLayout(col)
        return box

    def _build_settings(self):
        box = QtWidgets.QGroupBox("Settings")
        form = QtWidgets.QFormLayout(box)
        self.speed = QtWidgets.QSpinBox()
        self.speed.setRange(5, 360)
        self.speed.setSuffix(" °/s")
        self.speed.setValue(int(self.cfg.max_speed))
        form.addRow("Max joint speed", self.speed)
        links_row = QtWidgets.QHBoxLayout()
        self.link_spins = []
        for label, value in zip(("L1", "L2", "L3", "L4"), (self.cfg.links.base, self.cfg.links.upper, self.cfg.links.fore, self.cfg.links.tool)):
            spin = QtWidgets.QDoubleSpinBox()
            spin.setDecimals(1)
            spin.setRange(0, 200)
            spin.setSingleStep(0.5)
            spin.setValue(value)
            spin.setPrefix(f"{label} ")
            spin.valueChanged.connect(self._refresh)
            links_row.addWidget(spin)
            self.link_spins.append(spin)
        form.addRow("Links (cm)", links_row)
        save = QtWidgets.QPushButton("Save settings to config.json")
        save.clicked.connect(self._save_settings)
        form.addRow(save)
        return box

    # ---------------- motion ----------------
    def links(self) -> kin.Links:
        return kin.Links(*(s.value() for s in self.link_spins))

    def move_to(self, angles, from_user_input=False):
        """Plan a smooth move from wherever the arm is now to ``angles``."""
        self.target = self.cal.clamp(angles)
        self.move = Move.plan(self.cmd, self.target, self.speed.value())
        self.move_t0 = time.monotonic()
        self.release_timer.stop()
        if not self.tick.isActive():
            self.tick.start()
        if not from_user_input:
            self._sync_dials(self.target)

    def _on_tick(self):
        if self.move is None:
            self.tick.stop()
            return
        t = time.monotonic() - self.move_t0
        self.cmd = self.move.sample(t)
        self.servos.write_all(self.cmd)
        self._refresh()
        if self.move.done(t):
            self.move = None
            self.tick.stop()
            self._schedule_release()
            if self.sequence:
                self.dwell_timer.start()

    def stop(self):
        """Freeze immediately where the arm is (also cancels a sequence)."""
        self.sequence = []
        self.dwell_timer.stop()
        self.move = None
        self.tick.stop()
        self.target = self.cmd
        self._sync_dials(self.cmd)
        self.statusBar().showMessage("Stopped", 2000)

    def go_home(self):
        self.sequence = []
        self.move_to(self.cfg.home)

    def release_now(self):
        self.stop()
        self.servos.release()

    def _schedule_release(self):
        policy = self.cfg.release_after_move
        if policy == "always" or (policy == "auto" and not self.servos.jitter_free):
            self.release_timer.start()

    def _on_joint_input(self, joint, value):
        if self.target[joint] == value:
            return
        self.sequence = []  # manual jog cancels a running sequence
        self.dwell_timer.stop()
        new = list(self.target)
        new[joint] = float(value)
        self._sync_dials(new)
        self.move_to(new, from_user_input=True)

    def _sync_dials(self, angles):
        for dial, spin, a in zip(self.dials, self.spins, angles):
            for w in (dial, spin):
                w.blockSignals(True)
                w.setValue(int(round(a)))
                w.blockSignals(False)

    def _refresh(self):
        links = self.links()
        pose = kin.forward(self.cmd, links, self.cal)
        for key, value in (("X", pose.x), ("Y", pose.y), ("Z", pose.z), ("Pitch", pose.pitch)):
            lab, unit = self.readout[key]
            lab.setText(f"{value:7.1f}{unit}")
        self.view.update_arm(kin.servo_to_joint(self.cmd, self.cal), links, self.ik_marker)

    # ---------------- IK ----------------
    def _copy_current_pose(self):
        pose = kin.forward(self.cmd, self.links(), self.cal)
        for key, value in (("X", pose.x), ("Y", pose.y), ("Z", pose.z), ("Pitch", pose.pitch)):
            self.ik_spins[key].setValue(value)

    def _on_ik(self):
        target = kin.Pose(*(self.ik_spins[k].value() for k in ("X", "Y", "Z", "Pitch")))
        self.ik_marker = (target.x, target.y, target.z)
        solution = kin.inverse(target, self.links(), self.cal, prefer=self.cmd)
        if solution is None:
            self.ik_status.setText("⚠ Unreachable with this pitch or outside the joint limits — try another pitch.")
            self.ik_status.setStyleSheet("color:#dc2626")
            self._refresh()
            return
        self.ik_status.setText("→ " + ", ".join(f"{n} {a:.0f}°" for n, a in zip(JOINT_NAMES, solution)))
        self.ik_status.setStyleSheet("")
        self.sequence = []
        self.move_to(solution)

    # ---------------- taught points ----------------
    def _reload_point_list(self):
        self.point_list.clear()
        for name, angles in self.points.items():
            item = QtWidgets.QListWidgetItem(name)
            item.setToolTip(" / ".join(f"{n} {a:.0f}°" for n, a in zip(JOINT_NAMES, angles)))
            self.point_list.addItem(item)

    def _selected_point(self):
        item = self.point_list.currentItem()
        return item.text() if item else None

    def go_to_point(self, name):
        if name in self.points:
            self.sequence = []
            self.move_to(self.points[name])

    def _save_current_point(self):
        name, ok = QtWidgets.QInputDialog.getText(self, "Save point", "Name for the current pose:", text=f"P{len(self.points) + 1}")
        name = name.strip()
        if ok and name:
            self.points[name] = [round(a, 1) for a in self.cmd]
            cfgmod.save_points(self.points)
            self._reload_point_list()

    def _delete_point(self):
        name = self._selected_point()
        if name and name in self.points:
            del self.points[name]
            cfgmod.save_points(self.points)
            self._reload_point_list()

    def _save_points_from_list(self, *_):
        order = [self.point_list.item(i).text() for i in range(self.point_list.count())]
        self.points = {name: self.points[name] for name in order if name in self.points}
        cfgmod.save_points(self.points)

    def play_sequence(self):
        order = [self.point_list.item(i).text() for i in range(self.point_list.count())]
        if not order:
            return
        self.sequence = order[:]
        self._next_in_sequence()

    def _next_in_sequence(self):
        if not self.sequence:
            return
        name = self.sequence.pop(0)
        if not self.sequence and self.loop_box.isChecked():
            self.sequence = [self.point_list.item(i).text() for i in range(self.point_list.count())]
        self.statusBar().showMessage(f"Sequence: {name}", 1500)
        matches = self.point_list.findItems(name, QtCore.Qt.MatchExactly)
        if matches:
            self.point_list.setCurrentItem(matches[0])
        target = self.points.get(name)
        if target is None:
            self._next_in_sequence()
            return
        self.move_to(target)

    # ---------------- settings ----------------
    def _save_settings(self):
        self.cfg.links = self.links()
        self.cfg.max_speed = float(self.speed.value())
        self.cfg.raw["max_speed_deg_s"] = self.speed.value()
        cfgmod.save_config(self.cfg)
        self.statusBar().showMessage(f"Saved {cfgmod.CONFIG_PATH.name}", 3000)

    def closeEvent(self, event):
        self.stop()
        self.servos.close()
        super().closeEvent(event)


def main(argv=None):
    parser = argparse.ArgumentParser(description="4R manipulator control GUI")
    parser.add_argument("--simulate", action="store_true", help="never touch GPIO")
    parser.add_argument("--backend", choices=["auto", "pigpio", "rpi", "sim"], help="override config.json")
    parser.add_argument("-v", "--verbose", action="store_true", help="log every servo pulse")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, format="%(message)s")

    cfg = cfgmod.load_config()
    backend = "sim" if args.simulate else (args.backend or cfg.backend)

    app = QtWidgets.QApplication(sys.argv[:1])
    window = MainWindow(cfg, backend)
    window.resize(1180, 820)
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
