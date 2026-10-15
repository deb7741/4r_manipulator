"""
4R Manipulator control GUI (PyQt5).

  * Forward kinematics: turn the joint dials, see the tip position live.
  * Inverse kinematics: type a target X/Y/Z + tool pitch, press "Go".
  * Presets: the original taught points A–E.
  * Live side and top view of the arm.

Runs on a Raspberry Pi (drives the servos) or on any PC in simulation mode.

    python3 4R_Manipulator_GUI.py            # auto: real servos on a Pi
    python3 4R_Manipulator_GUI.py --simulate # never touch GPIO
"""

from __future__ import annotations

import math
import sys

from PyQt5 import QtCore, QtGui, QtWidgets

import kinematics as kin
from servo import ON_PI, ServoBank

# BCM pin for each joint: base, shoulder, elbow, wrist
SERVO_PINS = (17, 18, 22, 27)
JOINT_NAMES = ("Base", "Shoulder", "Elbow", "Wrist")

# Taught positions (servo angles in degrees) from the original project.
PRESETS = {
    "A": (116, 56, 66, 90),
    "B": (69, 56, 66, 90),
    "C": (11, 56, 66, 90),
    "D": (40, 56, 66, 90),
    "E": (56, 180, 77, 90),
}

DEFAULT_LINKS_CM = (8.0, 10.0, 8.0, 4.0)
SEND_DELAY_MS = 60      # wait for the dial to settle before moving the servo
RELEASE_DELAY_MS = 700  # stop pulses after this long (reduces jitter/heat)


class ArmView(QtWidgets.QWidget):
    """Side view (radial distance vs height) and top view (X vs Y)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(360, 560)
        self.points = []
        self.links = kin.Links(*DEFAULT_LINKS_CM)
        self.joints = (0.0, 0.0, 0.0, 0.0)

    def update_arm(self, joints, links):
        self.joints, self.links = joints, links
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

        yaw = math.radians(self.joints[0])
        planar = [(x * math.cos(yaw) + y * math.sin(yaw), z) for x, y, z in self.points]
        self._draw_view(p, QtCore.QRect(0, 0, self.width(), half), "Side view (r, z)", planar, reach, ground=True)
        top = [(x, y) for x, y, _ in self.points]
        self._draw_view(p, QtCore.QRect(0, half, self.width(), half), "Top view (x, y)", top, reach, ground=False)

    def _draw_view(self, p, rect, title, pts, reach, ground):
        margin = 24
        scale = (min(rect.width(), rect.height()) - 2 * margin) / (2.2 * reach)
        cx = rect.center().x()
        cy = rect.bottom() - margin - 0.1 * reach * scale if ground else rect.center().y()

        def to_px(u, v):
            return QtCore.QPointF(cx + u * scale, cy - v * scale)

        p.setBrush(QtCore.Qt.NoBrush)
        p.setPen(QtGui.QPen(QtGui.QColor("#2a2e38"), 1))
        p.drawRect(rect.adjusted(4, 4, -4, -4))
        p.setPen(QtGui.QColor("#8b93a7"))
        p.drawText(rect.adjusted(12, 8, 0, 0), QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop, title)

        # Reach circle / ground line
        p.setPen(QtGui.QPen(QtGui.QColor("#2f3542"), 1, QtCore.Qt.DashLine))
        if ground:
            p.drawLine(QtCore.QPointF(rect.left() + 8, cy), QtCore.QPointF(rect.right() - 8, cy))
        else:
            r = (reach - self.links.base) * scale
            p.drawEllipse(QtCore.QPointF(cx, cy), r, r)

        colors = ["#6b7280", "#60a5fa", "#a78bfa", "#f472b6"]
        for i in range(len(pts) - 1):
            p.setPen(QtGui.QPen(QtGui.QColor(colors[i % len(colors)]), 6, cap=QtCore.Qt.RoundCap))
            p.drawLine(to_px(*pts[i]), to_px(*pts[i + 1]))
        p.setPen(QtCore.Qt.NoPen)
        for i, pt in enumerate(pts):
            p.setBrush(QtGui.QColor("#facc15" if i == len(pts) - 1 else "#e5e7eb"))
            p.drawEllipse(to_px(*pt), 4.5, 4.5)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, simulate: bool):
        super().__init__()
        self.servos = ServoBank(SERVO_PINS, simulate=simulate)
        mode = "SIMULATION" if self.servos.simulate else "Raspberry Pi"
        self.setWindowTitle(f"4R Manipulator — {mode}")

        self._pending = set()  # joints whose servo needs updating
        self._send_timer = QtCore.QTimer(self, singleShot=True, interval=SEND_DELAY_MS)
        self._send_timer.timeout.connect(self._send_pending)
        self._release_timer = QtCore.QTimer(self, singleShot=True, interval=RELEASE_DELAY_MS)
        self._release_timer.timeout.connect(self.servos.release)

        self.view = ArmView()
        controls = QtWidgets.QVBoxLayout()
        controls.addWidget(self._build_links())
        controls.addWidget(self._build_joints())
        controls.addWidget(self._build_readout())
        controls.addWidget(self._build_ik())
        controls.addWidget(self._build_presets())
        controls.addStretch(1)

        root = QtWidgets.QHBoxLayout()
        root.addWidget(self.view, 1)
        root.addLayout(controls, 1)
        central = QtWidgets.QWidget()
        central.setLayout(root)
        self.setCentralWidget(central)
        self.statusBar().showMessage(
            "Servos live" if not self.servos.simulate else "Simulation mode — servo commands are printed to the console"
        )
        self.refresh()

    # ---------- UI builders ----------
    def _build_links(self):
        box = QtWidgets.QGroupBox("Link lengths (cm)")
        form = QtWidgets.QFormLayout(box)
        self.link_spins = []
        for label, value in zip(("Base column L1", "Upper arm L2", "Forearm L3", "Tool L4"), DEFAULT_LINKS_CM):
            spin = QtWidgets.QDoubleSpinBox(decimals=1, minimum=0.0, maximum=200.0, singleStep=0.5, value=value)
            spin.valueChanged.connect(self.refresh)
            form.addRow(label, spin)
            self.link_spins.append(spin)
        return box

    def _build_joints(self):
        box = QtWidgets.QGroupBox("Forward kinematics — servo angles (°)")
        grid = QtWidgets.QGridLayout(box)
        self.dials, self.dial_labels = [], []
        for i, name in enumerate(JOINT_NAMES):
            dial = QtWidgets.QDial(minimum=0, maximum=180, value=90, notchesVisible=True)
            dial.setWrapping(False)  # wrapping let the dial jump 180° -> 0°
            dial.setFixedSize(90, 90)
            dial.valueChanged.connect(lambda _v, j=i: self._on_dial(j))
            label = QtWidgets.QLabel(alignment=QtCore.Qt.AlignCenter)
            grid.addWidget(QtWidgets.QLabel(name, alignment=QtCore.Qt.AlignCenter), 0, i)
            grid.addWidget(dial, 1, i, alignment=QtCore.Qt.AlignCenter)
            grid.addWidget(label, 2, i)
            self.dials.append(dial)
            self.dial_labels.append(label)
        return box

    def _build_readout(self):
        box = QtWidgets.QGroupBox("End effector")
        row = QtWidgets.QHBoxLayout(box)
        self.readout = {}
        for key in ("X", "Y", "Z", "Pitch"):
            lcd = QtWidgets.QLCDNumber(digitCount=7)
            lcd.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
            lcd.setMinimumHeight(34)
            unit = "°" if key == "Pitch" else "cm"
            row.addWidget(QtWidgets.QLabel(f"{key} ({unit})"))
            row.addWidget(lcd)
            self.readout[key] = lcd
        return box

    def _build_ik(self):
        box = QtWidgets.QGroupBox("Inverse kinematics — go to a point")
        grid = QtWidgets.QGridLayout(box)
        self.ik_spins = {}
        for col, (key, lo, hi, val) in enumerate(
            (("X", -200, 200, 12.0), ("Y", -200, 200, 8.0), ("Z", -200, 200, 16.0), ("Pitch", -180, 180, -15.0))
        ):
            spin = QtWidgets.QDoubleSpinBox(decimals=1, minimum=lo, maximum=hi, singleStep=0.5, value=val)
            grid.addWidget(QtWidgets.QLabel(key), 0, col)
            grid.addWidget(spin, 1, col)
            self.ik_spins[key] = spin
        go = QtWidgets.QPushButton("Go")
        go.clicked.connect(self._on_ik)
        here = QtWidgets.QPushButton("Use current")
        here.setToolTip("Copy the current tip position into the target fields")
        here.clicked.connect(self._copy_current_pose)
        grid.addWidget(go, 1, 4)
        grid.addWidget(here, 0, 4)
        self.ik_status = QtWidgets.QLabel()
        grid.addWidget(self.ik_status, 2, 0, 1, 5)
        return box

    def _build_presets(self):
        box = QtWidgets.QGroupBox("Taught points")
        row = QtWidgets.QHBoxLayout(box)
        for key, angles in PRESETS.items():
            btn = QtWidgets.QPushButton(f"Point {key}")
            btn.setToolTip(" / ".join(f"{n} {a}°" for n, a in zip(JOINT_NAMES, angles)))
            btn.clicked.connect(lambda _c, a=angles: self.set_servo_angles(a))
            row.addWidget(btn)
        return box

    # ---------- behaviour ----------
    def links(self) -> kin.Links:
        return kin.Links(*(s.value() for s in self.link_spins))

    def servo_angles(self):
        return tuple(float(d.value()) for d in self.dials)

    def set_servo_angles(self, angles):
        """Move all dials at once (one servo update, not four)."""
        for i, (dial, angle) in enumerate(zip(self.dials, angles)):
            dial.blockSignals(True)
            dial.setValue(int(round(angle)))
            dial.blockSignals(False)
            self._pending.add(i)
        self._send_timer.start()
        self.refresh()

    def _on_dial(self, joint: int):
        self._pending.add(joint)
        self._send_timer.start()  # restarts while the dial is still moving
        self.refresh()

    def _send_pending(self):
        for j in sorted(self._pending):
            self.servos.move(SERVO_PINS[j], self.dials[j].value())
        self._pending.clear()
        self._release_timer.start()

    def refresh(self):
        servo = self.servo_angles()
        for label, angle in zip(self.dial_labels, servo):
            label.setText(f"{angle:.0f}°")
        links = self.links()
        pose = kin.forward(servo, links)
        for key, value in (("X", pose.x), ("Y", pose.y), ("Z", pose.z), ("Pitch", pose.pitch)):
            self.readout[key].display(f"{value:.1f}")
        self.view.update_arm(kin.servo_to_joint(servo), links)

    def _copy_current_pose(self):
        pose = kin.forward(self.servo_angles(), self.links())
        for key, value in (("X", pose.x), ("Y", pose.y), ("Z", pose.z), ("Pitch", pose.pitch)):
            self.ik_spins[key].setValue(value)

    def _on_ik(self):
        target = kin.Pose(*(self.ik_spins[k].value() for k in ("X", "Y", "Z", "Pitch")))
        solution = kin.inverse(target, self.links())
        if solution is None:
            self.ik_status.setText("⚠ Out of reach, or needs a servo outside 0–180°. Try another pitch.")
            self.ik_status.setStyleSheet("color: #dc2626")
            return
        self.ik_status.setText("Servos: " + ", ".join(f"{n} {a:.0f}°" for n, a in zip(JOINT_NAMES, solution)))
        self.ik_status.setStyleSheet("")
        self.set_servo_angles(solution)

    def closeEvent(self, event):
        self.servos.close()
        super().closeEvent(event)


def main():
    simulate = "--simulate" in sys.argv or not ON_PI
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow(simulate=simulate)
    window.resize(1100, 760)
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
