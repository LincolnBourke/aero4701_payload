#!/usr/bin/env python3
"""
Stewart Platform Digital Twin
==============================
Subscribes to SERVO_TARGETS over LCM, solves forward kinematics,
and renders a live 3D visualisation of the Stewart platform.

Dependencies:
    pip install lcm numpy matplotlib scipy

LCM message definition (servo_targets_t) must be importable.
If you don't have the Python LCM bindings generated yet, run:
    lcm-gen -p servo_targets_t.lcm
and place the generated file alongside this script, or adjust
PYTHONPATH to include your lcm message directory.
"""

import sys
import time
import threading
import argparse
import struct
import math
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.optimize import fsolve
from matplotlib.patches import FancyBboxPatch
import matplotlib.patches as mpatches

# ─────────────────────────────────────────────────────────────────────────────
# Stewart Platform Geometry  (mirrored exactly from stewartPlatform.hpp / .cpp)
# ─────────────────────────────────────────────────────────────────────────────
PLATFORM_Z_OFFSET      =  10.0      # mm  – base origin → bottom of platform
BASE_Z_OFFSET          =   9.4248   # mm  – base origin → top of servo brace
LOWER_ARM_LENGTH       =  18.5      # mm
UPPER_ARM_LENGTH       =  27.0      # mm
BASE_ANCHOR_RADIUS     =  32.7015   # mm
PLATFORM_ANCHOR_RADIUS =  33.023    # mm
BASE_ANCHOR_ANG_OFF    =   0.1581   # rad
PLATFORM_ANCHOR_ANG_OFF=   0.2877   # rad
NUM_SERVOS             =   6

# Servo angular positions (orientation of horn rotation axis, viewed from above)
SERVO_ANGULAR_POS = [
    3.0/2.0 * math.pi,   # 0
    1.0/2.0 * math.pi,   # 1
    1.0/6.0 * math.pi,   # 2
    7.0/6.0 * math.pi,   # 3
    5.0/6.0 * math.pi,   # 4
   11.0/6.0 * math.pi,   # 5
]


def build_anchors():
    """Return base_anchors and platform_anchors as (6,3) arrays."""
    base    = np.zeros((6, 3))
    platform = np.zeros((6, 3))
    for i in range(0, NUM_SERVOS, 2):
        centre_ang      = math.pi / 3 * i
        lower_base_ang  = centre_ang - BASE_ANCHOR_ANG_OFF
        upper_base_ang  = centre_ang + BASE_ANCHOR_ANG_OFF
        lower_plat_ang  = centre_ang - PLATFORM_ANCHOR_ANG_OFF
        upper_plat_ang  = centre_ang + PLATFORM_ANCHOR_ANG_OFF

        base[i]     = BASE_ANCHOR_RADIUS    * np.array([math.cos(lower_base_ang), math.sin(lower_base_ang), 0])
        base[i+1]   = BASE_ANCHOR_RADIUS    * np.array([math.cos(upper_base_ang), math.sin(upper_base_ang), 0])
        platform[i]   = PLATFORM_ANCHOR_RADIUS * np.array([math.cos(lower_plat_ang), math.sin(lower_plat_ang), 0])
        platform[i+1] = PLATFORM_ANCHOR_RADIUS * np.array([math.cos(upper_plat_ang), math.sin(upper_plat_ang), 0])
    return base, platform


BASE_ANCHORS, PLATFORM_ANCHORS = build_anchors()


# ─────────────────────────────────────────────────────────────────────────────
# Forward Kinematics  (Newton-Raphson, matching computePlatformPose() in C++)
# ─────────────────────────────────────────────────────────────────────────────

def rotation_matrix(roll, pitch, yaw):
    """ZYX Euler → rotation matrix R (same convention as C++ code)."""
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    return np.array([
        [cr*cp,  cr*sp*sy - sr*cy,  cr*sp*cy + sr*sy],
        [sr*cp,  sr*sp*sy + cr*cy,  sr*sp*cy - cr*sy],
        [  -sp,          cp*sy,             cp*cy   ],
    ])


def servo_horn_vector(servo_idx, servo_angle_rad):
    """
    3-D vector along servo horn (from motor axis to ball joint).
    Matches bottom_arm computation in computeAngularSkew().
    """
    ang_above = SERVO_ANGULAR_POS[servo_idx]
    c_a, s_a = math.cos(ang_above), math.sin(ang_above)
    c_h, s_h = math.cos(servo_angle_rad), math.sin(servo_angle_rad)
    return LOWER_ARM_LENGTH * np.array([c_a * c_h, s_a * c_h, s_h])


def fk_residuals(pose_vec, servo_angles_rad):
    """
    Residuals for the Newton-Raphson system.
    pose_vec = [x, y, z, roll, pitch, yaw]
    Each equation: ||(pos - base_i) + R*plat_i - h_i||^2 - upper_arm^2 = 0
    """
    x, y, z, roll, pitch, yaw = pose_vec
    R = rotation_matrix(roll, pitch, yaw)
    pos = np.array([x, y, z])
    residuals = np.zeros(6)
    for i in range(NUM_SERVOS):
        h_i = servo_horn_vector(i, servo_angles_rad[i])
        p_i = R @ PLATFORM_ANCHORS[i]
        diff = (pos - BASE_ANCHORS[i]) + p_i - h_i
        residuals[i] = np.dot(diff, diff) - UPPER_ARM_LENGTH**2
    return residuals


def solve_forward_kinematics(servo_angles_rad, initial_guess=None):
    """
    Solve FK for the platform pose given 6 servo angles.
    Returns (position [3], euler_angles [3]) or (None, None) on failure.

    initial_guess: [x, y, z, roll, pitch, yaw] – defaults to neutral pose.
    """
    if initial_guess is None:
        z0 = BASE_Z_OFFSET + PLATFORM_Z_OFFSET + 10.0  # neutral height
        initial_guess = [0.0, 0.0, z0, 0.0, 0.0, 0.0]

    try:
        sol, info, ier, msg = fsolve(
            fk_residuals, initial_guess,
            args=(servo_angles_rad,),
            full_output=True,
            xtol=1e-6,
            ftol=1e-6,
        )
        residual_norm = np.max(np.abs(info['fvec']))
        if ier == 1 and residual_norm < 1e-3:
            return np.array(sol[:3]), np.array(sol[3:])
    except Exception:
        pass
    return None, None


# ─────────────────────────────────────────────────────────────────────────────
# Geometry helpers for 3-D rendering
# ─────────────────────────────────────────────────────────────────────────────

def get_platform_anchor_world(pos, R, i):
    """World-frame position of platform anchor point i."""
    return pos + R @ PLATFORM_ANCHORS[i]


def get_elbow_position(base_anchor, servo_angle_rad, servo_idx):
    """World-frame position of the servo horn tip (elbow joint)."""
    h = servo_horn_vector(servo_idx, servo_angle_rad)
    return base_anchor + h


def euler_to_quat(roll, pitch, yaw):
    """ZYX Euler angles → quaternion [w, x, y, z]."""
    cr, sr = math.cos(roll/2),  math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2),   math.sin(yaw/2)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return np.array([w, x, y, z])


def polygon_vertices(anchors_world):
    """Return vertices of the hexagonal platform/base polygon."""
    return anchors_world  # Already in order 0-5


# ─────────────────────────────────────────────────────────────────────────────
# LCM subscriber  (runs in a background thread)
# ─────────────────────────────────────────────────────────────────────────────

class ServoTargetsMessage:
    """
    Hand-rolled decoder for payload_messages::servo_targets_t.
    Format: fingerprint (8 bytes int64) + 6 floats (6×4 bytes).
    Adjust if your LCM type hash or field layout differs.
    """
    CHANNEL = "SERVO_TARGETS"

    @staticmethod
    def decode(data):
        # Skip the 8-byte LCM fingerprint, then read 6 floats
        if len(data) < 8 + NUM_SERVOS * 4:
            return None
        angles = struct.unpack_from(f'>{NUM_SERVOS}f', data, 8)
        return list(angles)


class LCMSubscriber:
    def __init__(self):
        self.servo_angles = [0.0] * NUM_SERVOS
        self.last_update = None
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._lc = None
        self.connected = False

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        try:
            import lcm
            self._lc = lcm.LCM()

            # Try to import generated bindings first; fall back to hand decoder
            try:
                from payload_messages import servo_targets_t
                def handler(channel, data):
                    msg = servo_targets_t.decode(data)
                    with self._lock:
                        self.servo_angles = list(msg.angles)
                        self.last_update = time.time()
            except ImportError:
                def handler(channel, data):
                    angles = ServoTargetsMessage.decode(data)
                    if angles:
                        with self._lock:
                            self.servo_angles = angles
                            self.last_update = time.time()

            self._lc.subscribe(ServoTargetsMessage.CHANNEL, handler)
            self.connected = True
            print(f"[LCM] Subscribed to {ServoTargetsMessage.CHANNEL}")

            while self._running:
                self._lc.handle_timeout(50)  # ms

        except Exception as e:
            print(f"[LCM] Error: {e}")
            print("[LCM] Running in demo mode with animated servo angles.")
            self._run_demo()

    def _run_demo(self):
        """Animate a smooth sweep so the visualiser can be tested without LCM."""
        t0 = time.time()
        while self._running:
            t = time.time() - t0
            angles = [
                0.3 * math.sin(t * 0.5 + i * math.pi / 3)
                for i in range(NUM_SERVOS)
            ]
            with self._lock:
                self.servo_angles = angles
                self.last_update = time.time()
            time.sleep(0.05)

    def get_angles(self):
        with self._lock:
            return list(self.servo_angles), self.last_update

    def stop(self):
        self._running = False


# ─────────────────────────────────────────────────────────────────────────────
# Visualiser
# ─────────────────────────────────────────────────────────────────────────────

# Colour palette – dark industrial / telemetry aesthetic
C_BG          = "#0d0f14"
C_PANEL       = "#13161d"
C_GRID        = "#1e2330"
C_BASE        = "#2a7de1"      # blue  – base ring
C_PLATFORM    = "#e8c840"      # amber – platform ring
C_LOWER_ARM   = "#5ad4a0"      # teal  – servo horns
C_UPPER_ARM   = "#e07850"      # coral – upper rods
C_ANCHOR_BASE = "#2a7de1"
C_ANCHOR_PLAT = "#e8c840"
C_ELBOW       = "#5ad4a0"
C_TEXT        = "#c8d0e0"
C_ACCENT      = "#e8c840"
C_OK          = "#5ad4a0"
C_WARN        = "#e07850"
C_BAR_BG      = "#1e2330"
C_BAR_FG      = "#5ad4a0"


class DigitalTwin:
    def __init__(self, subscriber: LCMSubscriber):
        self.sub = subscriber
        self._last_guess = None          # warm-start FK
        self._pose_history = []          # for FK status indicator

        # ── Figure layout ────────────────────────────────────────────────────
        self.fig = plt.figure(figsize=(16, 9), facecolor=C_BG)
        self.fig.canvas.manager.set_window_title("Stewart Platform – Digital Twin")

        gs = gridspec.GridSpec(
            3, 3,
            figure=self.fig,
            left=0.02, right=0.98,
            top=0.94, bottom=0.04,
            wspace=0.35, hspace=0.50,
        )

        # 3-D view (spans 2 columns, all 3 rows)
        self.ax3d = self.fig.add_subplot(gs[:, :2], projection='3d')
        self._style_3d_axes(self.ax3d)

        # Servo bar chart
        self.ax_bars = self.fig.add_subplot(gs[0, 2])
        self._style_2d_axes(self.ax_bars, "SERVO ANGLES  (rad)")

        # Telemetry – position
        self.ax_pos = self.fig.add_subplot(gs[1, 2])
        self._style_2d_axes(self.ax_pos, "POSITION  (mm)")

        # Telemetry – orientation
        self.ax_ori = self.fig.add_subplot(gs[2, 2])
        self._style_2d_axes(self.ax_ori, "ORIENTATION  (deg)")

        # Title
        self.fig.text(
            0.5, 0.975, "STEWART PLATFORM  ·  DIGITAL TWIN",
            ha='center', va='top',
            fontsize=13, fontfamily='monospace',
            color=C_ACCENT, fontweight='bold', letterspacing=3,
        )

        # Status dot
        self.status_text = self.fig.text(
            0.98, 0.975, "● CONNECTING",
            ha='right', va='top',
            fontsize=9, fontfamily='monospace', color=C_WARN,
        )

        plt.ion()
        plt.show(block=False)

    # ── Styling helpers ───────────────────────────────────────────────────────

    def _style_3d_axes(self, ax):
        ax.set_facecolor(C_BG)
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.xaxis.pane.set_edgecolor(C_GRID)
        ax.yaxis.pane.set_edgecolor(C_GRID)
        ax.zaxis.pane.set_edgecolor(C_GRID)
        for axis in [ax.xaxis, ax.yaxis, ax.zaxis]:
            axis.label.set_color(C_TEXT)
            axis.set_tick_params(colors=C_TEXT, labelsize=6)
        ax.set_xlabel("X (mm)", color=C_TEXT, fontsize=7, labelpad=2)
        ax.set_ylabel("Y (mm)", color=C_TEXT, fontsize=7, labelpad=2)
        ax.set_zlabel("Z (mm)", color=C_TEXT, fontsize=7, labelpad=2)
        R = BASE_ANCHOR_RADIUS * 1.6
        Z_MAX = BASE_Z_OFFSET + PLATFORM_Z_OFFSET + LOWER_ARM_LENGTH + UPPER_ARM_LENGTH + 5
        ax.set_xlim(-R, R); ax.set_ylim(-R, R); ax.set_zlim(0, Z_MAX)
        ax.view_init(elev=25, azim=-60)
        ax.tick_params(labelsize=6)

    def _style_2d_axes(self, ax, title):
        ax.set_facecolor(C_PANEL)
        for spine in ax.spines.values():
            spine.set_edgecolor(C_GRID)
        ax.tick_params(colors=C_TEXT, labelsize=7)
        ax.set_title(title, color=C_ACCENT, fontsize=8,
                     fontfamily='monospace', pad=4, fontweight='bold')

    # ── Build geometry for current servo angles ───────────────────────────────

    def _compute_geometry(self, servo_angles):
        pos, euler = solve_forward_kinematics(servo_angles, self._last_guess)
        if pos is not None:
            self._last_guess = list(pos) + list(euler)
            R = rotation_matrix(*euler)
        else:
            # FK failed – show a flat neutral pose so display stays live
            z0 = BASE_Z_OFFSET + PLATFORM_Z_OFFSET
            pos = np.array([0.0, 0.0, z0])
            euler = np.array([0.0, 0.0, 0.0])
            R = np.eye(3)

        # Elbow positions (servo horn tips)
        elbows = np.array([
            BASE_ANCHORS[i] + servo_horn_vector(i, servo_angles[i])
            for i in range(NUM_SERVOS)
        ])

        # Platform anchor world positions
        plat_world = np.array([
            pos + R @ PLATFORM_ANCHORS[i]
            for i in range(NUM_SERVOS)
        ])

        # Base anchors – elevate to servo brace height for display
        base_world = BASE_ANCHORS.copy()
        base_world[:, 2] += 0  # base sits at z=0

        return base_world, elbows, plat_world, pos, euler, R

    # ── Draw / update ─────────────────────────────────────────────────────────

    def update(self):
        servo_angles, last_update = self.sub.get_angles()
        base_world, elbows, plat_world, pos, euler, R = self._compute_geometry(servo_angles)
        fk_ok = self._last_guess is not None

        # Status indicator
        age = time.time() - last_update if last_update else 999
        if age < 0.5:
            self.status_text.set_text("● LIVE")
            self.status_text.set_color(C_OK)
        elif age < 2.0:
            self.status_text.set_text("● STALE")
            self.status_text.set_color(C_WARN)
        else:
            self.status_text.set_text("● DEMO")
            self.status_text.set_color(C_WARN)

        self._draw_3d(base_world, elbows, plat_world, pos, euler)
        self._draw_servo_bars(servo_angles)
        self._draw_position(pos, euler, fk_ok)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def _draw_3d(self, base_world, elbows, plat_world, pos, euler):
        ax = self.ax3d
        ax.cla()
        self._style_3d_axes(ax)

        # ── Base ring ────────────────────────────────────────────────────────
        bx = list(base_world[:, 0]) + [base_world[0, 0]]
        by = list(base_world[:, 1]) + [base_world[0, 1]]
        bz = list(base_world[:, 2]) + [base_world[0, 2]]
        ax.plot(bx, by, bz, color=C_BASE, lw=2.0, alpha=0.9, zorder=5)
        ax.scatter(*base_world.T, color=C_ANCHOR_BASE, s=30, zorder=6, depthshade=False)

        # ── Platform ring ────────────────────────────────────────────────────
        px = list(plat_world[:, 0]) + [plat_world[0, 0]]
        py = list(plat_world[:, 1]) + [plat_world[0, 1]]
        pz = list(plat_world[:, 2]) + [plat_world[0, 2]]
        ax.plot(px, py, pz, color=C_PLATFORM, lw=2.0, alpha=0.9, zorder=5)
        ax.scatter(*plat_world.T, color=C_ANCHOR_PLAT, s=30, zorder=6, depthshade=False)

        # Platform fill polygon (translucent)
        verts = [list(zip(plat_world[:, 0], plat_world[:, 1], plat_world[:, 2]))]
        poly = Poly3DCollection(verts, alpha=0.12, facecolor=C_PLATFORM, edgecolor='none')
        ax.add_collection3d(poly)

        # Platform origin dot
        ax.scatter(*pos, color=C_PLATFORM, s=60, zorder=8, depthshade=False, marker='o')

        # Platform normal vector (visual attitude indicator)
        normal = R_from_euler(euler) @ np.array([0, 0, 1])
        scale = 15.0
        ax.quiver(pos[0], pos[1], pos[2],
                  normal[0]*scale, normal[1]*scale, normal[2]*scale,
                  color=C_PLATFORM, alpha=0.7, linewidth=1.5, arrow_length_ratio=0.3)

        # ── Arms (lower + upper) ─────────────────────────────────────────────
        for i in range(NUM_SERVOS):
            # Lower arm: base anchor → elbow
            ax.plot(
                [base_world[i, 0], elbows[i, 0]],
                [base_world[i, 1], elbows[i, 1]],
                [base_world[i, 2], elbows[i, 2]],
                color=C_LOWER_ARM, lw=2.5, alpha=0.95, zorder=4,
            )
            ax.scatter(*elbows[i], color=C_ELBOW, s=22, zorder=7, depthshade=False)

            # Upper arm: elbow → platform anchor
            ax.plot(
                [elbows[i, 0], plat_world[i, 0]],
                [elbows[i, 1], plat_world[i, 1]],
                [elbows[i, 2], plat_world[i, 2]],
                color=C_UPPER_ARM, lw=1.8, alpha=0.85, zorder=4,
            )

        # ── Legend ───────────────────────────────────────────────────────────
        handles = [
            mpatches.Patch(color=C_BASE,       label='Base'),
            mpatches.Patch(color=C_PLATFORM,   label='Platform'),
            mpatches.Patch(color=C_LOWER_ARM,  label='Servo horn'),
            mpatches.Patch(color=C_UPPER_ARM,  label='Upper rod'),
        ]
        ax.legend(handles=handles, loc='upper right', fontsize=7,
                  facecolor=C_PANEL, edgecolor=C_GRID,
                  labelcolor=C_TEXT, framealpha=0.8)

        # Euler angles as text in 3D view
        roll_d, pitch_d, yaw_d = np.degrees(euler)
        ax.text2D(0.02, 0.97,
                  f"r={roll_d:+.1f}°  p={pitch_d:+.1f}°  y={yaw_d:+.1f}°",
                  transform=ax.transAxes,
                  fontsize=7.5, fontfamily='monospace', color=C_TEXT, va='top')

    def _draw_servo_bars(self, servo_angles):
        ax = self.ax_bars
        ax.cla()
        self._style_2d_axes(ax, "SERVO ANGLES  (rad)")

        labels  = [f"S{i}" for i in range(NUM_SERVOS)]
        colors  = [C_LOWER_ARM if abs(a) < math.pi/4 else C_WARN for a in servo_angles]
        bars    = ax.barh(labels, servo_angles, color=colors,
                          height=0.55, left=0)
        ax.axvline(0, color=C_TEXT, lw=0.8, alpha=0.4)

        # Upper/lower limit markers
        ax.axvline( math.pi/2, color=C_WARN, lw=0.6, ls='--', alpha=0.5)
        ax.axvline(-math.pi/4, color=C_WARN, lw=0.6, ls='--', alpha=0.5)

        ax.set_xlim(-math.pi/2, math.pi/2)
        ax.set_facecolor(C_PANEL)
        ax.tick_params(axis='x', labelsize=7, colors=C_TEXT)
        ax.tick_params(axis='y', labelsize=8, colors=C_ACCENT)
        for spine in ax.spines.values():
            spine.set_edgecolor(C_GRID)
        ax.set_xlabel("rad", color=C_TEXT, fontsize=7)

        # Value labels on bars
        for bar, val in zip(bars, servo_angles):
            ax.text(
                bar.get_width() + (0.01 if val >= 0 else -0.01),
                bar.get_y() + bar.get_height()/2,
                f"{val:+.3f}",
                va='center',
                ha='left' if val >= 0 else 'right',
                fontsize=6.5, fontfamily='monospace', color=C_TEXT,
            )

    def _draw_position(self, pos, euler, fk_ok):
        """Draw numeric telemetry panels for position and orientation."""
        roll_d, pitch_d, yaw_d = np.degrees(euler)
        pos_labels  = [("X", pos[0], "mm"), ("Y", pos[1], "mm"), ("Z", pos[2], "mm")]
        ori_labels  = [("ROLL", roll_d, "°"), ("PITCH", pitch_d, "°"), ("YAW", yaw_d, "°")]
        fk_note     = "" if fk_ok else "  (FK failed – neutral)"

        for ax, items, ax_title in [
            (self.ax_pos, pos_labels,  f"POSITION  (mm){fk_note}"),
            (self.ax_ori, ori_labels,  f"ORIENTATION  (deg){fk_note}"),
        ]:
            ax.cla()
            self._style_2d_axes(ax, ax_title)
            ax.set_xlim(0, 1); ax.set_ylim(0, 1)
            ax.axis('off')

            for j, (lbl, val, unit) in enumerate(items):
                y = 0.78 - j * 0.33
                ax.text(0.04, y, lbl,
                        fontsize=9, fontfamily='monospace',
                        color=C_ACCENT, va='center', fontweight='bold')
                ax.text(0.96, y, f"{val:+8.2f} {unit}",
                        fontsize=11, fontfamily='monospace',
                        color=C_TEXT if fk_ok else C_WARN,
                        va='center', ha='right')
                ax.axhline(y - 0.16, color=C_GRID, lw=0.6, xmin=0.02, xmax=0.98)

    def run(self, refresh_hz=20):
        interval = 1.0 / refresh_hz
        print(f"[Visualiser] Running at {refresh_hz} Hz. Close window to quit.")
        try:
            while plt.get_fignums():
                self.update()
                time.sleep(interval)
        except KeyboardInterrupt:
            pass
        print("[Visualiser] Exiting.")


# ─────────────────────────────────────────────────────────────────────────────
# Small helper (outside class to avoid partial init dependency)
# ─────────────────────────────────────────────────────────────────────────────

def R_from_euler(euler):
    return rotation_matrix(*euler)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Stewart Platform Digital Twin")
    parser.add_argument("--hz", type=float, default=20,
                        help="Visualiser refresh rate in Hz (default: 20)")
    args = parser.parse_args()

    sub = LCMSubscriber()
    sub.start()

    twin = DigitalTwin(sub)
    twin.run(refresh_hz=args.hz)

    sub.stop()


if __name__ == "__main__":
    main()