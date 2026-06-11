#!/usr/bin/env python3
"""
Stewart Platform Servo Debug GUI
=================================
Displays rolling time-series graphs of:
  - Platform target position (x, y, z) and orientation (roll, pitch, yaw)
  - Target servo angles (dashed) and true servo angles (solid) for all 6 servos

Subscribes to:
  PLATFORM_TARGET  ->  platform_target_t   (position mm, orientation rad)
  SERVO_TARGETS    ->  servo_targets_t     (commanded angles rad)
  SERVO_STATE      ->  true_servo_angles_t (measured angles rad)

Usage:
    python3 servo_debug_gui.py
    python3 servo_debug_gui.py --window 60
"""

import sys
import os
import argparse
import threading
import time
import collections
import math

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation

# ── LCM message imports ───────────────────────────────────────────────────────
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lcm_messages'))
try:
    import lcm
    from payload_messages import servo_targets_t, true_servo_angles_t, platform_target_t
    LCM_AVAILABLE = True
except ImportError as e:
    print(f"[WARN] LCM import failed: {e}. Running in demo mode.")
    LCM_AVAILABLE = False

# ── Tuneable constants ────────────────────────────────────────────────────────
TRUE_ANGLE_ALPHA = 0.8      # Opacity of true servo angle lines
ANIMATION_INTERVAL_MS = 100 # Redraw period (10 Hz)
NUM_SERVOS = 6

# One colour per servo, shared between target (dashed) and true (solid) lines.
SERVO_COLOURS = [
    "#e6194b",  # servo 0 – red
    "#3cb44b",  # servo 1 – green
    "#4363d8",  # servo 2 – blue
    "#f58231",  # servo 3 – orange
    "#911eb4",  # servo 4 – purple
    "#42d4f4",  # servo 5 – cyan
]


# ── Thread-safe data buffers ──────────────────────────────────────────────────

class DataStore:
    """Stores timestamped samples in deques capped by the requested window."""

    def __init__(self, window_s: float):
        self.window = window_s
        self.lock = threading.Lock()
        # Each buffer is a deque of (t, value) tuples where t is time.time().
        self.platform_t:   collections.deque = collections.deque()  # (t, [x,y,z])
        self.platform_rpy: collections.deque = collections.deque()  # (t, [r,p,y])
        self.servo_target: collections.deque = collections.deque()  # (t, [a0..a5])
        self.servo_true:   collections.deque = collections.deque()  # (t, [a0..a5])

    def _trim(self, buf: collections.deque, now: float):
        while buf and (now - buf[0][0]) > self.window:
            buf.popleft()

    def add_platform(self, t: float, pos, rpy):
        with self.lock:
            self.platform_t.append((t, list(pos)))
            self.platform_rpy.append((t, list(rpy)))
            self._trim(self.platform_t, t)
            self._trim(self.platform_rpy, t)

    def add_servo_target(self, t: float, angles):
        with self.lock:
            self.servo_target.append((t, list(angles)))
            self._trim(self.servo_target, t)

    def add_servo_true(self, t: float, angles):
        with self.lock:
            self.servo_true.append((t, list(angles)))
            self._trim(self.servo_true, t)

    def snapshot(self):
        """Return a consistent copy of all buffers under the lock."""
        with self.lock:
            return (
                list(self.platform_t),
                list(self.platform_rpy),
                list(self.servo_target),
                list(self.servo_true),
            )


# ── LCM subscriber thread ─────────────────────────────────────────────────────

class LCMSubscriber:
    def __init__(self, store: DataStore):
        self.store = store
        self.lc = lcm.LCM()
        self.lc.subscribe("PLATFORM_TARGET", self._on_platform)
        self.lc.subscribe("SERVO_TARGETS",   self._on_servo_target)
        self.lc.subscribe("SERVO_STATE",     self._on_servo_true)

    def _on_platform(self, channel, data):
        msg = platform_target_t.decode(data)
        t = time.time()
        pos = msg.position          # [x, y, z] mm
        rpy = msg.orientation       # [roll, pitch, yaw] rad
        self.store.add_platform(t, pos, rpy)

    def _on_servo_target(self, channel, data):
        msg = servo_targets_t.decode(data)
        self.store.add_servo_target(time.time(), msg.angles)

    def _on_servo_true(self, channel, data):
        msg = true_servo_angles_t.decode(data)
        self.store.add_servo_true(time.time(), msg.angles)

    def spin(self):
        while True:
            self.lc.handle_timeout(10)


# ── Demo mode: inject synthetic data ─────────────────────────────────────────

class DemoInjector:
    def __init__(self, store: DataStore):
        self.store = store
        self._t0 = time.time()

    def spin(self):
        while True:
            t = time.time()
            elapsed = t - self._t0
            pos = [5 * math.sin(elapsed), 5 * math.cos(elapsed), 10 + 3 * math.sin(0.3 * elapsed)]
            rpy = [0.1 * math.sin(elapsed + i) for i in range(3)]
            target = [0.3 * math.sin(elapsed + i * 0.5) for i in range(NUM_SERVOS)]
            true_  = [target[i] + 0.02 * math.sin(5 * elapsed + i) for i in range(NUM_SERVOS)]
            self.store.add_platform(t, pos, rpy)
            self.store.add_servo_target(t, target)
            self.store.add_servo_true(t, true_)
            time.sleep(0.05)


# ── Plot helpers ──────────────────────────────────────────────────────────────

def _extract_series(buf, idx):
    """Return (times_relative, values) from a buffer of (t, list) tuples."""
    if not buf:
        return np.array([]), np.array([])
    now = buf[-1][0]
    ts = np.array([now - entry[0] for entry in buf])
    vs = np.array([entry[1][idx] for entry in buf])
    return -ts, vs   # negative so recent data is at x=0 on the right


def _update_line(line, xs, ys):
    line.set_xdata(xs)
    line.set_ydata(ys)


# ── Main GUI class ────────────────────────────────────────────────────────────

class DebugGUI:
    def __init__(self, store: DataStore, window_s: float):
        self.store = store
        self.window = window_s

        self.fig = plt.figure(figsize=(14, 8))
        self.fig.canvas.manager.set_window_title("Stewart Platform Debug")
        gs = gridspec.GridSpec(2, 2, figure=self.fig, hspace=0.45, wspace=0.35)

        # ── Row 0: position | orientation ────────────────────────────────────
        self.ax_pos = self.fig.add_subplot(gs[0, 0])
        self.ax_rpy = self.fig.add_subplot(gs[0, 1])

        self.ax_pos.set_title("Platform Position")
        self.ax_pos.set_ylabel("mm")
        self.ax_rpy.set_title("Platform Orientation")
        self.ax_rpy.set_ylabel("deg")

        pos_labels = ["x", "y", "z"]
        pos_colours = ["#e6194b", "#3cb44b", "#4363d8"]
        rpy_labels = ["roll", "pitch", "yaw"]
        rpy_colours = ["#f58231", "#911eb4", "#42d4f4"]

        self.lines_pos = [
            self.ax_pos.plot([], [], label=pos_labels[i], color=pos_colours[i])[0]
            for i in range(3)
        ]
        self.lines_rpy = [
            self.ax_rpy.plot([], [], label=rpy_labels[i], color=rpy_colours[i])[0]
            for i in range(3)
        ]
        self.ax_pos.legend(fontsize=8, loc="upper left")
        self.ax_rpy.legend(fontsize=8, loc="upper left")

        # ── Row 1: servo angles (full width) ─────────────────────────────────
        self.ax_servo = self.fig.add_subplot(gs[1, :])
        self.ax_servo.set_title("Servo Angles  (dashed = target, solid = true)")
        self.ax_servo.set_ylabel("deg")
        self.ax_servo.set_xlabel("time (s)")

        self.lines_servo_target = []
        self.lines_servo_true = []
        for i in range(NUM_SERVOS):
            lt, = self.ax_servo.plot([], [], linestyle='--', color=SERVO_COLOURS[i],
                                     label=f"Servo {i} target")
            lv, = self.ax_servo.plot([], [], linestyle='-', color=SERVO_COLOURS[i],
                                     alpha=TRUE_ANGLE_ALPHA, label=f"Servo {i} true")
            self.lines_servo_target.append(lt)
            self.lines_servo_true.append(lv)
        self.ax_servo.legend(fontsize=7, loc="upper left", ncol=2)

        self._set_xlims()
        for ax in (self.ax_pos, self.ax_rpy, self.ax_servo):
            ax.set_xlim(-self.window, 0)
            ax.grid(True, alpha=0.3)

    def _set_xlims(self):
        for ax in (self.ax_pos, self.ax_rpy, self.ax_servo):
            ax.set_xlim(-self.window, 0)

    def update(self, _frame):
        pt, prpy, st, sv = self.store.snapshot()

        # Position
        for i, line in enumerate(self.lines_pos):
            xs, ys = _extract_series(pt, i)
            _update_line(line, xs, ys)
        self.ax_pos.relim(); self.ax_pos.autoscale_view(scalex=False)

        # Orientation (convert rad → deg)
        for i, line in enumerate(self.lines_rpy):
            xs, ys = _extract_series(prpy, i)
            _update_line(line, xs, np.degrees(ys) if len(ys) else ys)
        self.ax_rpy.relim(); self.ax_rpy.autoscale_view(scalex=False)

        # Servo angles (convert rad → deg)
        for i in range(NUM_SERVOS):
            xs, ys = _extract_series(st, i)
            _update_line(self.lines_servo_target[i], xs, np.degrees(ys) if len(ys) else ys)
            xs, ys = _extract_series(sv, i)
            _update_line(self.lines_servo_true[i], xs, np.degrees(ys) if len(ys) else ys)
        self.ax_servo.relim(); self.ax_servo.autoscale_view(scalex=False)

        self._set_xlims()
        return (self.lines_pos + self.lines_rpy +
                self.lines_servo_target + self.lines_servo_true)

    def run(self):
        self._anim = animation.FuncAnimation(
            self.fig, self.update,
            interval=ANIMATION_INTERVAL_MS,
            blit=False,
        )
        plt.show()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Stewart Platform Servo Debug GUI")
    parser.add_argument("--window", type=float, default=60.0,
                        metavar="SECONDS",
                        help="Rolling graph time window in seconds (default: 20)")
    args = parser.parse_args()

    store = DataStore(window_s=args.window)

    if LCM_AVAILABLE:
        subscriber = LCMSubscriber(store)
        t = threading.Thread(target=subscriber.spin, daemon=True)
        print(f"[INFO] LCM subscribed. Window: {args.window}s")
    else:
        subscriber = DemoInjector(store)
        t = threading.Thread(target=subscriber.spin, daemon=True)
        print(f"[INFO] Demo mode. Window: {args.window}s")

    t.start()

    gui = DebugGUI(store, window_s=args.window)
    gui.run()


if __name__ == "__main__":
    main()
