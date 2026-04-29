"""
visualise_stewart.py

Produces:
  1. servo_angles.png   - plot of the 6 servo angles over time
  2. platform_pose.png  - plot of platform position (x,y,z) and orientation
                          (roll, pitch, yaw converted from quaternion) over time
  3. stewart_motion.gif - animated 3-D visualisation of the Stewart platform

Usage:
    python visualise_stewart.py --angles angles.csv --pose pose.csv

CSV formats
-----------
angles.csv  : each row has 6 comma-separated servo angles (radians)
              e.g.  0.1, -0.2, 0.3, 0.1, -0.1, 0.2

pose.csv    : each row has 7 comma-separated values
              pos_x, pos_y, pos_z, quat_x, quat_y, quat_z, quat_w
              e.g.  0, 0, 12.5, 0, 0, 0, 1
"""

import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.spatial.transform import Rotation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ── Stewart platform geometry (must match C++ defines) ────────────────────────
LOWER_ARM_LENGTH        = 18.5      # mm
UPPER_ARM_LENGTH        = 27.0      # mm
BASE_ANCHOR_RADIUS      = 32.7015   # mm
PLATFORM_ANCHOR_RADIUS  = 33.023    # mm
BASE_ANCHOR_ANG_OFFSET  = 0.1581    # rad
PLAT_ANCHOR_ANG_OFFSET  = 0.2877    # rad
BASE_Z_OFFSET           = 9.4248    # mm
PLATFORM_Z_OFFSET       = 3.60      # mm
NUM_SERVOS              = 6

# Angular positions of servo horns viewed from above (matches C++ constructor)
SERVO_ANGULAR_POS = np.array([
    3.0/2.0 * np.pi,
    1.0/2.0 * np.pi,
    1.0/6.0 * np.pi,
    7.0/6.0 * np.pi,
    5.0/6.0 * np.pi,
   11.0/6.0 * np.pi,
])


def build_anchors():
    """Return base_anchors and platform_anchors as (6,3) arrays."""
    base = np.zeros((NUM_SERVOS, 3))
    plat = np.zeros((NUM_SERVOS, 3))
    for k in range(0, NUM_SERVOS, 2):
        centre = np.pi / 3 * k
        base[k]   = BASE_ANCHOR_RADIUS * np.array([np.cos(centre - BASE_ANCHOR_ANG_OFFSET),
                                                    np.sin(centre - BASE_ANCHOR_ANG_OFFSET), 0])
        base[k+1] = BASE_ANCHOR_RADIUS * np.array([np.cos(centre + BASE_ANCHOR_ANG_OFFSET),
                                                    np.sin(centre + BASE_ANCHOR_ANG_OFFSET), 0])
        plat[k]   = PLATFORM_ANCHOR_RADIUS * np.array([np.cos(centre - PLAT_ANCHOR_ANG_OFFSET),
                                                        np.sin(centre - PLAT_ANCHOR_ANG_OFFSET), 0])
        plat[k+1] = PLATFORM_ANCHOR_RADIUS * np.array([np.cos(centre + PLAT_ANCHOR_ANG_OFFSET),
                                                        np.sin(centre + PLAT_ANCHOR_ANG_OFFSET), 0])
    return base, plat


BASE_ANCHORS, PLAT_ANCHORS_LOCAL = build_anchors()


def compute_platform_geometry(pos, quat_xyzw):
    """
    Given platform position (3,) and quaternion (x,y,z,w),
    return the world-space platform anchor positions (6,3),
    elbow positions (6,3), and bottom-arm vectors (6,3).
    pos is the user-space position (without the z offset already added).
    """
    # Add the z offsets that moveTo() applies internally
    world_pos = pos.copy()
    world_pos[2] += BASE_Z_OFFSET + PLATFORM_Z_OFFSET

    rot = Rotation.from_quat(quat_xyzw)  # scipy uses [x,y,z,w]
    plat_world = world_pos + rot.apply(PLAT_ANCHORS_LOCAL)   # (6,3)

    # Elbow positions from inverse kinematics
    elbows = np.zeros((NUM_SERVOS, 3))
    for i in range(NUM_SERVOS):
        c_ab = np.cos(SERVO_ANGULAR_POS[i])
        s_ab = np.sin(SERVO_ANGULAR_POS[i])
        # servo angle stored as the raw float from the CSV
        ang = 0.0  # default; will be overridden when called with angles
        c_h = np.cos(ang)
        s_h = np.sin(ang)
        elbows[i] = BASE_ANCHORS[i] + LOWER_ARM_LENGTH * np.array([c_ab*c_h, s_ab*c_h, s_h])

    return plat_world, elbows


def compute_platform_geometry_with_angles(pos, quat_xyzw, servo_angles):
    """Full version that uses actual servo angles."""
    world_pos = pos.copy()
    world_pos[2] += BASE_Z_OFFSET + PLATFORM_Z_OFFSET

    rot = Rotation.from_quat(quat_xyzw)
    plat_world = world_pos + rot.apply(PLAT_ANCHORS_LOCAL)

    elbows = np.zeros((NUM_SERVOS, 3))
    for i in range(NUM_SERVOS):
        c_ab = np.cos(SERVO_ANGULAR_POS[i])
        s_ab = np.sin(SERVO_ANGULAR_POS[i])
        ang  = servo_angles[i]
        c_h  = np.cos(ang)
        s_h  = np.sin(ang)
        elbows[i] = BASE_ANCHORS[i] + LOWER_ARM_LENGTH * np.array([c_ab*c_h, s_ab*c_h, s_h])

    return plat_world, elbows


# ── Polygon helpers ───────────────────────────────────────────────────────────

def hexagon_verts(points):
    """Return vertices ordered for a filled hexagonal polygon."""
    # Sort by angle around centroid for a clean polygon
    centre = points.mean(axis=0)
    angles = np.arctan2(points[:,1]-centre[1], points[:,0]-centre[0])
    idx = np.argsort(angles)
    return points[idx]


# ── 1. Servo angles plot ──────────────────────────────────────────────────────

def plot_servo_angles(angles: np.ndarray, out_path="servo_angles.png"):
    n_frames = len(angles)
    time = np.arange(n_frames)

    fig, ax = plt.subplots(figsize=(10, 5))
    colours = plt.cm.tab10(np.linspace(0, 0.6, NUM_SERVOS))

    for i in range(NUM_SERVOS):
        ax.plot(time, np.degrees(angles[:, i]), label=f"Servo {i+1}", color=colours[i])

    ax.set_xlabel("Frame")
    ax.set_ylabel("Angle (°)")
    ax.set_title("Servo Angles over Trajectory")
    ax.legend(ncol=3, fontsize=9)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"Saved {out_path}")


# ── 2. Platform pose plot ─────────────────────────────────────────────────────

def plot_platform_pose(pose: np.ndarray, out_path="platform_pose.png"):
    """pose has shape (N, 7): px py pz qx qy qz qw"""
    n_frames = len(pose)
    time = np.arange(n_frames)

    positions = pose[:, :3]
    quats     = pose[:, 3:]          # x y z w

    # Convert to Euler angles (intrinsic ZYX → yaw, pitch, roll → reorder)
    rot = Rotation.from_quat(quats)  # scipy [x,y,z,w]
    euler_deg = rot.as_euler("xyz", degrees=True)   # roll, pitch, yaw

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # Position subplot
    ax = axes[0]
    for j, label in enumerate(["x (mm)", "y (mm)", "z (mm)"]):
        ax.plot(time, positions[:, j], label=label)
    ax.set_ylabel("Position (mm)")
    ax.set_title("Platform Position over Trajectory")
    ax.legend(ncol=3, fontsize=9)
    ax.grid(True, alpha=0.3)

    # Orientation subplot
    ax = axes[1]
    for j, label in enumerate(["Roll (°)", "Pitch (°)", "Yaw (°)"]):
        ax.plot(time, euler_deg[:, j], label=label)
    ax.set_xlabel("Frame")
    ax.set_ylabel("Angle (°)")
    ax.set_title("Platform Orientation over Trajectory")
    ax.legend(ncol=3, fontsize=9)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"Saved {out_path}")


# ── 3. 3-D animated GIF ───────────────────────────────────────────────────────

def make_gif(angles: np.ndarray, pose: np.ndarray, out_path="stewart_motion.gif", fps=15):
    n_frames = len(angles)

    fig = plt.figure(figsize=(7, 7))
    ax  = fig.add_subplot(111, projection="3d")

    R = BASE_ANCHOR_RADIUS + LOWER_ARM_LENGTH + UPPER_ARM_LENGTH
    ax.set_xlim(-R, R);  ax.set_xlabel("X (mm)")
    ax.set_ylim(-R, R);  ax.set_ylabel("Y (mm)")
    ax.set_zlim(0, BASE_Z_OFFSET + PLATFORM_Z_OFFSET + 60)
    ax.set_zlabel("Z (mm)")
    ax.set_title("Stewart Platform Motion")

    # Draw the static base polygon once
    base_verts = hexagon_verts(BASE_ANCHORS[:, :2])
    base_z     = np.zeros(len(base_verts))
    ax.plot(
        np.append(base_verts[:,0], base_verts[0,0]),
        np.append(base_verts[:,1], base_verts[0,1]),
        np.append(base_z, base_z[0]),
        "k-", linewidth=1.5, alpha=0.5
    )
    ax.scatter(BASE_ANCHORS[:,0], BASE_ANCHORS[:,1], BASE_ANCHORS[:,2],
               c="k", s=20, zorder=5)

    # Mutable artists for animation
    arm_lines   = [ax.plot([], [], [], "b-", linewidth=2)[0] for _ in range(NUM_SERVOS)]
    upper_lines = [ax.plot([], [], [], "r-", linewidth=2)[0] for _ in range(NUM_SERVOS)]
    plat_poly   = [None]   # wrapped in list so closure can mutate it
    plat_scatter= ax.scatter([], [], [], c="r", s=30, zorder=6)
    frame_text  = ax.text2D(0.02, 0.95, "", transform=ax.transAxes, fontsize=9)

    def init():
        for line in arm_lines + upper_lines:
            line.set_data([], [])
            line.set_3d_properties([])
        return arm_lines + upper_lines

    def update(frame):
        pos        = pose[frame, :3]
        quat_xyzw  = pose[frame, 3:]
        servo_angs = angles[frame]

        plat_world, elbows = compute_platform_geometry_with_angles(pos, quat_xyzw, servo_angs)

        # Draw lower arms (base anchor → elbow)
        for i in range(NUM_SERVOS):
            xs = [BASE_ANCHORS[i,0], elbows[i,0]]
            ys = [BASE_ANCHORS[i,1], elbows[i,1]]
            zs = [BASE_ANCHORS[i,2], elbows[i,2]]
            arm_lines[i].set_data(xs, ys)
            arm_lines[i].set_3d_properties(zs)

        # Draw upper arms (elbow → platform anchor)
        for i in range(NUM_SERVOS):
            xs = [elbows[i,0],     plat_world[i,0]]
            ys = [elbows[i,1],     plat_world[i,1]]
            zs = [elbows[i,2],     plat_world[i,2]]
            upper_lines[i].set_data(xs, ys)
            upper_lines[i].set_3d_properties(zs)

        # Redraw platform polygon
        if plat_poly[0] is not None:
            plat_poly[0].remove()
        verts_2d = hexagon_verts(plat_world)
        poly = Poly3DCollection(
            [verts_2d],
            alpha=0.25, facecolor="steelblue", edgecolor="navy", linewidth=1
        )
        ax.add_collection3d(poly)
        plat_poly[0] = poly

        # Update platform anchor scatter
        plat_scatter._offsets3d = (plat_world[:,0], plat_world[:,1], plat_world[:,2])

        frame_text.set_text(f"Frame {frame+1}/{n_frames}")

        return arm_lines + upper_lines + [plat_scatter, frame_text]

    interval_ms = int(1000 / fps)
    anim = animation.FuncAnimation(
        fig, update, frames=n_frames, init_func=init,
        interval=interval_ms, blit=False
    )

    writer = animation.PillowWriter(fps=fps)
    anim.save(out_path, writer=writer)
    plt.close(fig)
    print(f"Saved {out_path}")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Visualise Stewart platform trajectory.")
    parser.add_argument("--angles", required=True,
                        help="Path to servo angles CSV (6 columns, radians)")
    parser.add_argument("--pose",   required=True,
                        help="Path to platform pose CSV (7 columns: px py pz qx qy qz qw)")
    parser.add_argument("--fps",    type=int, default=15,
                        help="Frames per second for the GIF (default: 15)")
    args = parser.parse_args()

    angles = np.loadtxt(args.angles, delimiter=",")   # (N, 6)
    pose   = np.loadtxt(args.pose,   delimiter=",")   # (N, 7)

    if angles.ndim == 1:
        angles = angles[np.newaxis, :]
    if pose.ndim == 1:
        pose = pose[np.newaxis, :]

    assert angles.shape[1] == NUM_SERVOS, \
        f"Expected {NUM_SERVOS} angle columns, got {angles.shape[1]}"
    assert pose.shape[1] == 7, \
        f"Expected 7 pose columns, got {pose.shape[1]}"
    assert len(angles) == len(pose), \
        "angles and pose CSVs must have the same number of rows"

    plot_servo_angles(angles)
    plot_platform_pose(pose)
    make_gif(angles, pose, fps=args.fps)


if __name__ == "__main__":
    main()