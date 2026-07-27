#!/usr/bin/env python3
"""
plan_safe.py  -  Phase 2c: find a collision-free path (the replanning loop).

This closes the loop from the proposal's flowchart:

    plan a path  ->  is it safe?  --no-->  generate an alternative  --.
                          |                                            |
                         yes  <---------------- re-validate <----------'

Strategy: first try moving straight to the goal. If that path collides
(using the exact check from Phase 2b), lift the motion UP and OVER the
obstacle via an intermediate waypoint, raising the height and re-checking
until the whole trajectory is clear.

Demo: a box on the table sits between two low pick points. The straight
sweep would plow through it, so the arm instead lifts, crosses above, and
sets down on the far side.

    python -m pip install ikpy matplotlib          # (already installed)
    python scripts/plan_safe.py

Animation shows the safe motion (green path) with the rejected direct path
in faint red for contrast. It loops; close the window to finish.
"""
import os
import warnings

import numpy as np

warnings.filterwarnings("ignore", message=".*fixed.*active_links_mask.*")
warnings.filterwarnings("ignore", message=".*single numeric RGB.*")

from ikpy.chain import Chain  # noqa: E402

URDF_PATH = os.path.join(os.path.dirname(__file__), "..", "urdf", "arm6dof.urdf")
CHAIN_PATH = [
    "base_link",
    "joint_1", "link_1", "joint_2", "link_2", "joint_3", "link_3",
    "joint_4", "link_4", "joint_5", "link_5", "joint_6", "link_6",
    "gripper_mount", "gripper_base", "tcp_fixed", "tcp",
]
ACTIVE_MASK = [False, True, True, True, True, True, True, False, False]
SEED = [0.0, -0.5, 1.0, 0.0, 0.5, 0.0]
LINK_RADIUS = 0.05
SAMPLES = 7


# --------------------------------------------------------------------------
# arm + kinematics
# --------------------------------------------------------------------------
def load_arm():
    return Chain.from_urdf_file(URDF_PATH, base_elements=CHAIN_PATH,
                                active_links_mask=ACTIVE_MASK)


def full_vector(j6):
    return [0.0] + list(j6) + [0.0, 0.0]


def tcp_position(chain, j6):
    return chain.forward_kinematics(full_vector(j6))[:3, 3]


def skeleton(chain, j6):
    frames = chain.forward_kinematics(full_vector(j6), full_kinematics=True)
    return [np.asarray(T[:3, 3]) for T in frames]


def link_segments(chain, j6, per=SAMPLES):
    pts = skeleton(chain, j6)
    return [np.array([a + t * (b - a) for t in np.linspace(0, 1, per)])
            for a, b in zip(pts[:-1], pts[1:])]


def solve_ik(chain, target_xyz):
    sol = chain.inverse_kinematics(target_position=list(target_xyz),
                                   initial_position=full_vector(SEED))
    return list(sol[1:7])


def s_curve(t):
    return 10 * t**3 - 15 * t**4 + 6 * t**5


def segment(q_a, q_b, n):
    q_a, q_b = np.asarray(q_a), np.asarray(q_b)
    return [q_a + s_curve(k / (n - 1)) * (q_b - q_a) for k in range(n)]


# --------------------------------------------------------------------------
# obstacle + collision (same method as Phase 2b)
# --------------------------------------------------------------------------
class Box:
    def __init__(self, center, half_extents, name="box"):
        self.c = np.asarray(center, float)
        self.h = np.asarray(half_extents, float)
        self.name = name

    def distance_to_point(self, p):
        return float(np.linalg.norm(np.maximum(np.abs(np.asarray(p) - self.c) - self.h, 0.0)))

    def top_z(self):
        return float(self.c[2] + self.h[2])

    def draw(self, ax, color="orange"):
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        x, y, z = self.c
        dx, dy, dz = self.h
        v = np.array([[x-dx, y-dy, z-dz], [x+dx, y-dy, z-dz], [x+dx, y+dy, z-dz], [x-dx, y+dy, z-dz],
                      [x-dx, y-dy, z+dz], [x+dx, y-dy, z+dz], [x+dx, y+dy, z+dz], [x-dx, y+dy, z+dz]])
        faces = [[0,1,2,3], [4,5,6,7], [0,1,5,4], [2,3,7,6], [1,2,6,5], [0,3,7,4]]
        ax.add_collection3d(Poly3DCollection([v[f] for f in faces], alpha=0.25,
                                             facecolor=color, edgecolor="k"))


def config_collides(chain, j6, obstacles):
    for seg in link_segments(chain, j6):
        for obs in obstacles:
            for p in seg:
                if obs.distance_to_point(p) < LINK_RADIUS:
                    return True
    return False


def path_collisions(chain, path, obstacles):
    return sum(config_collides(chain, q, obstacles) for q in path)


# --------------------------------------------------------------------------
# the replanning loop
# --------------------------------------------------------------------------
def plan_safe(chain, start_xyz, goal_xyz, obstacles, n=60, max_attempts=8):
    """Return (safe_path or None, log). log = list of (description, n_collisions)."""
    q_start, q_goal = solve_ik(chain, start_xyz), solve_ik(chain, goal_xyz)
    log = []

    # 1) try the direct path
    direct = segment(q_start, q_goal, n)
    bad = path_collisions(chain, direct, obstacles)
    log.append(("direct path", bad))
    if bad == 0:
        return direct, log

    # 2) escalate an up-and-over via-point until the path is clear
    mid_x = (start_xyz[0] + goal_xyz[0]) / 2
    mid_y = (start_xyz[1] + goal_xyz[1]) / 2
    base_height = max(o.top_z() for o in obstacles)
    for i in range(max_attempts):
        via_z = base_height + 0.08 + 0.10 * i
        q_via = solve_ik(chain, (mid_x, mid_y, via_z))
        path = segment(q_start, q_via, n // 2) + segment(q_via, q_goal, n // 2)
        bad = path_collisions(chain, path, obstacles)
        log.append((f"lift over, via z={via_z:.2f}", bad))
        if bad == 0:
            return path, log
    return None, log


# --------------------------------------------------------------------------
# demo
# --------------------------------------------------------------------------
def draw_arm(ax, chain, j6, color):
    p = np.array(skeleton(chain, j6))
    ax.plot(p[:, 0], p[:, 1], p[:, 2], color=color, lw=4, marker="o", ms=3)


def main():
    chain = load_arm()
    obstacles = [Box(center=[0.33, 0.0, 0.11], half_extents=[0.09, 0.11, 0.11],
                     name="box on table")]
    start_xyz = (0.35, -0.30, 0.16)
    goal_xyz = (0.35, 0.30, 0.16)

    print(f"\nPlanning a path from {start_xyz} to {goal_xyz} around "
          f"{len(obstacles)} obstacle(s):\n")
    safe_path, log = plan_safe(chain, start_xyz, goal_xyz, obstacles)
    for desc, bad in log:
        verdict = "CLEAR  <- using this" if bad == 0 else f"COLLISION ({bad} waypoints)"
        print(f"  {desc:<24}: {verdict}")

    if safe_path is None:
        print("\nNo safe path found within the attempt limit.")
        return
    attempts = len(log) - 1
    print(f"\nSafe path found after {attempts} replanning attempt(s). "
          f"The arm lifts up and over.\n")
    print("Opening the animation... close the window to finish.\n")

    # rejected direct path (for visual contrast)
    q0, q1 = solve_ik(chain, start_xyz), solve_ik(chain, goal_xyz)
    direct_pts = np.array([tcp_position(chain, q) for q in segment(q0, q1, 60)])
    safe_pts = np.array([tcp_position(chain, q) for q in safe_path])

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")

    def draw(frame):
        ax.cla()
        draw_arm(ax, chain, safe_path[frame], "steelblue")
        for obs in obstacles:
            obs.draw(ax)
        ax.plot(direct_pts[:, 0], direct_pts[:, 1], direct_pts[:, 2],
                color="red", ls=":", lw=1.5, alpha=0.6)          # rejected
        ax.plot(safe_pts[:frame + 1, 0], safe_pts[:frame + 1, 1],
                safe_pts[:frame + 1, 2], color="green", lw=2)     # taken
        ax.scatter(*goal_xyz, c="green", marker="*", s=90)
        ax.set_xlim(0, 0.6)
        ax.set_ylim(-0.4, 0.4)
        ax.set_zlim(0, 0.7)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.set_title(f"Phase 2c - safe path (red = rejected direct)   "
                     f"step {frame+1}/{len(safe_path)}")

    ani = FuncAnimation(fig, draw, frames=len(safe_path), interval=45, repeat=True)
    plt.show()


if __name__ == "__main__":
    main()
