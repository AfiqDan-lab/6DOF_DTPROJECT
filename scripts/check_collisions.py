#!/usr/bin/env python3
"""
check_collisions.py  -  Phase 2b: does the arm hit anything?

We can't run a full physics engine on this setup, so we use the standard
lightweight method: wrap each arm link in a capsule (its centerline + a
thickness = LINK_RADIUS), model obstacles as boxes and spheres, and measure
distances. If any part of the arm comes closer to an obstacle than its own
thickness, that configuration is in collision. The same distance idea, applied
between the arm's own links, catches self-collision.

This demo sweeps the arm horizontally through a post standing in its way. The
arm glides blue while clear and turns RED on every configuration that would
collide -- and prints which parts of the trajectory are unsafe. Phase 2c will
use exactly this check to route around the obstacle.

    python -m pip install ikpy matplotlib          # (already installed)
    python scripts/check_collisions.py

The animation loops; close the window to finish.
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

LINK_RADIUS = 0.05   # arm "thickness" used for the capsule model (metres)
SAMPLES = 7          # points sampled along each link segment


# --------------------------------------------------------------------------
# arm model
# --------------------------------------------------------------------------
def load_arm() -> Chain:
    return Chain.from_urdf_file(URDF_PATH, base_elements=CHAIN_PATH,
                                active_links_mask=ACTIVE_MASK)


def full_vector(joints6):
    return [0.0] + list(joints6) + [0.0, 0.0]


def tcp_position(chain, joints6):
    return chain.forward_kinematics(full_vector(joints6))[:3, 3]


def skeleton(chain, joints6):
    """3D positions of every joint frame, base -> tool (the arm's centerline)."""
    frames = chain.forward_kinematics(full_vector(joints6), full_kinematics=True)
    return [np.asarray(T[:3, 3]) for T in frames]


def link_segments(chain, joints6, per=SAMPLES):
    """List of per-link point clouds sampled along each segment of the skeleton."""
    pts = skeleton(chain, joints6)
    segs = []
    for a, b in zip(pts[:-1], pts[1:]):
        segs.append(np.array([a + t * (b - a) for t in np.linspace(0, 1, per)]))
    return segs


def solve_ik(chain, target_xyz):
    sol = chain.inverse_kinematics(target_position=list(target_xyz),
                                   initial_position=full_vector(SEED))
    return list(sol[1:7])


def s_curve(t):
    return 10 * t**3 - 15 * t**4 + 6 * t**5


def trajectory(q_start, q_goal, n=60):
    q_start, q_goal = np.asarray(q_start), np.asarray(q_goal)
    return [q_start + s_curve(k / (n - 1)) * (q_goal - q_start) for k in range(n)]


# --------------------------------------------------------------------------
# obstacles
# --------------------------------------------------------------------------
class Box:
    """Axis-aligned box: center and half-extents (half the width in each axis)."""
    def __init__(self, center, half_extents, name="box"):
        self.c = np.asarray(center, float)
        self.h = np.asarray(half_extents, float)
        self.name = name

    def distance_to_point(self, p):
        return float(np.linalg.norm(np.maximum(np.abs(np.asarray(p) - self.c) - self.h, 0.0)))

    def draw(self, ax, color="orange"):
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        x, y, z = self.c
        dx, dy, dz = self.h
        c = np.array([[x-dx, y-dy, z-dz], [x+dx, y-dy, z-dz], [x+dx, y+dy, z-dz], [x-dx, y+dy, z-dz],
                      [x-dx, y-dy, z+dz], [x+dx, y-dy, z+dz], [x+dx, y+dy, z+dz], [x-dx, y+dy, z+dz]])
        faces = [[0,1,2,3], [4,5,6,7], [0,1,5,4], [2,3,7,6], [1,2,6,5], [0,3,7,4]]
        ax.add_collection3d(Poly3DCollection([c[f] for f in faces], alpha=0.25,
                                             facecolor=color, edgecolor="k"))


class Sphere:
    def __init__(self, center, radius, name="sphere"):
        self.c = np.asarray(center, float)
        self.r = float(radius)
        self.name = name

    def distance_to_point(self, p):
        return float(np.linalg.norm(np.asarray(p) - self.c) - self.r)

    def draw(self, ax, color="orange"):
        u, v = np.mgrid[0:2*np.pi:16j, 0:np.pi:8j]
        ax.plot_surface(self.c[0]+self.r*np.cos(u)*np.sin(v),
                        self.c[1]+self.r*np.sin(u)*np.sin(v),
                        self.c[2]+self.r*np.cos(v), color=color, alpha=0.25)


# --------------------------------------------------------------------------
# collision tests
# --------------------------------------------------------------------------
def hits_obstacles(segs, obstacles):
    for obs in obstacles:
        for seg in segs:
            for p in seg:
                if obs.distance_to_point(p) < LINK_RADIUS:
                    return True, obs.name
    return False, None


def self_collision(segs, thresh=2 * LINK_RADIUS):
    """Conservative check: distal links vs the base/shoulder links (skip neighbours)."""
    proximal, distal = [0, 1], [3, 4, 5, 6, 7]
    for i in proximal:
        for j in distal:
            for p in segs[i]:
                if np.min(np.linalg.norm(segs[j] - p, axis=1)) < thresh:
                    return True
    return False


def is_colliding(chain, joints6, obstacles):
    """Return (colliding: bool, reason: str)."""
    segs = link_segments(chain, joints6)
    if self_collision(segs):
        return True, "self-collision"
    hit, name = hits_obstacles(segs, obstacles)
    return (True, f"obstacle '{name}'") if hit else (False, "clear")


def check_trajectory(chain, traj, obstacles):
    flags = [is_colliding(chain, q, obstacles)[0] for q in traj]
    n_bad = sum(flags)
    first = next((i for i, f in enumerate(flags) if f), None)
    print(f"\nChecked {len(traj)} waypoints against {len(obstacles)} obstacle(s).")
    if n_bad == 0:
        print("  Result: path is CLEAR end to end.")
    else:
        print(f"  Result: {n_bad}/{len(traj)} waypoints are in collision "
              f"(first at step {first}).")
        print("  -> This direct path is UNSAFE. Phase 2c will route around it.")
    return flags


def draw_arm(ax, chain, joints6, color):
    p = np.array(skeleton(chain, joints6))
    ax.plot(p[:, 0], p[:, 1], p[:, 2], color=color, lw=4, marker="o", ms=3)


# --------------------------------------------------------------------------
# demo
# --------------------------------------------------------------------------
def main():
    chain = load_arm()

    obstacles = [Box(center=[0.35, 0.0, 0.30], half_extents=[0.08, 0.05, 0.30],
                     name="post")]

    start = solve_ik(chain, (0.35, -0.30, 0.35))   # reach on the right
    goal = solve_ik(chain, (0.35, 0.30, 0.35))      # reach on the left
    traj = trajectory(start, goal, 60)
    path = np.array([tcp_position(chain, q) for q in traj])

    flags = check_trajectory(chain, traj, obstacles)
    print("\nOpening the animation (arm turns red where it collides)... "
          "close the window to finish.\n")

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")

    def draw(frame):
        ax.cla()
        colliding = flags[frame]
        draw_arm(ax, chain, traj[frame], "red" if colliding else "steelblue")
        for obs in obstacles:
            obs.draw(ax)
        ax.plot(path[:, 0], path[:, 1], path[:, 2], color="0.7", ls="--", lw=1)
        ax.set_xlim(0, 0.6)
        ax.set_ylim(-0.4, 0.4)
        ax.set_zlim(0, 0.7)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        state = "COLLISION" if colliding else "clear"
        ax.set_title(f"Phase 2b - collision check   step {frame+1}/{len(traj)}   [{state}]")

    ani = FuncAnimation(fig, draw, frames=len(traj), interval=45, repeat=True)
    plt.show()


if __name__ == "__main__":
    main()
