#!/usr/bin/env python3
"""
arm_lib.py  -  shared toolkit for the 6-DOF arm (Phases 1-2 consolidated).

Import this from other scripts so they stay short:

    from arm_lib import load_arm, solve_ik, plan_safe, Box, tcp_position

Nothing here is new -- it's the kinematics (Phase 1), smooth trajectories
(Phase 2a), collision detection (Phase 2b), and safe planner (Phase 2c),
gathered in one place.
"""
import os

import numpy as np
import warnings

warnings.filterwarnings("ignore", message=".*fixed.*active_links_mask.*")
warnings.filterwarnings("ignore", message=".*single numeric RGB.*")

from ikpy.chain import Chain

# arm_lib.py lives in scripts/, the URDF in ../urdf/
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


# ---------- kinematics ----------
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


_ik_rng = np.random.default_rng(0)
IK_ATTEMPTS = 8   # 1 default seed + random restarts; keeps the best solution


def solve_ik(chain, target_xyz, attempts=IK_ATTEMPTS):
    """Return (joint_angles[6], achieved_xyz, error_metres).

    Tries several starting guesses and keeps the closest solution, which
    recovers hard targets near the workspace edge that a single seed misses.
    Stops early as soon as a sub-0.1 mm solution is found (so easy targets
    stay fast).
    """
    target = np.asarray(target_xyz)
    seeds = [full_vector(SEED)]
    for _ in range(attempts - 1):
        seeds.append(full_vector(_ik_rng.uniform(-2.0, 2.0, size=6)))

    best_sol, best_ach, best_err = None, None, np.inf
    for seed in seeds:
        sol = chain.inverse_kinematics(target_position=list(target),
                                       initial_position=seed)
        ach = chain.forward_kinematics(sol)[:3, 3]
        err = float(np.linalg.norm(np.asarray(ach) - target))
        if err < best_err:
            best_sol, best_ach, best_err = sol, ach, err
        if best_err < 1e-4:
            break
    return [float(a) for a in best_sol[1:7]], np.asarray(best_ach), best_err


# ---------- smooth trajectories ----------
def s_curve(t):
    """Quintic smoothstep: zero velocity & acceleration at both ends."""
    return 10 * t**3 - 15 * t**4 + 6 * t**5


def segment(q_a, q_b, n):
    q_a, q_b = np.asarray(q_a), np.asarray(q_b)
    return [q_a + s_curve(k / (n - 1)) * (q_b - q_a) for k in range(n)]


# ---------- obstacles ----------
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


class Sphere:
    def __init__(self, center, radius, name="sphere"):
        self.c = np.asarray(center, float)
        self.r = float(radius)
        self.name = name

    def distance_to_point(self, p):
        return float(np.linalg.norm(np.asarray(p) - self.c) - self.r)

    def top_z(self):
        return float(self.c[2] + self.r)


# ---------- collision ----------
def config_collides(chain, j6, obstacles):
    for seg in link_segments(chain, j6):
        for obs in obstacles:
            for p in seg:
                if obs.distance_to_point(p) < LINK_RADIUS:
                    return True
    return False


def path_collisions(chain, path, obstacles):
    return sum(config_collides(chain, q, obstacles) for q in path)


# ---------- safe planner (Phase 2c) ----------
def plan_safe(chain, start_xyz, goal_xyz, obstacles, n=60, max_attempts=8):
    """Return (safe_path or None, log[(description, n_collisions)])."""
    q_start = solve_ik(chain, start_xyz)[0]
    q_goal = solve_ik(chain, goal_xyz)[0]
    log = []

    direct = segment(q_start, q_goal, n)
    bad = path_collisions(chain, direct, obstacles)
    log.append(("direct path", bad))
    if bad == 0:
        return direct, log

    mid_x = (start_xyz[0] + goal_xyz[0]) / 2
    mid_y = (start_xyz[1] + goal_xyz[1]) / 2
    base_height = max(o.top_z() for o in obstacles)
    for i in range(max_attempts):
        via_z = base_height + 0.08 + 0.10 * i
        q_via = solve_ik(chain, (mid_x, mid_y, via_z))[0]
        path = segment(q_start, q_via, n // 2) + segment(q_via, q_goal, n // 2)
        bad = path_collisions(chain, path, obstacles)
        log.append((f"lift over, via z={via_z:.2f}", bad))
        if bad == 0:
            return path, log
    return None, log


# ---------- viz helper ----------
def draw_arm(ax, chain, j6, color="steelblue"):
    p = np.array(skeleton(chain, j6))
    ax.plot(p[:, 0], p[:, 1], p[:, 2], color=color, lw=4, marker="o", ms=3)
