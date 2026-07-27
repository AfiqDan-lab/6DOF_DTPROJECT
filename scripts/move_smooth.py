#!/usr/bin/env python3
"""
move_smooth.py  -  Phase 2a: smooth motion between two poses.

Until now, IK gave us a single goal configuration and the arm just appeared
there. A real arm can't teleport -- its joints have to accelerate, travel, and
decelerate. So this builds a *trajectory*: a time-ordered sequence of in-between
joint configurations that the arm passes through smoothly.

The smoothness comes from a quintic (5th-order) "s-curve" time scaling, which
starts and ends with zero speed and zero acceleration -- so the arm eases into
motion and eases to a stop, exactly like real robot controllers do.

    python -m pip install ikpy matplotlib          # (already installed)

    python scripts/move_smooth.py                  # default A -> B move
    python scripts/move_smooth.py 0.2 0.3 0.65     # move to your own X Y Z goal

An animation window opens showing the arm gliding to the goal (green star),
tracing its tool path in red. It loops; close the window to finish.
"""
import argparse
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

N_STEPS = 60  # number of waypoints along the trajectory


def load_arm() -> Chain:
    return Chain.from_urdf_file(URDF_PATH, base_elements=CHAIN_PATH,
                                active_links_mask=ACTIVE_MASK)


def full_vector(joints6):
    return [0.0] + list(joints6) + [0.0, 0.0]


def tcp_position(chain, joints6):
    return chain.forward_kinematics(full_vector(joints6))[:3, 3]


def solve_ik(chain, target_xyz):
    sol = chain.inverse_kinematics(target_position=list(target_xyz),
                                   initial_position=full_vector(SEED))
    return list(sol[1:7])


def s_curve(t):
    """Quintic smoothstep: 0 -> 1 with zero velocity & acceleration at the ends."""
    return 10 * t**3 - 15 * t**4 + 6 * t**5


def trajectory(q_start, q_goal, n=N_STEPS):
    """A list of n joint configurations easing from q_start to q_goal."""
    q_start, q_goal = np.asarray(q_start), np.asarray(q_goal)
    return [q_start + s_curve(k / (n - 1)) * (q_goal - q_start) for k in range(n)]


def main():
    ap = argparse.ArgumentParser(description="Smoothly move the arm between two poses.")
    ap.add_argument("coords", nargs="*", type=float,
                    help="goal X Y Z in metres (omit for the default move)")
    args = ap.parse_args()

    chain = load_arm()

    start_xyz = (0.35, -0.25, 0.30)
    if len(args.coords) == 3:
        goal_xyz = tuple(args.coords)
    elif len(args.coords) == 0:
        goal_xyz = (0.20, 0.30, 0.65)
    else:
        ap.error("give exactly three numbers: X Y Z  (or none for the default)")

    q_start = solve_ik(chain, start_xyz)
    q_goal = solve_ik(chain, goal_xyz)
    traj = trajectory(q_start, q_goal)
    path = np.array([tcp_position(chain, q) for q in traj])

    print(f"\nStart pose  -> TCP {np.round(tcp_position(chain, q_start), 3)}")
    print(f"Goal  pose  -> TCP {np.round(tcp_position(chain, q_goal), 3)}")
    print(f"Trajectory  :  {len(traj)} smooth waypoints")
    print("\nOpening the animation... it loops; close the window to finish.\n")

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")

    def draw(frame):
        ax.cla()
        chain.plot(full_vector(traj[frame]), ax)
        # tool path: faint full route + solid trace of where we've been
        ax.plot(path[:, 0], path[:, 1], path[:, 2], color="0.8", ls="--", lw=1)
        ax.plot(path[:frame + 1, 0], path[:frame + 1, 1], path[:frame + 1, 2],
                color="red", lw=2)
        ax.scatter(*goal_xyz, c="green", marker="*", s=90)
        ax.set_xlim(-0.6, 0.6)
        ax.set_ylim(-0.6, 0.6)
        ax.set_zlim(0, 1.0)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.set_title(f"Phase 2a - smooth trajectory   (step {frame + 1}/{len(traj)})")

    # keep a reference to the animation so it isn't garbage-collected
    ani = FuncAnimation(fig, draw, frames=len(traj), interval=40, repeat=True)
    plt.show()


if __name__ == "__main__":
    main()
