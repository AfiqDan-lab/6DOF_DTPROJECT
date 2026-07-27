#!/usr/bin/env python3
"""
ik_demo.py  -  inverse kinematics for the 6-DOF arm.

Forward kinematics: joint angles -> tool position.
Inverse kinematics: tool position -> joint angles.   <-- this file

Give it a target point and it finds the six joint angles that put the tool
tip there, then double-checks by running FK on the answer and measuring how
far off it landed. That error is your proposal's "position accuracy" metric
(target: under 1 cm).

    python -m pip install ikpy matplotlib          # (already done in Phase 0)

    python scripts/ik_demo.py                      # run a set of demo targets
    python scripts/ik_demo.py 0.3 0.1 0.5          # solve for your own X Y Z (metres)

A 3D window opens with the arm reaching the target (red star). Drag to rotate.
"""
import argparse
import os
import warnings

import numpy as np

warnings.filterwarnings("ignore", message=".*fixed.*active_links_mask.*")
warnings.filterwarnings("ignore", message=".*single numeric RGB.*")  # ikpy plot colour nag (harmless)

from ikpy.chain import Chain  # noqa: E402

URDF_PATH = os.path.join(os.path.dirname(__file__), "..", "urdf", "arm6dof.urdf")

CHAIN_PATH = [
    "base_link",
    "joint_1", "link_1", "joint_2", "link_2", "joint_3", "link_3",
    "joint_4", "link_4", "joint_5", "link_5", "joint_6", "link_6",
    "gripper_mount", "gripper_base", "tcp_fixed", "tcp",
]
ACTIVE_MASK = [False, True, True, True, True, True, True, False, False]

# A bent "ready" posture used to seed the numerical IK search. A good seed
# helps it converge to a natural solution.
SEED_ANGLES = [0.0, -0.5, 1.0, 0.0, 0.5, 0.0]

TOLERANCE_M = 0.01  # 1 cm -- the proposal's position-accuracy target


def load_arm() -> Chain:
    return Chain.from_urdf_file(URDF_PATH, base_elements=CHAIN_PATH,
                                active_links_mask=ACTIVE_MASK)


def full_vector(joints6):
    """ikpy wants one value per link; pad the trailing fixed links with 0."""
    return [0.0] + list(joints6) + [0.0, 0.0]


def solve_ik(chain: Chain, target_xyz):
    """Return (joint_angles_rad[6], achieved_xyz, error_metres)."""
    seed = full_vector(SEED_ANGLES)
    solution = chain.inverse_kinematics(target_position=list(target_xyz),
                                        initial_position=seed)
    achieved = chain.forward_kinematics(solution)[:3, 3]
    error = float(np.linalg.norm(np.asarray(achieved) - np.asarray(target_xyz)))
    joints6 = [float(a) for a in solution[1:7]]
    return joints6, np.asarray(achieved), error, solution


def report(target, joints6, achieved, error):
    deg = [round(float(np.degrees(a)), 1) for a in joints6]
    reach = "reachable" if error <= TOLERANCE_M else "OUT OF REACH"
    print(f"\ntarget  = ({target[0]:+.3f}, {target[1]:+.3f}, {target[2]:+.3f}) m")
    print(f"  joint angles (deg): {deg}")
    print(f"  tool landed at    : ({achieved[0]:+.3f}, {achieved[1]:+.3f}, {achieved[2]:+.3f}) m")
    print(f"  position error    : {error * 1000:6.2f} mm   -> {reach}")
    if error > TOLERANCE_M:
        print("  (the arm stretched as close as it physically can, but this "
              "point is outside its workspace)")


def show(chain, cases):
    """cases: list of (title, target, solution)."""
    import matplotlib.pyplot as plt

    n = len(cases)
    cols = 2 if n > 1 else 1
    rows = (n + cols - 1) // cols
    fig = plt.figure(figsize=(6 * cols, 4.5 * rows))
    for k, (title, target, solution) in enumerate(cases):
        ax = fig.add_subplot(rows, cols, k + 1, projection="3d")
        chain.plot(solution, ax)
        ax.scatter(*target, c="red", s=70, marker="*")
        ax.set_title(title)
        ax.set_xlim(-0.6, 0.8)
        ax.set_ylim(-0.6, 0.6)
        ax.set_zlim(0, 1.0)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
    fig.suptitle("Phase 1 - inverse kinematics (red star = target)")
    plt.tight_layout()
    plt.show()


def main():
    ap = argparse.ArgumentParser(description="Solve IK for the 6-DOF arm.")
    ap.add_argument("coords", nargs="*", type=float,
                    help="target X Y Z in metres (omit to run demo targets)")
    args = ap.parse_args()

    chain = load_arm()

    if len(args.coords) == 3:
        targets = [("your target", tuple(args.coords))]
    elif len(args.coords) == 0:
        targets = [
            ("A", (0.3, 0.1, 0.5)),
            ("B", (0.4, -0.2, 0.3)),
            ("C", (0.0, 0.35, 0.6)),
            ("D (too far)", (0.8, 0.0, 0.85)),
        ]
    else:
        ap.error("give exactly three numbers: X Y Z  (or none for the demo)")

    cases = []
    for name, target in targets:
        joints6, achieved, error, solution = solve_ik(chain, target)
        report(target, joints6, achieved, error)
        cases.append((f"{name}: err {error * 1000:.1f} mm", target, solution))

    print("\nDone. Opening the 3D view... close the window to finish.\n")
    show(chain, cases)


if __name__ == "__main__":
    main()
