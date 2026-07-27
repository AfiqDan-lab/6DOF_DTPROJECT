#!/usr/bin/env python3
"""
view_arm.py  -  load the 6-DOF arm, print its joints, run forward kinematics,
and show it in an interactive 3D window.

Uses ikpy (pure Python) instead of pybullet, so it installs and runs on any
Python version -- including 3.14 -- with no C++ compiler needed.

    python -m pip install ikpy matplotlib
    python scripts/view_arm.py

A 3D window opens showing the arm at its home pose and a demo pose. Drag to
rotate, scroll to zoom. Close the window to exit.

Forward kinematics (FK) = "given the six joint angles, where does the tool tip
end up?" Next phase is inverse kinematics (IK): the reverse -- "given a target
point, what six angles get us there?"
"""
import os
import warnings

import numpy as np

# ikpy warns that the fixed base/tool links are "active"; harmless, so hush it.
warnings.filterwarnings("ignore", message=".*fixed.*active_links_mask.*")

from ikpy.chain import Chain  # noqa: E402

URDF_PATH = os.path.join(os.path.dirname(__file__), "..", "urdf", "arm6dof.urdf")

# Explicit path from the base to the TCP, so ikpy follows the arm (and not the
# gripper's finger branch). The 6 revolute joints sit between the fixed ends.
CHAIN_PATH = [
    "base_link",
    "joint_1", "link_1", "joint_2", "link_2", "joint_3", "link_3",
    "joint_4", "link_4", "joint_5", "link_5", "joint_6", "link_6",
    "gripper_mount", "gripper_base", "tcp_fixed", "tcp",
]
# Which links actually move (the 6 revolute joints); the rest are fixed frames.
ACTIVE_MASK = [False, True, True, True, True, True, True, False, False]


def load_arm() -> Chain:
    return Chain.from_urdf_file(URDF_PATH, base_elements=CHAIN_PATH,
                                active_links_mask=ACTIVE_MASK)


def full_vector(joints6):
    """ikpy wants one value per link; pad the two trailing fixed links with 0."""
    return [0.0] + list(joints6) + [0.0, 0.0]


def tcp_position(chain: Chain, joints6):
    T = chain.forward_kinematics(full_vector(joints6))
    return T[:3, 3]  # x, y, z of the tool tip in metres


def main():
    chain = load_arm()

    # ---- list the movable joints and their limits ----
    print(f"\nLoaded arm from {os.path.abspath(URDF_PATH)}")
    print("\nMovable joints (6):")
    jn = 1
    for link in chain.links:
        if getattr(link, "joint_type", "fixed") == "revolute":
            low, high = link.bounds
            print(f"  joint_{jn}: revolute   limits [{low:+.2f}, {high:+.2f}] rad")
            jn += 1

    # ---- forward-kinematics checks ----
    home = [0, 0, 0, 0, 0, 0]
    demo = [0.5, -0.6, 1.0, 0.0, 0.8, 0.0]
    for name, q in [("home", home), ("demo", demo)]:
        p = tcp_position(chain, q)
        print(f"\n{name} pose  angles(rad)={[round(a,2) for a in q]}")
        print(f"  -> TCP  x={p[0]:+.3f}  y={p[1]:+.3f}  z={p[2]:+.3f}  (metres)")

    print("\nFK works. Phase 0 is done -> next is Phase 1 (inverse kinematics).")
    print("Opening the 3D view... drag to rotate, close the window to finish.\n")

    # ---- interactive 3D view: home (left) + demo (right) ----
    import matplotlib.pyplot as plt  # imported here so the prints show first

    fig = plt.figure(figsize=(11, 5))
    for k, (name, q) in enumerate([("home pose", home), ("demo pose", demo)]):
        ax = fig.add_subplot(1, 2, k + 1, projection="3d")
        chain.plot(full_vector(q), ax)
        ax.set_title(name)
        ax.set_xlim(-0.6, 0.6)
        ax.set_ylim(-0.6, 0.6)
        ax.set_zlim(0, 1.0)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_zlabel("z (m)")
    fig.suptitle("6-DOF arm  -  Phase 0")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
