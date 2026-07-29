"""
Unit tests: forward & inverse kinematics (arm_lib).

These pin down the twin's most safety-critical numbers -- if IK regresses, the
arm goes to the wrong place. Includes deliberate PASS and FAIL cases:
  - PASS: reachable targets solve to < 1 cm.
  - FAIL (expected): a target far outside the workspace does NOT solve, and the
    solver reports the large error honestly instead of pretending it reached.
"""
import numpy as np
import pytest

import arm_lib as A

TOL_M = 0.01  # 1 cm, the project's position-accuracy target


# ---------------------------------------------------------------- forward K
def test_fk_returns_3d_point(chain):
    p = A.tcp_position(chain, [0.0, -0.5, 1.0, 0.0, 0.5, 0.0])
    assert p.shape == (3,)
    assert np.all(np.isfinite(p))


def test_fk_is_deterministic(chain):
    """Same joints -> same tool position, every time."""
    j = [0.1, -0.4, 0.9, 0.2, 0.4, -0.1]
    a = A.tcp_position(chain, j)
    b = A.tcp_position(chain, j)
    assert np.allclose(a, b)


def test_skeleton_has_expected_frame_count(chain):
    """One point per frame in the kinematic chain; enough to draw the arm."""
    pts = A.skeleton(chain, [0.0] * 6)
    assert len(pts) >= 7          # base + 6 joints at minimum
    assert all(p.shape == (3,) for p in pts)


# ---------------------------------------------------------------- inverse K
@pytest.mark.parametrize("joints", [
    [0.0, -0.5, 1.0, 0.0, 0.5, 0.0],
    [0.3, -0.3, 0.8, 0.1, 0.6, 0.2],
    [-0.4, -0.7, 1.2, -0.2, 0.3, 0.4],
])
def test_ik_round_trip_under_1cm(chain, joints):
    """FK a known config to get a guaranteed-reachable target, then IK it back.
    The recovered tool position must land within the 1 cm accuracy target."""
    target = A.tcp_position(chain, joints)
    _, achieved, err = A.solve_ik(chain, target)
    assert err < TOL_M
    assert np.linalg.norm(achieved - target) < TOL_M


def test_ik_returns_six_joint_angles(chain):
    target = A.tcp_position(chain, [0.2, -0.4, 0.9, 0.0, 0.5, 0.0])
    joints, _, _ = A.solve_ik(chain, target)
    assert len(joints) == 6
    assert all(isinstance(a, float) for a in joints)


def test_ik_far_target_is_reported_not_faked(chain):
    """FAIL-CASE: a target 5 m away is unreachable. The solver must NOT claim
    success -- it should return a large, honest error so callers can refuse."""
    _, _, err = A.solve_ik(chain, (5.0, 5.0, 5.0))
    assert err > TOL_M      # nowhere near reached; caller will reject it
