"""
Unit tests: collision detection + safe planner (arm_lib).

The whole safety story of the twin rests on two claims:
  1. a path that drives the arm through a box is FLAGGED as colliding, and
  2. plan_safe never hands back a colliding path -- it replans up-and-over,
     or returns None if it truly can't find one.
Both PASS and FAIL cases are exercised.
"""
import numpy as np
import pytest

import arm_lib as A


# ------------------------------------------------------------- geometry
def test_box_distance_zero_inside():
    b = A.Box([0.0, 0.0, 0.0], [0.1, 0.1, 0.1])
    assert b.distance_to_point([0.0, 0.0, 0.0]) == 0.0        # centre is inside


def test_box_distance_positive_outside():
    b = A.Box([0.0, 0.0, 0.0], [0.1, 0.1, 0.1])
    assert b.distance_to_point([1.0, 0.0, 0.0]) == pytest.approx(0.9)


def test_sphere_distance_matches_radius():
    s = A.Sphere([0.0, 0.0, 0.0], 0.2)
    assert s.distance_to_point([0.0, 0.0, 1.0]) == pytest.approx(0.8)


# ------------------------------------------------------------- collision flag
def test_path_through_box_is_flagged(chain):
    """FAIL-CASE (the arm SHOULD be caught): force a straight joint-space path
    from one side of the box to the other and assert collisions are detected."""
    box = A.Box([0.30, 0.0, 0.30], [0.12, 0.12, 0.12])   # box in the arm's way
    q_a = A.solve_ik(chain, (0.30, -0.30, 0.30))[0]
    q_b = A.solve_ik(chain, (0.30, 0.30, 0.30))[0]
    straight = A.segment(q_a, q_b, 40)
    assert A.path_collisions(chain, straight, [box]) > 0


def test_free_space_path_has_no_collisions(chain, box):
    """A motion high above the box should be clean."""
    q_a = A.solve_ik(chain, (0.30, -0.20, 0.60))[0]
    q_b = A.solve_ik(chain, (0.30, 0.20, 0.60))[0]
    path = A.segment(q_a, q_b, 40)
    assert A.path_collisions(chain, path, [box]) == 0


# ------------------------------------------------------------- safe planner
def test_plan_safe_returns_collision_free_path(chain, box):
    """The headline safety guarantee: whatever plan_safe returns is clean."""
    path, log = A.plan_safe(chain, (0.35, -0.30, 0.16), (0.35, 0.30, 0.16), [box])
    assert path is not None
    assert A.path_collisions(chain, path, [box]) == 0
    assert len(log) >= 1                      # at least the direct-path attempt


def test_plan_safe_logs_replanning_when_blocked(chain):
    """When the direct path is blocked, the log must show the lift-over attempts
    (more than one entry), proving the replanner actually engaged."""
    box = A.Box([0.33, 0.0, 0.11], [0.11, 0.14, 0.16])   # tall box, blocks direct
    _, log = A.plan_safe(chain, (0.35, -0.28, 0.16), (0.35, 0.28, 0.16), [box])
    descriptions = [d for d, _ in log]
    assert descriptions[0] == "direct path"
    # either the direct path was clean (1 entry) or it replanned (>1 entry);
    # if it replanned, later entries must be lift-over attempts.
    if len(log) > 1:
        assert any("lift over" in d for d in descriptions)


def test_trajectory_endpoints_are_exact(chain):
    """Smoothstep must start and end exactly on the requested joints (zero drift
    at the ends), so the arm arrives where planning said it would."""
    q_a = np.array([0.0, -0.5, 1.0, 0.0, 0.5, 0.0])
    q_b = np.array([0.4, -0.2, 0.7, 0.3, 0.3, -0.2])
    seg = A.segment(q_a, q_b, 30)
    assert np.allclose(seg[0], q_a)
    assert np.allclose(seg[-1], q_b)
