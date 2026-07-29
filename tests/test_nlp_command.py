"""
Unit tests: natural-language command parsing (nlp_command).

Tests target the built-in rule parser and the clamp/resolve helpers, which run
deterministically with no Ollama/LLM present -- exactly the environment CI runs
in. (The LLM path is an optional enhancement; the fallback is what must always
work, so that is what we pin down.)
"""
import pytest

import nlp_command as N


# ------------------------------------------------------------- named locations
@pytest.mark.parametrize("phrase,expected", [
    ("move to the left bin",  N.NAMED_LOCATIONS["left bin"]),
    ("go to the right bin",   N.NAMED_LOCATIONS["right bin"]),
    ("return home",           N.NAMED_LOCATIONS["home"]),
    ("go to the drop zone",   N.NAMED_LOCATIONS["drop zone"]),
])
def test_named_locations_resolve(phrase, expected):
    r = N.rule_interpret(phrase)
    assert (r["x"], r["y"], r["z"]) == expected


# ------------------------------------------------------------- explicit coords
def test_explicit_coordinates_parsed():
    r = N.rule_interpret("go to x 0.3 y 0.1 z 0.4")
    assert (r["x"], r["y"], r["z"]) == (0.3, 0.1, 0.4)


def test_explicit_coordinates_with_equals_and_negatives():
    r = N.rule_interpret("x=0.25, y=-0.2, z=0.45")
    assert (r["x"], r["y"], r["z"]) == (0.25, -0.2, 0.45)


# ------------------------------------------------------------- relative words
def test_left_is_positive_y():
    """Project axis convention: LEFT = +y. Guards against an axis-flip regression."""
    base = N.NAMED_LOCATIONS["center"]
    r = N.rule_interpret("shift left")
    assert r["y"] > base[1]


def test_right_is_negative_y():
    base = N.NAMED_LOCATIONS["center"]
    r = N.rule_interpret("nudge right")
    assert r["y"] < base[1]


def test_up_increases_z():
    base = N.NAMED_LOCATIONS["center"]
    r = N.rule_interpret("reach up high")
    assert r["z"] > base[2]


# ------------------------------------------------------------- clamping (safety)
def test_clamp_keeps_targets_inside_workspace():
    """FAIL-CASE made safe: an out-of-range request is clamped to the reachable
    envelope rather than being sent to the arm as-is."""
    x, y, z = N.clamp(0.9, 0.9, 0.9)
    assert x <= N.WORKSPACE["x"][1]
    assert y <= N.WORKSPACE["y"][1]
    assert z <= N.WORKSPACE["z"][1]


def test_clamp_lower_bound():
    x, y, z = N.clamp(-1.0, -1.0, -1.0)
    assert x >= N.WORKSPACE["x"][0]
    assert y >= N.WORKSPACE["y"][0]
    assert z >= N.WORKSPACE["z"][0]


# ------------------------------------------------------------- resolve_target
def test_resolve_target_rejects_garbage():
    """Unparseable input yields None so the caller can refuse it."""
    assert N.resolve_target(None) is None
    assert N.resolve_target({"foo": 1}) is None


def test_resolve_target_accepts_location_name():
    assert N.resolve_target({"location": "home"}) == N.NAMED_LOCATIONS["home"]


# ------------------------------------------------------------- full entry point
def test_interpret_output_shape_and_bounds():
    """interpret() must always return clamped, rounded x/y/z plus the method
    used, whichever interpreter ran."""
    r = N.interpret("go to x 0.9 y 0.0 z 0.4")   # x out of range -> clamped
    assert set(r) == {"x", "y", "z", "method"}
    assert r["x"] <= N.WORKSPACE["x"][1]
    assert isinstance(r["method"], str)
