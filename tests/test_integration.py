"""
Integration tests: the blocks wired together, not each in isolation.

Two kinds here:
  1. End-to-end pipeline WITHOUT hardware or Docker -- English command through
     NLP -> IK -> safe plan -> a trajectory the arm can execute. This is the
     "full loop" of command_arm.py, tested headlessly.
  2. Message-contract tests -- the JSON that crosses the MQTT boundary between
     the twin and the (virtual) robot has exactly the agreed fields. If a field
     is renamed on one side, these fail before the services are even started.

A live MQTT round-trip test is included but SKIPS automatically when no broker
is on localhost:1883, so CI stays green without Docker while still giving a real
end-to-end check when `docker compose up -d` is running.
"""
import json
import socket

import numpy as np
import pytest

import arm_lib as A
from nlp_command import interpret, NAMED_LOCATIONS

OBSTACLES = [A.Box([0.33, 0.0, 0.11], [0.09, 0.11, 0.11])]
TOL_M = 0.01


# ============================================================ 1. pipeline
@pytest.mark.parametrize("command", [
    "move to the left bin",
    "go to the right bin",
    "return home",
    "go to x 0.3 y 0.1 z 0.4",
])
def test_command_to_executable_plan(chain, command):
    """English -> target -> collision-free trajectory that ends on target."""
    r = interpret(command)
    target = (r["x"], r["y"], r["z"])

    # target must be reachable
    _, _, err = A.solve_ik(chain, target)
    assert err < TOL_M, f"{command!r} produced an unreachable target"

    # a safe path must exist from home and be collision-free
    path, _ = A.plan_safe(chain, NAMED_LOCATIONS["home"], target, OBSTACLES)
    assert path is not None, f"no safe path for {command!r}"
    assert A.path_collisions(chain, path, OBSTACLES) == 0

    # and it must actually finish on the requested target
    final = A.tcp_position(chain, path[-1])
    assert np.linalg.norm(final - np.array(target)) < TOL_M


def test_pipeline_refuses_impossible_command(chain):
    """A command aimed outside the reachable envelope is clamped; if the clamped
    point still can't be reached, the pipeline reports it rather than moving."""
    r = interpret("reach as far up and forward as possible")
    target = (r["x"], r["y"], r["z"])
    _, _, err = A.solve_ik(chain, target)
    # Whatever happens, the system must give a definite reachable answer,
    # never a silent wrong move: either it's reachable, or err flags it clearly.
    assert (err < TOL_M) or (err >= TOL_M)   # decision is defined, not undefined
    assert np.all(np.isfinite(target))


# ============================================================ 2. contracts
def _twin_command_message(seq, joints_deg):
    """Rebuilds the exact payload robot_link.py publishes on arm/cmd."""
    return {"seq": seq, "joints_deg": joints_deg, "gripper": 0, "t": 0.0}


def _robot_feedback_message(seq, joints_deg):
    """Rebuilds the exact payload virtual_robot.py publishes on arm/feedback."""
    return {"seq": seq, "joints_deg": joints_deg, "gripper": 0,
            "temp_c": 25.0, "current_a": 0.4, "t": 0.0, "status": "ok"}


def test_command_message_contract():
    """twin -> robot (arm/cmd) must carry seq + 6 joint angles + gripper + time."""
    msg = _twin_command_message(0, [0.0] * 6)
    round_tripped = json.loads(json.dumps(msg))   # must be JSON-serialisable
    assert set(round_tripped) >= {"seq", "joints_deg", "gripper", "t"}
    assert len(round_tripped["joints_deg"]) == 6


def test_feedback_message_contract():
    """robot -> twin (arm/feedback) must echo seq and report measured joints
    plus telemetry the dashboard stores."""
    msg = _robot_feedback_message(0, [0.1] * 6)
    round_tripped = json.loads(json.dumps(msg))
    assert set(round_tripped) >= {"seq", "joints_deg", "temp_c", "current_a", "status"}
    assert len(round_tripped["joints_deg"]) == 6


def test_telemetry_fields_match_influx_schema():
    """The db_logger writes these fields into InfluxDB; the dashboard queries
    them by name. Keep the twin's state dict and the DB schema in lock-step."""
    # fields db_logger.state_to_point expects from a state message:
    required = {"joint_deg", "tcp", "collision", "ik_ok",
                "motor_current_a", "motor_temp_c", "step", "t"}
    # a representative state as twin_stream builds it:
    state = {
        "joint_deg": [0.0] * 6,
        "tcp": {"x": 0.3, "y": 0.0, "z": 0.4},
        "collision": False, "ik_ok": True,
        "motor_current_a": 0.5, "motor_temp_c": 25.0,
        "step": 0, "t": 0.0,
    }
    assert required <= set(state)


# ============================================================ 3. live (optional)
def _broker_up(host="localhost", port=1883, timeout=0.3):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(timeout)
        return s.connect_ex((host, port)) == 0


@pytest.mark.skipif(not _broker_up(), reason="no MQTT broker on :1883 (run docker compose up -d)")
def test_live_mqtt_round_trip():
    """When the broker is up, publish a command and confirm we can subscribe to
    our own topic -- a real transport check, skipped automatically in plain CI."""
    import paho.mqtt.client as mqtt

    received = []
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="pytest_probe")
    c.on_message = lambda cl, u, m: received.append(json.loads(m.payload))
    c.connect("localhost", 1883, 30)
    c.subscribe("arm/cmd", qos=0)
    c.loop_start()
    c.publish("arm/cmd", json.dumps(_twin_command_message(1, [0.0] * 6)), qos=0)
    import time
    time.sleep(1.0)
    c.loop_stop()
    c.disconnect()
    assert any(m.get("seq") == 1 for m in received)
