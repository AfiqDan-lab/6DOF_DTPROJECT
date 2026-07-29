#!/usr/bin/env python3
"""
twin_stream.py  -  Phase 3a: the twin broadcasts its live state over ZMQ.

The twin plans a safe path (Phase 2c), then "executes" it back and forth
forever, publishing its state ~25 times a second on a ZMQ PUB socket. This is
the "Motion Planner -> Digital Twin (Joint Angles, ZMQ)" and live-state stream
from your proposal's communication table.

Run this in ONE terminal; run monitor.py (or the Phase 3b logger) in another.

    python -m pip install pyzmq          # (installed in Phase 3)
    python scripts/twin_stream.py

Press Ctrl+C to stop.
"""
import json
import time

import numpy as np
import zmq

import arm_lib as A

ADDR = "tcp://127.0.0.1:5556"   # the channel the monitor/logger connects to
RATE_HZ = 25                    # state updates per second (your 25-30 fps target)


def build_loop_path(chain):
    """Plan a safe path over a box, then loop it forward and back."""
    box = A.Box(center=[0.33, 0.0, 0.11], half_extents=[0.09, 0.11, 0.11],
                name="box on table")
    start, goal = (0.35, -0.30, 0.16), (0.35, 0.30, 0.16)
    path, _ = A.plan_safe(chain, start, goal, [box])
    if path is None:
        raise RuntimeError("no safe path found")
    return path + path[::-1], [box]   # there and back, so it repeats smoothly


def make_telemetry():
    """Closure returning simulated motor current & temperature from motion."""
    state = {"prev": None, "temp": 25.0}
    dt = 1.0 / RATE_HZ

    def step(j6):
        j = np.asarray(j6)
        speed = 0.0 if state["prev"] is None else np.linalg.norm(j - state["prev"]) / dt
        state["prev"] = j
        current = 0.5 + 2.0 * speed                       # amps: idle + load
        state["temp"] += 0.3 * current - 0.05 * (state["temp"] - 25.0)  # heat/cool
        return round(float(current), 2), round(float(state["temp"]), 1)

    return step


def main():
    chain = A.load_arm()
    path, obstacles = build_loop_path(chain)
    telemetry = make_telemetry()

    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    pub.bind(ADDR)
    print(f"Twin publishing on {ADDR} at {RATE_HZ} Hz. Ctrl+C to stop.\n")
    time.sleep(0.3)  # let subscribers connect (ZMQ 'slow joiner')

    period = 1.0 / RATE_HZ
    i = 0
    try:
        while True:
            q = path[i % len(path)]
            tcp = A.tcp_position(chain, q)
            current, temp = telemetry(q)
            state = {
                "t": time.time(),
                "step": i,
                "joint_deg": [round(float(np.degrees(a)), 2) for a in q],
                "tcp": {"x": round(float(tcp[0]), 4),
                        "y": round(float(tcp[1]), 4),
                        "z": round(float(tcp[2]), 4)},
                "collision": bool(A.config_collides(chain, q, obstacles)),
                "ik_ok": True,
                "motor_current_a": current,
                "motor_temp_c": temp,
            }
            pub.send_json(state)

            if i % RATE_HZ == 0:  # heartbeat once per second
                print(f"  step {i:5d}  tcp=({state['tcp']['x']:+.2f},"
                      f"{state['tcp']['y']:+.2f},{state['tcp']['z']:+.2f})  "
                      f"temp={temp:.1f}C  current={current:.2f}A")
            i += 1
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nStopping twin.")
    finally:
        pub.close()
        ctx.term()


if __name__ == "__main__":
    main()
