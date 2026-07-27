#!/usr/bin/env python3
"""
command_arm.py  -  Phase 4b: drive the arm with plain-English commands.

This is the whole flowchart in one loop:

    you type a sentence
      -> interpret() turns it into a target (LLM or fallback)   [Phase 4a]
      -> reachability check                                     [Phase 1]
      -> plan_safe() finds a collision-free path                [Phase 2]
      -> the arm animates to it in 3D
      -> (optional) state streams to InfluxDB/Grafana           [Phase 3]

    python scripts/command_arm.py            # 3D window + type commands
    python scripts/command_arm.py --stream   # also broadcast state over ZMQ

With --stream, run db_logger.py in another terminal to log to InfluxDB and
watch your Grafana dashboard update as the arm moves.

Type commands like: "move to the left bin", "go to the right bin",
"return home", "reach up high", "go to x 0.3 y 0.1 z 0.4". Blank line quits.
"""
import argparse
import time

import numpy as np

import arm_lib as A
from nlp_command import interpret, NAMED_LOCATIONS

# scene: one box sitting on the table (same obstacle as earlier phases)
OBSTACLES = [A.Box(center=[0.33, 0.0, 0.11], half_extents=[0.09, 0.11, 0.11],
                   name="box on table")]
REACH_TOL = 0.01   # 1 cm: matches the proposal's accuracy target


def plan_command(chain, cmd, current_xyz):
    """Interpret a command and plan a safe path from current_xyz to it."""
    r = interpret(cmd)
    target = (r["x"], r["y"], r["z"])
    info = {"target": target, "method": r["method"]}

    _, _, err = A.solve_ik(chain, target)
    if err > REACH_TOL:
        return {**info, "ok": False, "reason": f"out of reach ({err*1000:.0f} mm short)"}

    path, log = A.plan_safe(chain, current_xyz, target, OBSTACLES)
    if path is None:
        return {**info, "ok": False, "reason": "no collision-free path found"}
    return {**info, "ok": True, "path": path, "log": log}


def make_telemetry(rate_hz):
    state = {"prev": None, "temp": 25.0}
    dt = 1.0 / rate_hz

    def step(q):
        j = np.asarray(q)
        speed = 0.0 if state["prev"] is None else np.linalg.norm(j - state["prev"]) / dt
        state["prev"] = j
        current = 0.5 + 2.0 * speed
        state["temp"] += 0.3 * current - 0.05 * (state["temp"] - 25.0)
        return round(float(current), 2), round(float(state["temp"]), 1)
    return step


def main():
    ap = argparse.ArgumentParser(description="Drive the arm with English commands.")
    ap.add_argument("--stream", action="store_true",
                    help="also publish state over ZMQ (for db_logger/Grafana)")
    args = ap.parse_args()

    chain = A.load_arm()
    current_xyz = NAMED_LOCATIONS["home"]
    rate_hz = 30
    telemetry = make_telemetry(rate_hz)

    # optional ZMQ publisher (same channel/schema as twin_stream.py)
    pub = None
    if args.stream:
        import zmq
        ctx = zmq.Context()
        pub = ctx.socket(zmq.PUB)
        pub.bind("tcp://127.0.0.1:5556")
        time.sleep(0.3)
        print("Streaming state on tcp://127.0.0.1:5556 (run db_logger.py to log it).")

    import matplotlib.pyplot as plt
    plt.ion()
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")

    def render(q, target=None, trail=None, title="", colliding=False):
        ax.cla()
        A.draw_arm(ax, chain, q, "red" if colliding else "steelblue")
        for o in OBSTACLES:
            o.draw(ax)
        if trail is not None and len(trail):
            t = np.array(trail)
            ax.plot(t[:, 0], t[:, 1], t[:, 2], color="green", lw=2)
        if target is not None:
            ax.scatter(*target, c="green", marker="*", s=110)
        ax.set_xlim(0, 0.6); ax.set_ylim(-0.4, 0.4); ax.set_zlim(0, 0.9)
        ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
        ax.set_title(title)

    # show the starting pose
    q_now = A.solve_ik(chain, current_xyz)[0]
    render(q_now, title="ready - type a command in the terminal")
    plt.pause(0.01)

    print("\nType a command (blank line or Ctrl+C to quit).")
    print('Examples: "move to the left bin", "go to the right bin", '
          '"return home", "reach up high"\n')

    step_i = 0
    try:
        while True:
            cmd = input("command> ").strip()
            if not cmd:
                break

            res = plan_command(chain, cmd, current_xyz)
            tx, ty, tz = res["target"]
            print(f"  interpreted [{res['method']}] -> "
                  f"({tx:+.2f}, {ty:+.2f}, {tz:+.2f})")
            if not res["ok"]:
                print(f"  cannot do that: {res['reason']}\n")
                continue

            attempts = len(res["log"]) - 1
            note = ("direct path was clear" if attempts == 0
                    else f"avoided obstacle (replanned {attempts}x, lifted over)")
            print(f"  plan: {note}. Moving...\n")

            path = res["path"]
            trail = []
            period = 1.0 / rate_hz
            for q in path:
                tcp = A.tcp_position(chain, q)
                trail.append(tcp)
                colliding = A.config_collides(chain, q, OBSTACLES)
                render(q, target=res["target"], trail=trail,
                       title=f'"{cmd}"', colliding=colliding)
                if pub is not None:
                    cur, temp = telemetry(q)
                    pub.send_json({
                        "t": time.time(), "step": step_i,
                        "joint_deg": [round(float(np.degrees(a)), 2) for a in q],
                        "tcp": {"x": round(float(tcp[0]), 4),
                                "y": round(float(tcp[1]), 4),
                                "z": round(float(tcp[2]), 4)},
                        "collision": bool(colliding), "ik_ok": True,
                        "motor_current_a": cur, "motor_temp_c": temp,
                    })
                    step_i += 1
                plt.pause(period)

            current_xyz = res["target"]
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        if pub is not None:
            pub.close()
        print("\nDone.")


if __name__ == "__main__":
    main()
