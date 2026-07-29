#!/usr/bin/env python3
"""
robot_link.py  -  Phase 5a: the twin sends a planned motion to the robot.

This is the "Digital Twin -> Physical Robot (Robot Commands, MQTT)" and
"Physical Robot -> Digital Twin (Sensor Feedback, MQTT)" pair from your
communication table. The twin plans a safe, collision-free trajectory (Phases
2 and 4), streams it to the robot over MQTT as joint commands, and reads the
measured joint angles coming back -- exactly what it will do with the real
ESP32.

    docker compose up -d
    python scripts/virtual_robot.py          # terminal 1 (the robot)
    python scripts/robot_link.py             # terminal 2 (the twin) -- demo motion
    python scripts/robot_link.py "reach up high"   # or a command of your own

Press Ctrl+C to stop.
"""
import argparse
import json
import time

import numpy as np
import paho.mqtt.client as mqtt

import arm_lib as A
from nlp_command import interpret, NAMED_LOCATIONS

BROKER_HOST = "localhost"
BROKER_PORT = 1883
T_CMD = "arm/cmd"
T_FEEDBACK = "arm/feedback"
T_STATUS = "arm/status"
RATE_HZ = 25

OBSTACLES = [A.Box(center=[0.33, 0.0, 0.11], half_extents=[0.09, 0.11, 0.11])]

# shared with the MQTT background thread
fb = {"count": 0, "measured": None, "online": None}


def on_message(client, userdata, msg):
    data = json.loads(msg.payload)
    if msg.topic == T_FEEDBACK:
        fb["measured"] = data["joints_deg"]
        fb["count"] += 1
    elif msg.topic == T_STATUS:
        fb["online"] = data.get("online")
        print(f"  [robot status: {'ONLINE' if data.get('online') else 'offline'}]")


def main():
    ap = argparse.ArgumentParser(description="Send a planned motion to the robot.")
    ap.add_argument("command", nargs="*", help="English command (default: a demo move)")
    args = ap.parse_args()
    cmd = " ".join(args.command) if args.command else "move to the left bin"

    # plan: start at the right bin so the path must cross (and avoid) the box
    chain = A.load_arm()
    start = NAMED_LOCATIONS["right bin"]
    r = interpret(cmd)
    target = (r["x"], r["y"], r["z"])
    path, log = A.plan_safe(chain, start, target, OBSTACLES)
    if path is None:
        print("No collision-free path found for that command.")
        return

    attempts = len(log) - 1
    plan_note = "direct path" if attempts == 0 else f"avoided obstacle ({attempts}x lift-over)"
    print(f"\ncommand: \"{cmd}\"  [{r['method']}]  ->  target "
          f"({target[0]:+.2f}, {target[1]:+.2f}, {target[2]:+.2f})")
    print(f"plan: {plan_note}, {len(path)} waypoints\n")

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="twin_link")
    client.on_message = on_message
    client.connect(BROKER_HOST, BROKER_PORT, 60)
    client.subscribe(T_FEEDBACK, qos=0)
    client.subscribe(T_STATUS, qos=0)
    client.loop_start()
    time.sleep(0.5)  # allow status/connection to settle

    print("Streaming trajectory to the robot over MQTT (cmd -> feedback loop):")
    period = 1.0 / RATE_HZ
    for i, q in enumerate(path):
        deg = [round(float(np.degrees(a)), 2) for a in q]
        client.publish(T_CMD, json.dumps({"seq": i, "joints_deg": deg,
                                          "gripper": 0, "t": time.time()}), qos=0)
        if i % 15 == 0 and fb["measured"] is not None:
            m = fb["measured"]
            err = max(abs(a - b) for a, b in zip(deg, m))
            print(f"  wp {i:3d}: cmd j1={deg[0]:+6.1f}  robot j1={m[0]:+6.1f}  "
                  f"(max joint gap {err:.2f} deg)")
        time.sleep(period)

    time.sleep(0.5)  # let the last feedback arrive
    print(f"\nDone. Received {fb['count']} feedback messages from the robot.")
    if fb["measured"] is not None:
        final = [round(float(np.degrees(a)), 2) for a in path[-1]]
        err = max(abs(a - b) for a, b in zip(final, fb["measured"]))
        print(f"Final commanded vs measured: max joint error {err:.2f} deg "
              f"(servo slew + sensor noise).")
    client.loop_stop()
    client.disconnect()


if __name__ == "__main__":
    main()
