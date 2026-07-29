#!/usr/bin/env python3
"""
virtual_robot.py  -  Phase 5a: a stand-in for the physical arm (no hardware).

It behaves exactly like the ESP32 firmware will: connect to the MQTT broker,
listen for joint commands from the twin on 'arm/cmd', "move" its servos to
follow them, and report measured joint angles + motor telemetry back on
'arm/feedback'. This lets you test the entire twin<->robot link today; when
your real ESP32 arrives, it publishes/subscribes the same topics and this
script is simply switched off.

    docker compose up -d                 # the Mosquitto broker (from Phase 0)
    python -m pip install paho-mqtt
    python scripts/virtual_robot.py      # leave running; then run robot_link.py

Ctrl+C to stop.
"""
import json
import os
import time

import numpy as np
import paho.mqtt.client as mqtt

BROKER_HOST = os.environ.get("BROKER_HOST", "localhost")   # localhost normally; service name in Docker
BROKER_PORT = int(os.environ.get("BROKER_PORT", "1883"))
T_CMD = "arm/cmd"           # twin -> robot : joint commands
T_FEEDBACK = "arm/feedback" # robot -> twin : measured joints + telemetry
T_STATUS = "arm/status"     # robot -> twin : online/offline (retained)

MAX_STEP_DEG = 8.0          # servo slew limit per update (mimics real dynamics)
NAME = "virtual_robot"


class VirtualRobot:
    def __init__(self):
        self.joints = [0.0] * 6   # current servo angles (degrees)
        self.temp = 25.0
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=NAME)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        # if this process dies, the broker tells the twin we went offline
        self.client.will_set(T_STATUS, json.dumps({"online": False, "name": NAME}),
                             qos=1, retain=True)

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        print(f"Connected to broker. Listening for commands on '{T_CMD}'.")
        client.subscribe(T_CMD, qos=0)
        client.publish(T_STATUS, json.dumps({"online": True, "name": NAME}),
                       qos=1, retain=True)

    def _on_message(self, client, userdata, msg):
        cmd = json.loads(msg.payload)
        target = cmd["joints_deg"]

        # move each servo toward its target, limited by the slew rate
        new = []
        for cur, tgt in zip(self.joints, target):
            d = max(-MAX_STEP_DEG, min(MAX_STEP_DEG, tgt - cur))
            new.append(cur + d)
        speed = float(np.linalg.norm(np.array(new) - np.array(self.joints)))
        self.joints = new

        # simulated motor telemetry + noisy "encoder" reading
        current = round(0.4 + 0.15 * speed, 2)
        self.temp = round(self.temp + 0.05 * current - 0.02 * (self.temp - 25.0), 1)
        measured = [round(j + np.random.uniform(-0.3, 0.3), 2) for j in self.joints]

        client.publish(T_FEEDBACK, json.dumps({
            "seq": cmd.get("seq"),
            "joints_deg": measured,
            "gripper": cmd.get("gripper", 0),
            "temp_c": self.temp,
            "current_a": current,
            "t": time.time(),
            "status": "ok",
        }), qos=0)

    def run(self):
        self.client.connect(BROKER_HOST, BROKER_PORT, 60)
        try:
            self.client.loop_forever()
        except KeyboardInterrupt:
            self.client.publish(T_STATUS, json.dumps({"online": False, "name": NAME}),
                                qos=1, retain=True)
            self.client.disconnect()
            print("\nVirtual robot offline.")


if __name__ == "__main__":
    VirtualRobot().run()
