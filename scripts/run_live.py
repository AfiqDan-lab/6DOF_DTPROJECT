#!/usr/bin/env python3
"""
run_live.py  -  one-command live twin for a demo.

Continuously drives the arm through a tour of targets (planning a safe path
for each leg) and writes its live state straight into InfluxDB at ~25 Hz, so
your Grafana dashboard updates in real time. Run this alongside Omniverse
(playing the arm clip) to have the whole system live at once:

    Omniverse  = the arm, rendered          (play eval_showcase.usda)
    Grafana    = live telemetry graphs       (http://localhost:3000)
    InfluxDB   = the stored history          (this script writes it)

    docker compose up -d
    python -m pip install influxdb-client
    python scripts/run_live.py

It replaces running twin_stream.py + db_logger.py separately (no ZMQ needed
here, since nothing is subscribing). Ctrl+C to stop.
"""
import os
import time

import numpy as np

import arm_lib as A
from nlp_command import NAMED_LOCATIONS
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS

# InfluxDB (must match docker-compose.yml)
INFLUX_URL = os.environ.get("INFLUX_URL", "http://localhost:8086")
INFLUX_TOKEN = os.environ.get("INFLUX_TOKEN", "dev-token-change-me")
INFLUX_ORG = os.environ.get("INFLUX_ORG", "sixdof")
INFLUX_BUCKET = os.environ.get("INFLUX_BUCKET", "robot_telemetry")

RATE_HZ = 25
BATCH = 25
OBSTACLES = [A.Box([0.33, 0.0, 0.11], [0.09, 0.11, 0.11])]

TOUR = [
    NAMED_LOCATIONS["home"],
    NAMED_LOCATIONS["left bin"],
    NAMED_LOCATIONS["right bin"],
    (0.30, 0.00, 0.65),
    NAMED_LOCATIONS["drop zone"],
    (0.45, 0.00, 0.40),
    (0.30, -0.25, 0.25),
]


def motion_stream(chain):
    """Infinite generator of joint configs cycling through the tour."""
    while True:
        for a, b in zip(TOUR, TOUR[1:] + TOUR[:1]):
            path, _ = A.plan_safe(chain, a, b, OBSTACLES)
            if path:
                for q in path:
                    yield q


def make_telemetry():
    state = {"prev": None, "temp": 25.0}
    dt = 1.0 / RATE_HZ

    def step(q):
        j = np.asarray(q)
        speed = 0.0 if state["prev"] is None else np.linalg.norm(j - state["prev"]) / dt
        state["prev"] = j
        current = 0.5 + 2.0 * speed
        state["temp"] += 0.3 * current - 0.05 * (state["temp"] - 25.0)
        return round(float(current), 2), round(float(state["temp"]), 1)
    return step


def state_point(chain, q, step_i, telemetry):
    tcp = A.tcp_position(chain, q)
    cur, temp = telemetry(q)
    deg = [round(float(np.degrees(a)), 2) for a in q]
    colliding = A.config_collides(chain, q, OBSTACLES)
    return (Point("arm_state").tag("source", "twin")
            .field("j1", deg[0]).field("j2", deg[1]).field("j3", deg[2])
            .field("j4", deg[3]).field("j5", deg[4]).field("j6", deg[5])
            .field("x", round(float(tcp[0]), 4)).field("y", round(float(tcp[1]), 4))
            .field("z", round(float(tcp[2]), 4))
            .field("collision", int(colliding)).field("ik_ok", 1)
            .field("current_a", cur).field("temp_c", temp).field("step", step_i)
            .time(int(time.time() * 1e9), WritePrecision.NS))


def main():
    client = InfluxDBClient(url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG)
    try:
        if not client.ping():
            raise RuntimeError("ping returned false")
    except Exception as e:
        print("Could not reach InfluxDB at", INFLUX_URL, "->", e)
        print("Is the Docker stack running?  docker compose up -d")
        print("Do the token/org/bucket here match docker-compose.yml?")
        client.close()
        return
    write_api = client.write_api(write_options=SYNCHRONOUS)
    print(f"Live twin running -> InfluxDB bucket '{INFLUX_BUCKET}'. "
          f"Open Grafana at http://localhost:3000. Ctrl+C to stop.\n")

    chain = A.load_arm()
    telemetry = make_telemetry()
    stream = motion_stream(chain)

    buffer, total, i = [], 0, 0
    period = 1.0 / RATE_HZ
    try:
        while True:
            q = next(stream)
            buffer.append(state_point(chain, q, i, telemetry))
            if len(buffer) >= BATCH:
                write_api.write(bucket=INFLUX_BUCKET, org=INFLUX_ORG, record=buffer)
                total += len(buffer)
                buffer = []
                print(f"  streaming... {total} states written")
            i += 1
            time.sleep(period)
    except KeyboardInterrupt:
        if buffer:
            write_api.write(bucket=INFLUX_BUCKET, org=INFLUX_ORG, record=buffer)
            total += len(buffer)
        print(f"\nStopped. Wrote {total} states.")
    finally:
        client.close()


if __name__ == "__main__":
    main()
