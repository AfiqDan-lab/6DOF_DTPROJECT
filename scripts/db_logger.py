#!/usr/bin/env python3
"""
db_logger.py  -  Phase 3b: write the twin's state stream into InfluxDB.

Same listener as monitor.py, but instead of printing to screen it stores every
state in the InfluxDB time-series database from your Phase 0 Docker stack. This
is the "Digital Twin -> Database" arrow in your communication table.

Run order:
    1) docker compose up -d            # InfluxDB must be running
    2) python scripts/twin_stream.py   # terminal 1 (the twin)
    3) python scripts/db_logger.py     # terminal 2 (this)

    python -m pip install pyzmq influxdb-client

Then verify with:  python scripts/query_influx.py
Press Ctrl+C to stop.
"""
import zmq

from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS

# ---- ZMQ (must match twin_stream.py) ----
ADDR = "tcp://127.0.0.1:5556"

# ---- InfluxDB (must match docker-compose.yml) ----
INFLUX_URL = "http://localhost:8086"
INFLUX_TOKEN = "dev-token-change-me"
INFLUX_ORG = "sixdof"
INFLUX_BUCKET = "robot_telemetry"

BATCH = 25   # write to the DB once per ~second (25 states)


def state_to_point(s):
    j = s["joint_deg"]
    tcp = s["tcp"]
    return (Point("arm_state")
            .tag("source", "twin")
            .field("j1", j[0]).field("j2", j[1]).field("j3", j[2])
            .field("j4", j[3]).field("j5", j[4]).field("j6", j[5])
            .field("x", tcp["x"]).field("y", tcp["y"]).field("z", tcp["z"])
            .field("collision", int(s["collision"]))
            .field("ik_ok", int(s["ik_ok"]))
            .field("current_a", s["motor_current_a"])
            .field("temp_c", s["motor_temp_c"])
            .field("step", s["step"])
            .time(int(s["t"] * 1e9), WritePrecision.NS))


def main():
    # connect to InfluxDB and check it's actually reachable
    client = InfluxDBClient(url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG)
    try:
        if not client.ping():
            raise RuntimeError("ping returned false")
    except Exception as e:
        print("Could not reach InfluxDB at", INFLUX_URL)
        print("  ->", e)
        print("  Is the Docker stack running?  Try:  docker compose up -d")
        print("  Also check the token/org/bucket here match docker-compose.yml.")
        client.close()
        return
    write_api = client.write_api(write_options=SYNCHRONOUS)
    print(f"Connected to InfluxDB (bucket '{INFLUX_BUCKET}'). Writing the "
          f"twin's stream...\n")

    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect(ADDR)
    sub.setsockopt_string(zmq.SUBSCRIBE, "")

    buffer, total = [], 0
    try:
        while True:
            s = sub.recv_json()
            buffer.append(state_to_point(s))
            if len(buffer) >= BATCH:
                write_api.write(bucket=INFLUX_BUCKET, org=INFLUX_ORG, record=buffer)
                total += len(buffer)
                buffer = []
                print(f"  wrote {total} states to InfluxDB so far...")
    except KeyboardInterrupt:
        if buffer:
            write_api.write(bucket=INFLUX_BUCKET, org=INFLUX_ORG, record=buffer)
            total += len(buffer)
        print(f"\nStopping logger. Wrote {total} states in total.")
    finally:
        sub.close()
        ctx.term()
        client.close()


if __name__ == "__main__":
    main()
