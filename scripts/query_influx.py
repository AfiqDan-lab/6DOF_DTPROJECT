#!/usr/bin/env python3
"""
query_influx.py  -  Phase 3b check: read recent rows back out of InfluxDB.

Run this after db_logger.py has been writing for a few seconds, to confirm the
twin's data actually landed in the database.

    python scripts/query_influx.py
"""
from influxdb_client import InfluxDBClient

INFLUX_URL = "http://localhost:8086"
INFLUX_TOKEN = "dev-token-change-me"
INFLUX_ORG = "sixdof"
INFLUX_BUCKET = "robot_telemetry"


def main():
    client = InfluxDBClient(url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG)
    try:
        if not client.ping():
            raise RuntimeError("ping returned false")
    except Exception as e:
        print("Could not reach InfluxDB at", INFLUX_URL, "->", e)
        print("Is Docker running?  docker compose up -d")
        client.close()
        return

    q = client.query_api()

    # how many rows in the last 10 minutes?
    count_flux = f'''
from(bucket: "{INFLUX_BUCKET}")
  |> range(start: -10m)
  |> filter(fn: (r) => r._measurement == "arm_state" and r._field == "z")
  |> count()
'''
    tables = q.query(count_flux, org=INFLUX_ORG)
    n = tables[0].records[0].get_value() if tables and tables[0].records else 0
    print(f"\narm_state rows in the last 10 min: {n}")

    if not n:
        print("No data yet. Is twin_stream.py running and db_logger.py writing?")
        client.close()
        return

    # last 5 states, fields pivoted into columns
    last_flux = f'''
from(bucket: "{INFLUX_BUCKET}")
  |> range(start: -10m)
  |> filter(fn: (r) => r._measurement == "arm_state")
  |> pivot(rowKey: ["_time"], columnKey: ["_field"], valueColumn: "_value")
  |> tail(n: 5)
'''
    print("\nMost recent 5 states:")
    for table in q.query(last_flux, org=INFLUX_ORG):
        for r in table.records:
            v = r.values
            print(f"  {r.get_time():%H:%M:%S}  "
                  f"tcp=({v.get('x'):+.2f},{v.get('y'):+.2f},{v.get('z'):+.2f})  "
                  f"temp={v.get('temp_c'):.1f}C  current={v.get('current_a'):.2f}A  "
                  f"collision={int(v.get('collision'))}")
    client.close()


if __name__ == "__main__":
    main()
