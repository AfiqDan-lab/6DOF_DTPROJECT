# Deployment Guide & Evidence Checklist

Everything needed to deploy the twin as containerized microservices, scale them,
and prove persistence — with the exact commands to run and the screenshot/log
evidence to capture for the Project Deployment rubric.

Run all commands **from the project root** (`6dof_dt/`).

---

## 1. Build and deploy all microservices

```bash
docker compose -f deploy/docker-compose.full.yml up -d --build
```

This builds the three application images (`twin`, `robot`, and — if enabled —
`logger`) and starts them alongside Mosquitto, InfluxDB, and Grafana.

Confirm everything is up:

```bash
docker compose -f deploy/docker-compose.full.yml ps
```

**📸 Evidence 1:** screenshot of `docker compose ps` showing all containers
`Up`. This is your "successful deployment of microservices."

---

## 2. Health-check the stack

```bash
py -3.11 scripts/check_stack.py        # expect: OK, OK, OK
```

**📸 Evidence 2:** the `OK OK OK` output.

---

## 3. Verify the individual service images exist

```bash
docker images | findstr sixdof        # Windows
docker images | grep  sixdof          # macOS/Linux
```

**📸 Evidence 3:** the three `sixdof/*` images listed. This is
"containerization of individual microservices."

---

## 4. Prove persistence (data survives a restart)

```bash
# 1. write some telemetry (let it run ~15 s, then Ctrl+C)
py -3.11 scripts/run_live.py

# 2. confirm rows are stored
py -3.11 scripts/query_influx.py       # note the row count / latest timestamp

# 3. restart ONLY the database container
docker compose -f deploy/docker-compose.full.yml restart influxdb

# 4. query again AFTER the restart — the data is still there
py -3.11 scripts/query_influx.py
```

**📸 Evidence 4:** the two `query_influx.py` outputs (before and after the
restart) showing the same historical rows. This proves
"data captured and stored is persistent."

---

## 5. Demonstrate scaling

```bash
# run three virtual-robot replicas behind the broker
docker compose -f deploy/docker-compose.full.yml up -d --scale robot=3

# show the three replicas
docker compose -f deploy/docker-compose.full.yml ps robot
```

**📸 Evidence 5:** `ps` output showing `..._robot_1/2/3` all `Up`. This is
"deployment and scaling of the microservices."

To scale back down:

```bash
docker compose -f deploy/docker-compose.full.yml up -d --scale robot=1
```

---

## 6. Run the full test suite against the deployed system

With the broker running, the live-MQTT integration test no longer skips:

```bash
py -3.11 -m pytest -v
```

**📸 Evidence 6:** pytest output showing `test_live_mqtt_round_trip PASSED`
(not skipped) and the full suite green. This is your
"complete test suite testing the microservices and overall digital twin."

---

## 7. Tear down

```bash
# stop containers, keep the data volume
docker compose -f deploy/docker-compose.full.yml down

# stop AND delete the data volume (full reset)
docker compose -f deploy/docker-compose.full.yml down -v
```

---

## Evidence summary (paste these into the report)

| # | Evidence | Rubric line it satisfies |
|---|----------|--------------------------|
| 1 | `docker compose ps` all Up | Deployment of interacting microservices |
| 2 | `check_stack.py` OK×3 | Services reachable |
| 3 | `docker images` sixdof/* | Containerization of individual microservices |
| 4 | query before/after restart | Persistence in data & state storage |
| 5 | `--scale robot=3` ps output | Scaling of microservices |
| 6 | pytest live test passing | Test suite for microservices + full flow |
| — | `docs/MICROSERVICES.md` | Contract: route, port, protocol, format, open/close |
