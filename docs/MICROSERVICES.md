# Microservice Architecture & Interface Contracts

This document partitions the 6-DOF digital twin into microservices and specifies
the exact contract for every communicating pair: route/topic, port, protocol,
data format, and when communication starts and ends. It is the reference for the
Project Deployment deliverable.

---

## 1. Service inventory

| # | Service | Container / image | Language & entry | Role (single responsibility) |
|---|---------|-------------------|------------------|------------------------------|
| 1 | **MQTT broker** | `eclipse-mosquitto:2` (`dt_mosquitto`) | — | Message bus for twin ↔ robot commands and feedback |
| 2 | **InfluxDB** | `influxdb:2` (`dt_influxdb`) | — | Time-series persistence of all telemetry |
| 3 | **Grafana** | `grafana/grafana-oss` (`dt_grafana`) | — | Live dashboards reading from InfluxDB |
| 4 | **Twin** | `sixdof/twin` (`dt_twin`) | Python / `run_live.py` | Plans safe motion, produces live state + telemetry |
| 5 | **DB-logger** | `sixdof/logger` | Python / `db_logger.py` | Bridges the twin's ZMQ stream into InfluxDB |
| 6 | **Virtual-robot** | `sixdof/robot` | Python / `virtual_robot.py` | Stand-in for the physical ESP32 arm (scalable) |

The **NLP interpreter** and the **kinematics/planning toolkit** (`arm_lib.py`)
are in-process libraries imported by the Twin, not separate network services;
they have no wire contract and are covered by the unit-test suite instead.

---

## 2. Communication contracts

Each row is a directed contract between two services. "Opens / closes" states
when the channel is established and torn down.

### C1 — Twin → Robot: joint commands
| Field | Value |
|---|---|
| Producer → Consumer | Twin (`robot_link.py`) → Virtual-robot |
| Protocol | MQTT (QoS 0) |
| Broker / port | `mosquitto:1883` |
| Topic (route) | `arm/cmd` |
| Data format | JSON `{ "seq": int, "joints_deg": [6 × float], "gripper": int, "t": float }` |
| Opens | When the twin has a planned trajectory to send |
| Closes | After the final waypoint is published; MQTT session ends on disconnect |
| Frequency | ~25 messages/s (one per waypoint) |

### C2 — Robot → Twin: measured feedback
| Field | Value |
|---|---|
| Producer → Consumer | Virtual-robot → Twin |
| Protocol | MQTT (QoS 0) |
| Broker / port | `mosquitto:1883` |
| Topic (route) | `arm/feedback` |
| Data format | JSON `{ "seq": int, "joints_deg": [6 × float], "gripper": int, "temp_c": float, "current_a": float, "t": float, "status": str }` |
| Opens | On receipt of each `arm/cmd` message |
| Closes | One feedback per command; session ends on disconnect |

### C3 — Robot → Twin: liveness status
| Field | Value |
|---|---|
| Producer → Consumer | Virtual-robot → Twin |
| Protocol | MQTT (QoS 1, **retained**, Last-Will-and-Testament) |
| Broker / port | `mosquitto:1883` |
| Topic (route) | `arm/status` |
| Data format | JSON `{ "online": bool, "name": str }` |
| Opens | On robot connect (`online:true`, retained) |
| Closes | On clean disconnect or broker-detected drop → LWT publishes `online:false` |

### C4 — Twin → DB-logger: live state stream
| Field | Value |
|---|---|
| Producer → Consumer | Twin (`twin_stream.py`) → DB-logger |
| Protocol | ZeroMQ PUB/SUB |
| Endpoint / port | `tcp://twin:5556` |
| Route | ZMQ topic-less SUB (subscribe to all) |
| Data format | JSON per message: `{ "t": float, "step": int, "joint_deg": [6], "tcp": {"x","y","z"}, "collision": bool, "ik_ok": bool, "motor_current_a": float, "motor_temp_c": float }` |
| Opens | Twin binds PUB at startup; logger connects SUB (either order) |
| Closes | On process stop; PUB/SUB is connectionless and resumes if either restarts |
| Frequency | 25 Hz |

### C5 — DB-logger → InfluxDB: persistence write
| Field | Value |
|---|---|
| Producer → Consumer | DB-logger (also Twin's `run_live.py`) → InfluxDB |
| Protocol | HTTP (InfluxDB v2 write API, line protocol) |
| Endpoint / port | `http://influxdb:8086` |
| Route | `POST /api/v2/write?org=sixdof&bucket=robot_telemetry` |
| Auth | Token `dev-token-change-me` (dev only — rotate for real use) |
| Data format | Measurement `arm_state`, tag `source=twin`, fields `j1..j6, x, y, z, collision, ik_ok, current_a, temp_c, step` |
| Opens | Batched write every ~25 states (~1 s) |
| Closes | On flush; client closed at shutdown |

### C6 — Grafana → InfluxDB: dashboard queries
| Field | Value |
|---|---|
| Producer → Consumer | Grafana → InfluxDB |
| Protocol | HTTP (Flux query API) |
| Endpoint / port | `http://influxdb:8086` |
| Route | `POST /api/v2/query?org=sixdof` |
| Query language | **Flux** |
| Data format | Flux query text → tabular result of `arm_state` fields |
| Opens | On dashboard refresh (every 5 s when live) |
| Closes | Per-request (stateless HTTP) |

> **Networking note:** inside Docker, services address each other by service
> name (`influxdb`, `mosquitto`), **not** `localhost`. `localhost` only works
> for tools on the host. This is the single most common Grafana "no data"
> cause and is called out in the README troubleshooting.

---

## 3. Data-flow summary

```
 User ── English ──▶ Twin ──(C4 ZMQ 25Hz)──▶ DB-logger ──(C5 HTTP)──▶ InfluxDB ──(C6 Flux)──▶ Grafana
                     │  ▲                                                  ▲
        (C1 arm/cmd) │  │ (C2 arm/feedback, C3 arm/status)                │
                     ▼  │                                        (persistent 30-day volume)
                Virtual-robot  ───────────────────────────────────────────┘
                  (× N replicas, scalable)
```

---

## 4. Scaling & persistence

- **Scaling.** `virtual-robot` is stateless (it holds only its own servo state,
  seeded on connect), so it can run as N replicas:
  `docker compose -f deploy/docker-compose.full.yml up -d --scale robot=3`.
  The broker fans `arm/cmd` out to all replicas; each answers on `arm/feedback`.
- **Persistence.** InfluxDB writes to the named volume `influxdb_data` with a
  30-day retention. Restarting or recreating the container preserves history —
  the persistence test in §5 proves it.

---

## 5. Test matrix (which test covers which contract)

| Contract / concern | Test |
|---|---|
| C1 command message shape | `tests/test_integration.py::test_command_message_contract` |
| C2 feedback message shape | `tests/test_integration.py::test_feedback_message_contract` |
| C4/C5 telemetry ↔ Influx schema | `tests/test_integration.py::test_telemetry_fields_match_influx_schema` |
| C1/C2 live transport (broker up) | `tests/test_integration.py::test_live_mqtt_round_trip` |
| Full command → plan → execute | `tests/test_integration.py::test_command_to_executable_plan` |
| Kinematics accuracy | `tests/test_kinematics.py` |
| Collision / safe planning | `tests/test_planning.py` |
| NLP parsing & clamping | `tests/test_nlp_command.py` |
| Every image still builds | CI job `build-images` |
| Persistence across restart | Manual, see `docs/DEPLOYMENT.md` §4 |
| Scaling to N replicas | Manual, see `docs/DEPLOYMENT.md` §5 |
