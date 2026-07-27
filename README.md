# 6-DOF Robotic Arm — Digital Twin

A simulated digital twin of a 6-DOF robotic arm. We type English
command, a language model turns it into a target, the twin plans a smooth,
collision-free path, and the arm executes it — while its live state streams to
a database and dashboards, and can be rendered in NVIDIA Omniverse.

Everything here runs **in simulation** .

```
type a command
  -> AI interprets it into a target (x, y, z)
  -> inverse kinematics + smooth trajectory
  -> collision check; if blocked, replan up-and-over
  -> arm executes the motion (3D view)
  -> state streams -> InfluxDB -> Grafana dashboards + Omniverse
```

---

## 1. What is needed

- **Python 3.11** (recommended).
- **Docker Desktop** — runs the MQTT broker, database, and dashboards.
- **Ollama** — for the natural-language AI. Without it, a built-in
  parser is used automatically. https://ollama.com
- **NVIDIA Omniverse** with an RTX GPU — for the photorealistic 3D
  view. The ready-made `.usda` files in `usd/` open with no Python.

Windows note: commands below use `py -3.11` (the Windows Python launcher). On
macOS/Linux use `python3` instead.

---

## 2. Setup (once)

1. **Install Python 3.11.** Windows: get the "Windows installer (64-bit)" from
   https://www.python.org/downloads/release/python-3119/ and tick
   **"Add python.exe to PATH"** during install. Verify:
   ```
   py -3.11 --version        # should print Python 3.11.x
   ```
2. **Install Docker Desktop** and launch it (wait until it says "Engine
   running").
3. **Unzip this project**, then open the `6dof_dt` folder in VS Code
   (File → Open Folder).
4. **Open a terminal** in VS Code (Terminal → New Terminal, or Ctrl+`). Run all
   commands below **from the `6dof_dt` folder**.
5. **Install the Python packages:**
   ```
   py -3.11 -m pip install -r requirements.txt
   ```
6. **Start the backend services** (MQTT + InfluxDB + Grafana):
   ```
   docker compose up -d
   py -3.11 scripts/check_stack.py     # should print OK, OK, OK
   ```

You're set up.

---

## 3. Quick start — see the arm in 2 minutes

```
py -3.11 scripts/view_arm.py
```
A 3D window opens showing the arm and a forward-kinematics check. Then try
inverse kinematics — give it a target and watch it solve:
```
py -3.11 scripts/ik_demo.py 0.3 0.1 0.5
```

---

## 4. Running each part

Run these from the `6dof_dt` folder. Each opens a 3D window unless noted.

| Command | What it shows |
|---|---|
| `py -3.11 scripts/view_arm.py` | Load the arm; forward kinematics |
| `py -3.11 scripts/ik_demo.py` | Inverse kinematics to demo targets (add `X Y Z` for your own) |
| `py -3.11 scripts/move_smooth.py` | Smooth trajectory between two poses |
| `py -3.11 scripts/check_collisions.py` | Collision detection — arm turns red hitting a box |
| `py -3.11 scripts/plan_safe.py` | The replanning loop — lifts up and over the box |
| `py -3.11 scripts/nlp_command.py` | English → target (type commands) |
| `py -3.11 scripts/command_arm.py` | **Full loop:** type a command → arm plans & moves |
| `py -3.11 scripts/evaluate.py` | Metrics over 500+ trials (text output, no window) |

For `command_arm.py`, type things like `move to the left bin`, `go to the right
bin`, `return home`, `reach up high`, `go to x 0.3 y 0.1 z 0.4`. Blank line quits.

---

## 5. The full live demo (all systems at once)

This runs the twin live into the database and dashboards. Make sure Docker is
running (`docker compose up -d`).

1. **Start the live twin** (continuous motion → InfluxDB):
   ```
   py -3.11 scripts/run_live.py
   ```
2. **Open Grafana:** Open Grafana: http://localhost:3001 (login `admin` / `admin`). Add an
   InfluxDB data source (see troubleshooting for the exact settings), then build
   a dashboard with panels querying the `arm_state` measurement (joint angles,
   x/y/z, temp_c, current_a, collision). Set the time range to **Last 5 minutes**
   and refresh to **5s** — the graphs move in real time.
3. **Omniverse (optional):** open `usd/eval_showcase.usda`, press F to frame it,
   press Play. The arm runs its test tour, avoiding the box.

### Twin ↔ robot link (the physical-robot protocol, tested with a stand-in)
Two terminals:
```
py -3.11 scripts/virtual_robot.py     # terminal 1: pretends to be the ESP32
py -3.11 scripts/robot_link.py        # terminal 2: twin sends a planned motion
```

---

## 6. Viewing the Omniverse 3D

In any NVIDIA Omniverse app:
**File → Open** → pick `usd/arm_motion.usda` or `usd/eval_showcase.usda` →
select `/World` and press **F** to frame → press **Play** on the timeline.


---

## 7. Troubleshooting

- **`python` not recognized / opens the Store (Windows):** use `py -3.11`.
- **PowerShell rejects `&&`:** run each command on its own line.
- **`pip` compiles something and fails (e.g., a C++ error):** you're on the
  wrong Python. Use Python 3.11.
- **`docker compose` errors about the daemon:** Docker Desktop isn't running —
  open the app and wait for "Engine running".
- **Grafana shows no data:** three things must be right in the InfluxDB data
  source — **URL = `http://influxdb:8086`** (NOT `localhost`, because Grafana
  talks to InfluxDB *inside* Docker), **Query language = Flux**, and
  **Organization / Token / Default Bucket** must match `docker-compose.yml`
  (`sixdof` / `dev-token-change-me` / `robot_telemetry`).
- **A script says it can't reach InfluxDB:** start Docker (`docker compose up
  -d`) and check the token/org/bucket at the top of the script match
  `docker-compose.yml`.
- **The AI says `[built-in parser]` instead of `[LLM ...]`:** Ollama isn't
  running or the model isn't pulled. Install Ollama, run `ollama pull qwen2.5`,
  keep the app open. The parser fallback still works without it.
- **Two-terminal scripts:** the streaming demos need one terminal for the
  publisher and one for the listener — use VS Code's split-terminal button.

---

## 8. Project structure

```
6dof_dt/
├── README.md               
├── requirements.txt        <- Python dependencies
├── docker-compose.yml      <- MQTT (Mosquitto) + InfluxDB + Grafana
├── mosquitto/config/       <- broker config
├── urdf/arm6dof.urdf       <- the arm model (kinematics come from here)
├── usd/                    <- ready-made Omniverse scenes (.usda)
└── scripts/
    ├── arm_lib.py          <- SHARED toolkit: kinematics, planning, collision
    ├── view_arm.py         <- load + forward kinematics
    ├── ik_demo.py          <- inverse kinematics
    ├── move_smooth.py      <- smooth trajectories
    ├── check_collisions.py <- collision detection
    ├── plan_safe.py        <- collision-free replanning loop
    ├── nlp_command.py      <- English -> target (LLM + fallback)
    ├── command_arm.py      <- full command -> plan -> move loop
    ├── twin_stream.py      <- publish state over ZMQ (25 Hz)
    ├── monitor.py          <- print the ZMQ stream
    ├── db_logger.py        <- ZMQ stream -> InfluxDB
    ├── query_influx.py     <- read recent rows back from InfluxDB
    ├── run_live.py         <- one-command live twin -> InfluxDB (for Grafana)
    ├── virtual_robot.py    <- stand-in for the physical arm (MQTT)
    ├── robot_link.py       <- twin sends planned motion to the robot (MQTT)
    ├── make_usd.py         <- (re)generate an Omniverse animation
    ├── make_eval_usd.py    <- (re)generate the evaluation tour
    ├── evaluate.py         <- measure the success metrics
    └── check_stack.py      <- health-check the Docker services
```

`arm_lib.py` is imported by most scripts — keep it in `scripts/`.

---

## 9. Status

Complete (in simulation): kinematics, motion planning, collision avoidance,
natural-language commands, ZMQ/MQTT communication, InfluxDB logging, Grafana
dashboards, and Omniverse rendering. Evaluated over 500+ trials: position
accuracy < 1 cm at 100%, zero collisions, joint accuracy 100%.

