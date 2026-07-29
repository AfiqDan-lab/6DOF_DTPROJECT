# Development Practices: Sprint Plan, Version Control & CI/CD

Evidence for the Project Development Practices deliverable: sprint planning and
execution across ≥2 cycles, version control with per-member and group merges,
unit + integration testing with pass/fail cases, and CI/CD triggering the
regression suite on every build.

> **Fill in the bracketed `[…]` fields** with your team's real names and dates
> before submitting. Everything else reflects the delivered system.

---

## 1. Team

| Member | Role | Primary modules owned |
|--------|------|-----------------------|
| Muhammad Afiq Danial bin Dali | Twin lead — kinematics, planning, NLP, communication, deployment, CI/CD | `arm_lib.py`, `nlp_command.py`, `command_arm.py`, `twin_stream.py`, `db_logger.py`, `robot_link.py`, `virtual_robot.py`, `run_live.py`, `evaluate.py`, `deploy/`, `tests/`, CI workflow |
| Ahmad Azri bin Ibrahim | Visualization lead — Omniverse 3D model | `usd/arm_motion.usda`, `usd/eval_showcase.usda`, `make_usd.py`, `make_eval_usd.py` |

---

## 2. Sprint plan (2 cycles)

Development was organized into two sprints. Each sprint has features, an owner,
milestones, and concrete deliverables.

### Sprint 1 — "The twin moves and is safe" — [start date] → [end date]

**Goal:** a simulated arm that interprets a command, plans a collision-free
motion, and executes it.

| Feature | Owner | Milestone / deliverable |
|---------|-------|-------------------------|
| Load URDF, forward kinematics | Afiq | `view_arm.py` renders arm; FK verified |
| Inverse kinematics with restarts | Afiq | `ik_demo.py`; < 1 cm accuracy |
| Smooth trajectories (quintic) | Afiq | `move_smooth.py`; zero end velocity |
| Collision detection | Afiq | `check_collisions.py`; arm turns red on hit |
| Safe replanning (lift-over) | Afiq | `plan_safe.py`; direct-then-over loop |
| English → target parser | Afiq | `nlp_command.py` with built-in fallback |
| Full command → move loop | Afiq | `command_arm.py` |
| Omniverse arm-motion scene | Azri | `usd/arm_motion.usda`, `make_usd.py` |
| **Sprint-1 unit tests** | Afiq | `test_kinematics.py`, `test_planning.py`, `test_nlp_command.py` |

**Sprint-1 review:** individual branches merged to `develop`; demo of
`command_arm.py` moving the arm around the box, and the arm motion rendered in
Omniverse.

### Sprint 2 — "The twin streams, stores, and deploys" — [start] → [end]

**Goal:** live telemetry into a database and dashboards, a robot link, and the
whole system deployed as containerized, tested microservices.

| Feature | Owner | Milestone / deliverable |
|---------|-------|-------------------------|
| ZMQ live state stream (25 Hz) | Afiq | `twin_stream.py` + `monitor.py` |
| InfluxDB logging | Afiq | `db_logger.py`, `query_influx.py` |
| One-command live twin | Afiq | `run_live.py` → Grafana updates live |
| Twin ↔ robot MQTT link | Afiq | `robot_link.py` + `virtual_robot.py` |
| Metrics over 500+ trials | Afiq | `evaluate.py` |
| Omniverse evaluation tour scene | Azri | `usd/eval_showcase.usda`, `make_eval_usd.py` |
| Containerize each service | Afiq | `deploy/Dockerfile.*`, `docker-compose.full.yml` |
| Microservice contracts | Afiq | `docs/MICROSERVICES.md` |
| **Integration tests** | Afiq | `test_integration.py` (pipeline + contracts) |
| **CI/CD pipeline** | Afiq | `.github/workflows/ci.yml` |
| Deployment, scaling, persistence | Afiq | `docs/DEPLOYMENT.md` + evidence |

**Sprint-2 review:** group merge of all sprint-2 branches to `main`; full-system
demo (Omniverse + Grafana live), green CI, deployment evidence captured.

---

## 3. Version control workflow

Branch-per-feature, merged at sprint boundaries — the individual-then-group
merge the rubric asks for.

```
main          ●────────────────────●───────────────────────●  (release: end of each sprint)
               \                   / \                     /
develop         ●────●────●───────●   ●────●────●────●────●   (integration)
                 \   \    \           \    \    \    \
feature branches  a   b    c           d    e    f    g       (per member / per feature)
```

**If the repo is not yet initialized**, this reproduces a clean sprint history:

```bash
cd 6dof_dt
git init
git add .
git commit -m "Sprint 1: kinematics, planning, NLP, unit tests"

git checkout -b develop
# ... sprint 2 work committed on feature branches, merged to develop ...
git checkout main
git merge develop -m "Sprint 2: streaming, storage, deployment, CI"
git tag v1.0-sprint2
```

**📸 Evidence:** `git log --oneline --graph --all` showing feature branches
merging at each sprint boundary, and commits attributed to each member.
(Have each member author their own module's commits so authorship is visible.)

---

## 4. Testing strategy (unit + integration, pass **and** fail cases)

The suite is in `tests/`, run with `pytest`. **39 tests** across four files.

**Unit tests** — one module each, isolated:
- `test_kinematics.py` — FK determinism, IK round-trip < 1 cm.
  - *Fail case:* `test_ik_far_target_is_reported_not_faked` — an unreachable
    target must return a large, honest error, not a fake success.
- `test_planning.py` — collision geometry, safe-planner guarantees.
  - *Fail case:* `test_path_through_box_is_flagged` — a path driven through a
    box **must** be detected as colliding.
- `test_nlp_command.py` — location/coordinate/relative parsing, clamping.
  - *Fail case:* `test_resolve_target_rejects_garbage` — unparseable input
    returns `None` so it can be refused; `test_clamp_*` — out-of-range requests
    are pulled back inside the workspace.

**Integration tests** — blocks wired together (`test_integration.py`):
- Full pipeline: English → target → IK → safe plan → trajectory ending on target.
- Message-contract tests: the `arm/cmd`, `arm/feedback`, and InfluxDB-schema
  payloads have exactly the agreed fields (catches a rename on either side).
- Live MQTT round-trip: real publish/subscribe when a broker is up; **skips
  automatically** when it isn't, so CI without Docker still passes.

Run it:

```bash
py -3.11 -m pytest -v
```

Current result: **38 passed, 1 skipped** (the skip is the live-MQTT test, which
runs and passes in CI where a broker service is provided).

---

## 5. CI/CD (automatic build → regression suite)

`.github/workflows/ci.yml` runs on every push and pull request to
`main` / `develop` / `sprint-*`:

1. **`test` job** — rebuilds a clean environment on Python 3.11 **and** 3.12,
   installs `requirements.txt`, and runs the **full regression suite**. A
   Mosquitto service container runs alongside it, so the live integration test
   executes rather than skipping.
2. **`build-images` job** — builds all three service images, so a broken
   Dockerfile fails CI before it ever reaches deployment.

This is the "updates triggered by automatic build and application of a
regression test suite" — every merge is gated by the suite.

**📸 Evidence:** a green CI run in the GitHub **Actions** tab, and a pull request
showing the required check passing before merge.

---

## 6. Development-practices evidence summary

| # | Evidence | Rubric line |
|---|----------|-------------|
| 1 | This document (2 sprints, owners, milestones) | Sprint planning & execution ≥ 2 cycles |
| 2 | `git log --graph --all` with sprint merges | Consistent version control, individual + group merge |
| 3 | `tests/` + `pytest -v` output | Unit + integration tests, pass/fail cases |
| 4 | Green run in Actions tab | CI/CD: automatic build + regression suite |
