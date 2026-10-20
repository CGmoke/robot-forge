<div align="center">

# RobotForge

**Reusable building blocks for robot software, plus a real-robot engineering case study** —
one monorepo, two layers: a library you can install, and a project that runs on actual hardware.

[![License](https://img.shields.io/badge/License-Apache--2.0-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB.svg)](https://www.python.org/)
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E.svg)](https://docs.ros.org/en/jazzy/)
[![tests](https://img.shields.io/badge/tests-128%20passed-brightgreen.svg)](packages/robotics_algorithm/tests)
[![CI](https://github.com/CGmoke/robot-forge/actions/workflows/python-ci.yml/badge.svg)](https://github.com/CGmoke/robot-forge/actions/workflows/python-ci.yml)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](CONTRIBUTING.md)

[English](README.en.md) · [简体中文](README.md)

</div>

---

## What this is

RobotForge is a **monorepo for robot software**. It keeps two kinds of content side by side:

| Directory | What goes here | What is here now |
| --- | --- | --- |
| [`packages/`](packages/) | Libraries shared across projects — decoupled from any specific robot, usable anywhere | `robotics_algorithm`: a pure-Python algorithm library, `pip install`-able |
| [`projects/`](projects/) | Publishable robot projects — a complete engineering effort for one robot | `tianyi25/greeting`: a ROS 2 real-robot greeting application for the Tianyi 2.5 |

Two design rules:

| Rule | Meaning |
| --- | --- |
| 🧩 **Push reuse downward** | Algorithms and utilities shared by multiple projects become a library — no copy-paste reuse |
| 🔌 **Stay robot-agnostic** | Code under `packages/` contains no absolute paths, reads no environment variables, and depends on no robot-specific configuration |

> Projects under `projects/` **must not** depend on each other. Anything shared belongs in `packages/`.

## What is inside

| Project | What it is | Stack | Status |
| --- | --- | --- | --- |
| [`packages/robotics_algorithm`](packages/robotics_algorithm/README.md) | A Python library in 8 subpackages: 3D geometry, occupancy grids, planar inverse kinematics, pinhole vision, YAML/NPZ serialization, IoU tracking, a Modbus gripper, and subprocess bridging | Python 3.10+, numpy | ✅ **128 tests, installable** |
| [`projects/tianyi25/greeting`](projects/tianyi25/greeting/README.md) | Full "greet / guide / farewell" flow for the Tianyi 2.5 humanoid: 6 ROS 2 packages (interfaces, upper-body motion, chassis navigation, voice, gamepad teleop, state machine) | ROS 2 Jazzy, rclpy | ⚠️ **Real-robot project — needs the hardware** |

> ⚠️ About `projects/tianyi25/greeting`: this runs on a **physical** robot. It depends on a
> vendor-supplied platform environment (a platform `setup.bash`, `ros2_bridge_msgs`,
> `bodyctrl_msgs`) and an actual Tianyi 2.5 with a knewbots chassis.
> **You cannot run it without the robot** — but the code, the interface contract, and the
> architecture documents are readable and instructive, especially the
> [interface freeze table](projects/tianyi25/greeting/接口契约冻结表.md) and the state-machine design.

## Quick start

### A. Use the Python library (30 seconds, works for anyone)

```bash
git clone https://github.com/CGmoke/robot-forge.git
cd robot-forge/packages/robotics_algorithm
python -m pip install -e ".[test]"
python -m pytest          # expect 128 passed
```

```python
from robotics_algorithm.geometry.boxes import iou
from robotics_algorithm.tracking.iou_tracker import IoUTracker

tracker = IoUTracker(min_hits=1)
frame = tracker.update([{"bbox": [0, 0, 10, 10], "label": "person"}])
print(frame[0]["track_id"], iou([0, 0, 10, 10], [2, 2, 12, 12]))
```

No install needed — just put `src/` on `PYTHONPATH`:

```bash
PYTHONPATH=src python -c "from robotics_algorithm.geometry.boxes import iou; print(iou([0,0,10,10],[2,2,12,12]))"
```

### B. Read the greeting project (no hardware required)

The shortest path to understanding the whole chain is three documents:

1. [Project overview and architecture](projects/tianyi25/greeting/README.md) — package layering, data flow, launch entries
2. [Interface freeze table](projects/tianyi25/greeting/接口契约冻结表.md) — the single source of truth for names, fields, QoS, and semantics
3. [On-robot operator guide](projects/tianyi25/greeting/greeting_ws/README.md) — build, launch, trigger, debug

With the robot available, follow the operator guide:

```bash
source /opt/ros/jazzy/setup.bash
source <vendor-platform>/setup.bash       # provides ros2_bridge_msgs / bodyctrl_msgs
cd projects/tianyi25/greeting/greeting_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch greeting_teleop greeting_bringup.launch.py
```

## Repository layout

```
robot-forge/
├── README.md                          # Chinese README
├── README.en.md                       # You are here (English)
├── NOTICE                             # Copyright and provenance
├── CONTRIBUTING.md                    # How to file issues / PRs
├── CHANGELOG.md                       # Release notes
├── LICENSE                            # Apache-2.0
├── .github/                           # CI, issue / PR templates
├── docs/
│   └── adding-a-project.md            # Project conventions + IP self-check
├── packages/                          # Libraries shared across projects
│   └── robotics_algorithm/
│       ├── README.md                  # Module index and API overview
│       ├── robotics-algorithm-learning-guide.md   # Full learning guide
│       ├── pyproject.toml
│       ├── src/robotics_algorithm/    # geometry / gridmap / motion / vision / io / tracking / hardware / runtime
│       └── tests/                     # 128 unit tests
└── projects/                          # Publishable robot projects
    ├── README.md                      # Directory conventions: what may / may not go here
    └── tianyi25/
        └── greeting/
            ├── README.md              # Workspace overview: layering, data flow, launch entries
            ├── greeting_ws/           # ROS 2 workspace (6 packages)
            │   ├── README.md          # On-robot operator guide
            │   └── src/
            │       ├── greeting_interfaces/     # msg / srv / action — bottom of the dependency graph
            │       ├── greeting_body/           # Upper-body motion action server
            │       ├── greeting_nav/            # Chassis REST navigation action server
            │       ├── greeting_voice/          # Speech output
            │       ├── greeting_teleop/         # Gamepad gateway + command bridges
            │       └── greeting_orchestrator/   # Greeting state machine (single commander)
            └── *.md                   # Interface contract, collaboration rules, project plan
```

## Requirements

| Use case | Requirements |
| --- | --- |
| Python library | Python 3.10+; only hard dependency is `numpy`. Optional extras: `[vision]`, `[hardware]`, `[yaml]` |
| Greeting project | Ubuntu 24.04, ROS 2 Jazzy, a **physical Tianyi 2.5** with its vendor platform environment, and a knewbots chassis (REST `:9090`) |

## Documentation index

| Document | Contents |
| --- | --- |
| [`packages/robotics_algorithm/README.md`](packages/robotics_algorithm/README.md) | Module index, installation, API overview |
| [`robotics-algorithm-learning-guide.md`](packages/robotics_algorithm/robotics-algorithm-learning-guide.md) | Full learning guide, written as a guided tour |
| [`projects/tianyi25/greeting/README.md`](projects/tianyi25/greeting/README.md) | Greeting project overview: package layering, data flow, launch entries |
| [`接口契约冻结表.md`](projects/tianyi25/greeting/接口契约冻结表.md) | Interface contract: names, fields, QoS, semantics, state machine (Chinese) |
| [`greeting_ws/README.md`](projects/tianyi25/greeting/greeting_ws/README.md) | On-robot operator guide: setup, build, launch, trigger, debug (Chinese) |
| [`projects/README.md`](projects/README.md) | What may and may not go under `projects/` |
| [`docs/adding-a-project.md`](docs/adding-a-project.md) | Directory conventions, README template, and IP self-check (Chinese) |
| [`NOTICE`](NOTICE) | Copyright and provenance |
| [`.github`](.github/) | What CI runs, how to file issues and PRs |

> The greeting project's design documents are written in Chinese. Code, docstrings,
> and this English README are intended to be enough to evaluate and reuse the code.

## Roadmap

See [CHANGELOG.md](CHANGELOG.md) for what has shipped. Near-term direction:

- [ ] **v0.2**: publish `robotics_algorithm` to PyPI; add type annotations and API docs
- [ ] Keep extracting the reusable parts of the greeting project into `packages/`
- [ ] Add performance benchmarks and more doctest examples
- [x] English README (this file)

Ideas and claims are welcome in [Issues](https://github.com/CGmoke/robot-forge/issues).

## Contributing

Any form of participation is welcome: bug reports, documentation fixes, tests, new modules.

- Read [CONTRIBUTING.md](CONTRIBUTING.md) before submitting
- **Adding a project?** Start with [docs/adding-a-project.md](docs/adding-a-project.md) — especially the IP self-check
- Not sure whether something is worth doing? Open an [Issue](https://github.com/CGmoke/robot-forge/issues) first and we will figure it out together

## ⭐ If it helped you

A **star** is the most direct way to support this project: it costs nothing, but it makes
RobotForge easier to find and keeps the maintainers motivated to keep pulling reusable
pieces out of the real-robot work.

[![Star this repo](https://img.shields.io/badge/%E2%AD%90_Star_this_repo-blue?style=flat-square)](https://github.com/CGmoke/robot-forge/stargazers)

## License

[Apache License 2.0](LICENSE). Everything in this repository is original work and contains no
third-party assets — see [NOTICE](NOTICE) for details.
