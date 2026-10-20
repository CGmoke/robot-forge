<div align="center">

# RobotForge · 机器人锻造厂

**机器人软件的可复用构建块 + 实机工程实践** —— 一个仓库，两层内容：能装的库，和能上真机的工程。

[![License](https://img.shields.io/badge/License-Apache--2.0-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB.svg)](https://www.python.org/)
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E.svg)](https://docs.ros.org/en/jazzy/)
[![tests](https://img.shields.io/badge/tests-128%20passed-brightgreen.svg)](packages/robotics_algorithm/tests)
[![CI](https://github.com/CGmoke/robot-forge/actions/workflows/python-ci.yml/badge.svg)](https://github.com/CGmoke/robot-forge/actions/workflows/python-ci.yml)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](CONTRIBUTING.md)

[简体中文](README.md) · [English](README.en.md)

</div>

---

## 📌 这是什么

RobotForge 是一个面向机器人软件开发的 **monorepo**。它把两类内容放在一起维护：

| 目录 | 放什么 | 现在有什么 |
| --- | --- | --- |
| [`packages/`](packages/) | 跨项目复用的库 —— 与具体机器人解耦，任何项目都能用 | `robotics_algorithm`：纯 Python 算法库，可 `pip install` |
| [`projects/`](projects/) | 可公开的机器人工程 —— 某个机器人的完整工程 | `tianyi25/greeting`：天轶 2.5 迎宾机器人 ROS 2 实机工程 |

两条设计原则：

| 原则 | 含义 |
| --- | --- |
| 🧩 **复用下沉** | 多个工程共用的算法与工具抽成库，杜绝复制粘贴式复用 |
| 🔌 **与机器人解耦** | `packages/` 下的代码不含绝对路径、不读环境变量、不依赖任何具体机器人的配置 |

> `projects/` 之间**不允许**互相依赖；公共部分一律下沉到 `packages/`。

## 🗂 仓库里有什么

| 项目 | 是什么 | 技术栈 | 状态 |
| --- | --- | --- | --- |
| [`packages/robotics_algorithm`](packages/robotics_algorithm/README.md) | 8 个子包的 Python 构建块：3D 几何、占据栅格、平面逆运动学、针孔视觉、YAML/NPZ 序列化、IoU 跟踪、Modbus 夹爪、子进程桥接 | Python 3.10+ · numpy | ✅ **128 个单测，可安装** |
| [`projects/tianyi25/greeting`](projects/tianyi25/greeting/README.md) | 天轶 2.5 人形机器人「迎宾 / 引导 / 送别」全流程：6 个 ROS 2 包（接口 / 上半身动作 / 底盘导航 / 语音 / 手柄 / 状态机） | ROS 2 Jazzy · rclpy | ⚠️ **实机工程，无真机跑不通** |

> ⚠️ 关于 `projects/tianyi25/greeting`：这是跑在**真机**上的工程，依赖厂商随设备提供的
> 平台环境（平台 `setup.bash`、`ros2_bridge_msgs`、`bodyctrl_msgs`）与一台真实的天轶 2.5。
> **没有机器人你无法把它跑起来**，但代码、接口契约与架构文档可以读、可以借鉴 ——
> 尤其是它的[接口契约冻结表](projects/tianyi25/greeting/接口契约冻结表.md)与状态机设计。

## 🚀 快速开始

### A. 用 Python 算法库（30 秒，任何人都能跑）

```bash
git clone https://github.com/CGmoke/robot-forge.git
cd robot-forge/packages/robotics_algorithm
python -m pip install -e ".[test]"
python -m pytest          # 期望 128 passed
```

```python
from robotics_algorithm.geometry.boxes import iou
from robotics_algorithm.tracking.iou_tracker import IoUTracker

tracker = IoUTracker(min_hits=1)
frame = tracker.update([{"bbox": [0, 0, 10, 10], "label": "person"}])
print(frame[0]["track_id"], iou([0, 0, 10, 10], [2, 2, 12, 12]))
```

不安装也能用 —— 把 `src/` 加进 `PYTHONPATH`：

```bash
PYTHONPATH=src python -c "from robotics_algorithm.geometry.boxes import iou; print(iou([0,0,10,10],[2,2,12,12]))"
```

### B. 读迎宾工程（无需真机）

先看这三份文档，最短路径理解整条链路：

1. [项目总览与架构](projects/tianyi25/greeting/README.md) —— 6 个包的分层、数据流、启动入口
2. [接口契约冻结表](projects/tianyi25/greeting/接口契约冻结表.md) —— 接口名 / 字段 / QoS / 语义的**唯一事实来源**
3. [实机操作指南](projects/tianyi25/greeting/greeting_ws/README.md) —— 编译、启动、触发、调试全流程

有真机时按操作指南执行即可：

```bash
source /opt/ros/jazzy/setup.bash
source <厂商平台>/setup.bash          # 提供 ros2_bridge_msgs / bodyctrl_msgs
cd projects/tianyi25/greeting/greeting_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch greeting_teleop greeting_bringup.launch.py
```

## 🧱 仓库结构

```
robot-forge/
├── README.md                          # 你在这里（中文）
├── README.en.md                       # English README
├── NOTICE                             # 版权与来源声明
├── CONTRIBUTING.md                    # 怎么提 issue / PR
├── CHANGELOG.md                       # 版本变更记录
├── LICENSE                            # Apache-2.0
├── .github/                           # CI、issue / PR 模板
├── docs/
│   └── adding-a-project.md            # 新项目接入规范 + 知识产权自检
├── packages/                          # 跨项目复用的库
│   └── robotics_algorithm/
│       ├── README.md                  # 模块索引与 API 概览
│       ├── robotics-algorithm-learning-guide.md   # 全模块学习指南
│       ├── pyproject.toml
│       ├── src/robotics_algorithm/    # geometry / gridmap / motion / vision / io / tracking / hardware / runtime
│       └── tests/                     # 128 个单测
└── projects/                          # 可公开的机器人工程
    ├── README.md                      # 目录约定、什么能放 / 不能放
    └── tianyi25/
        └── greeting/
            ├── README.md              # 工作空间总览：分层、数据流、启动入口
            ├── 接口契约冻结表.md        # 接口的唯一事实来源（内网地址已脱敏）
            └── greeting_ws/           # ROS 2 工作空间（6 个包 + systemd 自启脚本）
                ├── README.md          # 实机操作指南
                └── src/
                    ├── greeting_interfaces/     # msg / srv / action，依赖最底层
                    ├── greeting_body/           # 上半身动作服务端
                    ├── greeting_nav/            # 底盘 REST 导航服务端
                    ├── greeting_voice/          # 语音播报
                    ├── greeting_teleop/         # 手柄网关 + 命令桥
                    └── greeting_orchestrator/   # 迎宾状态机（唯一指挥者）
```

## 🖥 环境要求

| 用途 | 要求 |
| --- | --- |
| Python 库 | Python 3.10+，仅硬依赖 `numpy`；可选依赖按需安装（`[vision]` / `[hardware]` / `[yaml]`） |
| 迎宾工程 | Ubuntu 24.04 + ROS 2 Jazzy + **天轶 2.5 真机**及其平台环境 + knewbots 底盘（REST `:9090`） |

## 📚 文档索引

| 文档 | 内容 |
| --- | --- |
| [`packages/robotics_algorithm/README.md`](packages/robotics_algorithm/README.md) | 模块索引、安装方式与 API 概览 |
| [`robotics-algorithm-learning-guide.md`](packages/robotics_algorithm/robotics-algorithm-learning-guide.md) | 全模块学习指南（导读式，适合入门） |
| [`projects/tianyi25/greeting/README.md`](projects/tianyi25/greeting/README.md) | 迎宾工程总览：包分层、数据流、常用启动入口 |
| [`接口契约冻结表.md`](projects/tianyi25/greeting/接口契约冻结表.md) | 接口名 / 字段 / QoS / 语义 / 状态机 |
| [`greeting_ws/README.md`](projects/tianyi25/greeting/greeting_ws/README.md) | 实机操作指南：环境、编译、启动、一键触发、调试 |
| [`projects/README.md`](projects/README.md) | `projects/` 的目录约定、可放 / 不可放内容的对照表 |
| [`docs/adding-a-project.md`](docs/adding-a-project.md) | 新增项目的目录规范、README 模板与接入自检清单（含知识产权） |
| [`NOTICE`](NOTICE) | 版权与来源声明 |
| [`.github`](.github/) | 提交前会跑什么、issue / PR 怎么写 |

## 🗺 路线图

见 [CHANGELOG.md](CHANGELOG.md) 了解已完成的内容。近期方向：

- [ ] **v0.2**：`robotics_algorithm` 发布到 PyPI，补齐类型标注与 API 文档
- [ ] 继续把迎宾工程中可复用的部分下沉到 `packages/`
- [ ] 补充性能基准与更多 doctest 示例
- [x] 英文 README（[README.en.md](README.en.md)）

欢迎在 [Issues](https://github.com/CGmoke/robot-forge/issues) 里认领或提新需求。

## 🤝 贡献

欢迎任何形式的参与：报 bug、修文档、加测试、提新模块。

- 提交前请读 [CONTRIBUTING.md](CONTRIBUTING.md)
- **加新项目**请先看 [docs/adding-a-project.md](docs/adding-a-project.md) —— 特别是知识产权自检
- 不确定要不要做，先开个 [Issue](https://github.com/CGmoke/robot-forge/issues) 聊，避免白写

## ⭐ 如果它帮到了你

点个 **Star** 是最直接的支持 —— 不改变任何代码，但能让更多人搜到 RobotForge，
也让维护者有动力继续把实机工程里可复用的部分下沉成库。

[![Star this repo](https://img.shields.io/badge/%E2%AD%90_Star_this_repo-blue?style=flat-square)](https://github.com/CGmoke/robot-forge/stargazers)

## 📄 协议

本项目采用 [Apache License 2.0](LICENSE)。仓库内容均为原创、不含第三方资产，详见 [NOTICE](NOTICE)。
