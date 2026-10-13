#!/usr/bin/env python3
"""迎宾编排层 —— 唯一指挥者 / 唯一状态机（接口契约 §1）。

职责：
  · 订阅 /greeting/control_cmd、/greeting/events、/greeting/perception/person、/greeting/intent
  · 按 config/greeting_script.yaml 的段顺序，依次调用三个 Action：
        /greeting/speak        (Speak)
        /greeting/navigate_to  (NavigateTo)
        /greeting/play_motion  (PlayMotion)
  · cue 步骤：一步内【并行】编排「动作 + 语音」，动作比语音早 lead_s 触发，
    两者都结束后再推进下一步。大会开始打招呼 / 结束离场祝贺 lead_s=1.0，
    其余流程 lead_s=3.0（见讲稿 YAML）。
  · 发布 /greeting/state（std_msgs/String，reliable + transient_local，depth 1）
  · 发布 /greeting/health（SubsystemHealth，1 Hz）
  · 提供 /greeting/control 与 /greeting/reload_script 两个服务

🔴 单一指挥者规则：只有本节点调用上述三个 Action，模块之间禁止互相调用。
🔴 仿真下 navigate_to 由桩服务端包 Nav2；实机上包底盘 REST :9090。编排层不关心差异。

控制指令（ConsoleCommand.cmd）：
  start | pause | resume | next | prev | goto(需 arg=段号) | stop | manual

其中 goto 为【单段点播】：只播该段（动作 + 语音同步），播完即回 IDLE，不续播下一段。
供遥控器"一段一个按键"使用；整段连播请用 start。
"""
from __future__ import annotations

from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from greeting_interfaces.action import NavigateTo, PlayMotion, Speak
from greeting_interfaces.msg import (
    ConsoleCommand,
    GreetingEvent,
    GreetingIntent,
    PersonDetection,
    SubsystemHealth,
)
from greeting_interfaces.srv import GreetingControl
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

# 礼仪硬要求：动作幅度统一 ≤ 0.7，编排层先钳一次
MOTION_SPEED_LIMIT = 0.7
# 处于"正在执行讲稿"的基准状态；讲稿各段 name 在加载时动态并入
BASE_RUNNING_STATES = {"RUNNING"}

# 实机本体话题
ROBOT_STATE_TOPIC = "/robot_state"                          # robot_control 发布，有发布者即进程存活
BODY_CMD_TOPICS = ("/head/cmd", "/arm/cmd", "/waist/cmd")   # 命令无订阅者会被静默丢弃
ARM_CMD_TOPIC = "/arm/cmd"                                  # 动作硬预检的门禁通道


class GreetingOrchestrator(Node):
    """迎宾编排层节点"""
    def __init__(self) -> None:
        """初始化节点"""
        super().__init__("greeting_orchestrator")                 # 节点名
        self.declare_parameter("config_file", "")                 # 配置文件路径
        self.declare_parameter("state_topic", "/greeting/state")  # 状态发布主题
        self.declare_parameter("default_nav_timeout", 60.0)       # 导航超时时间(s)
        self.declare_parameter("approved_script_version", "1.1")  # 审核通过的讲稿版本
        self.declare_parameter("default_motion_lead_s", 3.0)      # cue 步骤未显式写 lead_s 时，动作相对语音的默认提前量(s)

        self._cfg_file = self.get_parameter("config_file").value
        if not self._cfg_file:
            self._cfg_file = str(
                Path(get_package_share_directory("greeting_orchestrator"))
                / "config"
                / "greeting_script.yaml"
            )
        self._running_states = set(BASE_RUNNING_STATES)
        self._load_script_or_fail(self._cfg_file)

        self._state = "IDLE"  # 状态机当前状态
        self._seg_idx = -1    # 当前执行的段索引，-1 表示未开始
        self._step_idx = 0    # 当前执行的步骤索引，0 表示段开始
        self._paused = False  # 是否暂停执行
        #: goto 点播模式：播完该段即回 IDLE，不续播下一段（遥控器"一段一键"用）
        self._single_segment = False  # goto 点播模式：播完该段即回 IDLE，不续播下一段（遥控器"一段一键"用）
        self._pending = None          # 单步: {'kind','stage','future','label'}；cue: 复合结构
        self._goal_handle = None
        self._cue_timer = None        # cue 里"延迟下发语音"的一次性定时器
        self._missing_logged = set()  # 已记录的缺失事件（如导航超时）
        self._default_motion_lead_s = float(
            self.get_parameter("default_motion_lead_s").value
        )

        # ---- /greeting/state: reliable + transient_local / depth 1（状态机状态发布）----
        state_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_pub = self.create_publisher(
            String, self.get_parameter("state_topic").value, state_qos
        )
        self._health_pub = self.create_publisher(SubsystemHealth, "/greeting/health", 10)

        # ---- 订阅（控制指令、事件、意图、感知到的人）----
        reliable10 = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            ConsoleCommand, "/greeting/control_cmd", self._on_cmd, reliable10
        )
        self.create_subscription(GreetingEvent, "/greeting/events", self._on_event, reliable10)
        self.create_subscription(GreetingIntent, "/greeting/intent", self._on_intent, reliable10)
        self.create_subscription(
            PersonDetection,
            "/greeting/perception/person",
            self._on_person,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        # ---- Action 客户端 ----
        self._speak = ActionClient(self, Speak, "/greeting/speak")
        self._nav = ActionClient(self, NavigateTo, "/greeting/navigate_to")
        self._motion = ActionClient(self, PlayMotion, "/greeting/play_motion")

        # ---- 服务 S1 / S2 ----
        self.create_service(GreetingControl, "/greeting/control", self._on_srv)
        self.create_service(GreetingControl, "/greeting/reload_script", self._on_reload)

        self.create_timer(0.5, self._tick)           # 状态机更新定时器
        self.create_timer(1.0, self._publish_health) # 健康状态发布定时器

        self._publish_state()                        # 初始化状态发布
        self.get_logger().info(
            f"编排层就绪：讲稿 {len(self._segments)} 段，"
            f"等待 /greeting/control_cmd 下发 start"
        )

    def _load_script_or_fail(self, path: str) -> None:
        """加载讲稿文件，失败则退出节点"""
        self._segments = []
        ok, text = self._load_script(path)
        if ok:
            self.get_logger().info(text)
        else:
            self.get_logger().error(text)

    def _load_script(self, path: str) -> tuple[bool, str]:
        """加载讲稿文件，返回是否成功、成功则讲稿版本号、失败则错误信息"""
        # L5 内容守护：校验结果供 /greeting/health 的 script_version_ok 上报
        self._script_version_ok = False
        try:
            data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        except Exception as exc:  # noqa: BLE001
            return False, f"读取讲稿失败: {exc}"
        version = str(data.get("version", ""))
        approved = str(self.get_parameter("approved_script_version").value)
        if version != approved:
            # 契约 §3：reload_script 必须校验审核版本号，不符则拒绝加载
            return False, f"讲稿版本 {version!r} 与审核版本 {approved!r} 不符，拒绝加载"
        self._segments = data.get("segments", [])
        # 讲稿各段 name（大写）并入"可推进状态"，否则该段状态会被 _tick 视为非运行态而卡住
        self._running_states = set(BASE_RUNNING_STATES) | {
            str(seg.get("name", "")).upper() for seg in self._segments
        }
        self._script_version_ok = True
        return True, f"讲稿已加载，版本 {version}，共 {len(self._segments)} 段"

    # ------------------------------------------------------------ 状态机
    def _publish_state(self) -> None:
        """发布当前状态"""
        self._state_pub.publish(String(data=self._state))

    def _set_state(self, state: str) -> None:
        """设置当前状态"""
        if state != self._state:
            self.get_logger().info(f"状态切换: {self._state} -> {state}")
            self._state = state
            self._publish_state()

    def _publish_health(self) -> None:
        """发布当前健康状态（实机自检）。

        实机信号来源（对应规划方案 §7.7 的分层守护）：
          · L1 本体守护：``/robot_state`` 有发布者 ⇔ robot_control 进程存活。
            该进程会静默退出，届时命令发出去毫无反应且无报错，必须能看出来。
          · L2 通道守护：``/arm/cmd`` 有订阅者 ⇔ 命令不会被静默丢弃（动作门禁）。
          · L4 语音守护：``/greeting/speak`` 服务端在线（TTS 链路入口可用）。
          · L5 内容守护：讲稿版本与审核版本比对结果（加载/重载时给出）。
          · L3 底盘 / L6 DTC：当前 autostart 链路未接入底盘 REST 监测，
            按"未监测"上报，以 detail 的 ``chassis=unmonitored`` 为准。
        """
        # 一次性统计缺订阅者的本体通道（detail 里逐个列出，便于现场定位）
        missing = [t for t in BODY_CMD_TOPICS if self.count_subscribers(t) < 1]

        msg = SubsystemHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""
        # L1：robot_control 存活 ⇔ /robot_state 有发布者（未上电/退出即无发布者）
        msg.robot_control_alive = self.count_publishers(ROBOT_STATE_TOPIC) >= 1
        # L2：动作门禁通道 /arm/cmd 必须有订阅者
        msg.arm_cmd_has_subscriber = ARM_CMD_TOPIC not in missing
        # L4：语音链路入口（speak 服务端）在线
        msg.audio_ok = self._speak.server_is_ready()
        # L5：讲稿版本校验结果
        msg.script_version_ok = self._script_version_ok
        # L3 / L6：底盘未接入监测，保守上报"未监测"（不冒充正常）
        msg.chassis_rest_ok = False
        msg.slam_status = 0
        msg.dtc_level = 0
        msg.detail = self._health_detail(msg, missing)
        self._health_pub.publish(msg)

    def _health_detail(self, msg: SubsystemHealth, missing: list) -> str:
        """拼装人类可读的健康摘要，列出缺失通道，便于现场一眼定位。"""
        return (
            f"real: body={'ok' if msg.robot_control_alive else 'no_publisher'}"
            f" audio={'ok' if msg.audio_ok else 'speak_server_offline'}"
            f" script={'ok' if msg.script_version_ok else 'version_mismatch'}"
            f" channels={'ok' if not missing else 'missing[' + ','.join(missing) + ']'}"
            " chassis=unmonitored"
        )

    # ------------------------------------------------------------ 指令机
    def _on_cmd(self, msg: ConsoleCommand) -> None:
        """处理指令"""
        ok, text = self._handle_command(msg.cmd.strip(), msg.arg.strip())
        if not ok:
            self.get_logger().warn(f"[cmd:{msg.cmd}] {text}")

    def _on_srv(self, request: GreetingControl.Request, response: GreetingControl.Response):
        """处理指令"""
        ok, text = self._handle_command(request.cmd.strip(), request.arg.strip())
        response.success = ok
        response.message = text
        response.current_state = self._state_descriptor()
        return response

    def _on_reload(self, request: GreetingControl.Request, response: GreetingControl.Response):
        """重载讲稿"""
        ok, text = self._load_script(self._cfg_file)
        response.success = ok
        response.message = text
        response.current_state = self._state_descriptor()
        return response

    def _state_descriptor(self) -> str:
        """拼装人类可读的当前状态描述，列出缺失通道，便于现场一眼定位。"""
        seg_no = self._seg_idx + 1 if self._seg_idx >= 0 else 0
        return f"{self._state}:seg={seg_no}/step={self._step_idx}"

    def _handle_command(self, cmd: str, arg: str) -> tuple[bool, str]:
        """处理指令"""
        if cmd == "start":
            self._single_segment = False
            self._start_segment(0)
            return True, "开始接待"
        if cmd == "pause":
            if self._state not in self._running_states:
                return False, f"当前状态 {self._state} 不可暂停"
            self._paused = True
            self._set_state("PAUSED")
            return True, "已暂停"
        if cmd == "resume":
            if self._state != "PAUSED":
                return False, f"当前状态 {self._state} 不可恢复"
            self._paused = False
            self._resume_state()
            return True, "已恢复"
        if cmd == "next":
            self._single_segment = False
            self._start_segment(min(self._seg_idx + 1, len(self._segments) - 1))
            return True, "跳到下一段"
        if cmd == "prev":
            self._single_segment = False
            self._start_segment(max(self._seg_idx - 1, 0))
            return True, "回到上一段"
        if cmd == "goto":
            try:
                idx = int(arg) - 1
            except ValueError:
                return False, f"goto 参数非法: {arg!r}（应为段号，如 2）"
            if not 0 <= idx < len(self._segments):
                return False, f"段号 {arg!r} 超出范围（1~{len(self._segments)}）"
            # 单段点播：只播该段，播完回 IDLE（遥控器"一段一键"）
            self._single_segment = True
            self._start_segment(idx)
            return True, f"点播第 {arg} 段（播完回到 IDLE）"
        if cmd == "stop":
            self._abort_pending()
            self._set_state("IDLE")
            return True, "已停止"
        if cmd == "manual":
            self._abort_pending()
            self._set_state("MANUAL")
            return True, "进入人工接管"
        return False, f"未知指令: {cmd!r}"

    # ------------------------------------------------------------ 事件/感知
    def _on_event(self, msg: GreetingEvent) -> None:
        """处理事件"""
        if msg.event_type == GreetingEvent.EVENT_MANUAL_START:
            self._single_segment = False
            self._start_segment(0)
        elif msg.event_type == GreetingEvent.EVENT_END:
            self._abort_pending()
            self._set_state("IDLE")
        elif msg.event_type == GreetingEvent.EVENT_LEADER_ARRIVED:
            # 契约 §5.1：仅提示，不自动接待
            self.get_logger().info(f"人员就位提示（距离 {msg.distance_m:.2f} m），等待人工 start")
        elif msg.event_type == GreetingEvent.EVENT_LEADER_LEAVE:
            self.get_logger().info("人员离场提示")

    def _on_person(self, msg: PersonDetection) -> None:
        """处理人员检测"""
        if msg.detected:
            self.get_logger().debug(
                f"感知到人员: 距离 {msg.distance_m:.2f} m 方位 {msg.bearing_rad:.2f} rad "
                f"置信度 {msg.confidence:.2f}"
            )

    def _on_intent(self, msg: GreetingIntent) -> None:
        """处理意图"""
        self.get_logger().info(f"收到意图 {msg.intent!r} 槽位 {msg.slot_json!r}")

    # ------------------------------------------------------------ 段/步推进
    def _start_segment(self, idx: int) -> None:
        """开始第 idx 段"""
        if not self._segments:
            self._set_state("IDLE")
            return
        self._abort_pending()
        idx = max(0, min(idx, len(self._segments) - 1))
        self._seg_idx = idx
        self._step_idx = 0
        self._paused = False
        seg = self._segments[idx]
        self._set_state(str(seg.get("name", f"SEG{idx + 1}")).upper())
        self.get_logger().info(
            f"开始第 {idx + 1} 段 [{self._state}]，共 {len(seg.get('steps', []))} 步"
        )

    def _resume_state(self) -> None:
        """恢复状态"""
        if 0 <= self._seg_idx < len(self._segments):
            seg = self._segments[self._seg_idx]
            self._set_state(str(seg.get("name", f"SEG{self._seg_idx + 1}")).upper())

    def _tick(self) -> None:
        """定时器"""
        if self._state not in self._running_states or self._paused:
            return
        if self._pending is not None:
            self._poll_pending()
            return
        self._start_next_step()

    def _poll_pending(self) -> None:
        """轮询待办任务"""
        p = self._pending
        if p["kind"] == "cue":
            self._poll_cue(p)
            return
        if not p["future"].done():
            return
        self._pending = None
        if p["stage"] == "goal":
            handle = p["future"].result()
            if handle is None or not handle.accepted:
                self.get_logger().warn(f"[{p['label']}] 目标被拒绝，跳过该步")
                self._advance_step()
                return
            self._goal_handle = handle
            self._pending = {
                "kind": p["kind"],
                "stage": "result",
                "future": handle.get_result_async(),
                "label": p["label"],
            }
            return

        # stage == 'result'
        wrapped = p["future"].result()
        status = getattr(wrapped, "status", GoalStatus.STATUS_UNKNOWN)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"[{p['label']}] 完成")
        else:
            self.get_logger().warn(f"[{p['label']}] 未成功（status={status}），继续下一步")
        self._goal_handle = None
        self._advance_step()

    def _start_next_step(self) -> None:
        """开始下一步"""
        seg = self._segments[self._seg_idx]
        steps = seg.get("steps", [])
        if self._step_idx >= len(steps):
            self._advance_segment()
            return
        step = steps[self._step_idx]
        kind = step.get("kind")
        if kind == "cue":
            self._start_cue(step)
        elif kind == "speak":
            if not self._speak.server_is_ready():
                self._warn_missing("speak")
                return
            goal = Speak.Goal()
            goal.text = str(step.get("text", ""))
            goal.interrupt = False
            goal.resume_point = 0.0
            label = f"speak:{goal.text[:12]}…"
            self._send(self._speak, goal, kind, label)
        elif kind == "motion":
            if not self._motion.server_is_ready():
                self._warn_missing("play_motion")
                return
            goal = PlayMotion.Goal()
            goal.motion_name = str(step.get("motion_name", ""))
            # 礼仪硬要求：编排层先钳一次（服务端还会再钳一次）
            goal.speed_scale = min(float(step.get("speed_scale", 0.5)), MOTION_SPEED_LIMIT)
            label = f"motion:{goal.motion_name}"
            self._send(self._motion, goal, kind, label)
        elif kind == "nav":
            if not self._nav.server_is_ready():
                self._warn_missing("navigate_to")
                return
            goal = NavigateTo.Goal()
            goal.waypoint = str(step.get("waypoint", ""))
            goal.timeout_s = float(
                step.get("timeout_s", self.get_parameter("default_nav_timeout").value)
            )
            label = f"nav:{goal.waypoint}"
            self._send(self._nav, goal, kind, label)
        else:
            self.get_logger().warn(f"未知 step 类型 {kind!r}，跳过")
            self._advance_step()

    def _send(self, client: ActionClient, goal, kind: str, label: str) -> None:
        """下发 Action 目标"""
        self.get_logger().info(f"下发 [{label}]")
        self._pending = {
            "kind": kind,
            "stage": "goal",
            "future": client.send_goal_async(goal),
            "label": label,
        }

    # --------------------------------------------------- cue：动作提前、语音跟随（并行）
    def _start_cue(self, step) -> None:
        """一步内并行编排「动作 + 语音」：先触发动作，延迟 lead_s 再触发语音。

        礼仪时序：大会开始打招呼 / 结束离场祝贺 lead_s=1.0，其余流程 lead_s=3.0。
        两个 Action 并发执行，只有【两者都结束】才推进下一步。
        """
        if not self._motion.server_is_ready():
            self._warn_missing("play_motion")
            return
        if not self._speak.server_is_ready():
            self._warn_missing("speak")
            return

        motion_name = str(step.get("motion_name", ""))
        text = str(step.get("text", ""))
        # 礼仪硬要求：编排层先钳一次（服务端还会再钳一次）
        speed = min(float(step.get("speed_scale", 0.5)), MOTION_SPEED_LIMIT)
        lead = max(0.0, float(step.get("lead_s", self._default_motion_lead_s)))

        goal = PlayMotion.Goal()
        goal.motion_name = motion_name
        goal.speed_scale = speed
        label = f"cue:{motion_name}+{text[:10]}…"
        self.get_logger().info(f"下发 [{label}]：动作先行，语音延后 {lead:.1f}s 触发")
        self._pending = {
            "kind": "cue",
            "label": label,
            "text": text,
            "motion": self._new_sub(self._motion.send_goal_async(goal)),
            "speak": None,
        }
        if lead <= 1e-6:
            self._send_cue_speak()
        else:
            self._cue_timer = self.create_timer(lead, self._send_cue_speak)

    def _send_cue_speak(self) -> None:
        """cue 的语音下发（提前量到点触发；也用作 lead_s=0 时的立即下发）。"""
        if self._cue_timer is not None:
            self._cue_timer.cancel()
            self._cue_timer = None
        p = self._pending
        if p is None or p.get("kind") != "cue" or p["speak"] is not None:
            return
        goal = Speak.Goal()
        goal.text = p["text"]
        goal.interrupt = False
        goal.resume_point = 0.0
        p["speak"] = self._new_sub(self._speak.send_goal_async(goal))
        self.get_logger().info(f"[{p['label']}] 语音已下发")

    def _poll_cue(self, p) -> None:
        """轮询 cue 任务"""
        self._poll_sub(p["motion"])
        if p["speak"] is not None:
            self._poll_sub(p["speak"])
        if not p["motion"]["done"] or p["speak"] is None or not p["speak"]["done"]:
            return
        self._log_sub(p["label"], "动作", p["motion"])
        self._log_sub(p["label"], "语音", p["speak"])
        if self._cue_timer is not None:
            self._cue_timer.cancel()
            self._cue_timer = None
        self._pending = None
        self._advance_step()

    @staticmethod
    def _new_sub(future) -> dict:
        """构造一个 Action 子任务状态块（goal -> result 两阶段）。"""
        return {
            "stage": "goal",
            "future": future,
            "handle": None,
            "done": False,
            "status": None,
            "rejected": False,
        }

    def _poll_sub(self, sub) -> None:
        """推进单个 Action 子任务：goal 阶段等接受 -> result 阶段等结束。"""
        if sub["done"]:
            return
        if sub["stage"] == "goal":
            if not sub["future"].done():
                return
            handle = sub["future"].result()
            if handle is None or not handle.accepted:
                sub["rejected"] = True
                sub["done"] = True
                return
            sub["handle"] = handle
            sub["stage"] = "result"
            sub["future"] = handle.get_result_async()
            return
        if not sub["future"].done():
            return
        wrapped = sub["future"].result()
        sub["status"] = getattr(wrapped, "status", GoalStatus.STATUS_UNKNOWN)
        sub["done"] = True

    def _log_sub(self, label: str, what: str, sub) -> None:
        """记录 Action 子任务状态"""
        if sub["rejected"]:
            self.get_logger().warn(f"[{label}] {what}目标被拒绝，继续下一步")
        elif sub["status"] == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"[{label}] {what}完成")
        else:
            self.get_logger().warn(
                f"[{label}] {what}未成功（status={sub['status']}），继续下一步"
            )

    def _warn_missing(self, server: str) -> None:
        """警告缺失服务端"""
        if server not in self._missing_logged:
            self._missing_logged.add(server)
            self.get_logger().warn(f"动作服务端 /greeting/{server} 不在线，等待其启动…")

    def _advance_step(self) -> None:
        """推进到下一步"""
        self._step_idx += 1
        seg = self._segments[self._seg_idx]
        if self._step_idx >= len(seg.get("steps", [])):
            self._advance_segment()

    def _advance_segment(self) -> None:
        """推进到下一段"""
        if self._single_segment:
            # goto 点播：本段（动作 + 语音）已结束，不续播下一段
            self.get_logger().info(f"单段点播完成（第 {self._seg_idx + 1} 段），回到 IDLE")
            self._seg_idx = -1
            self._step_idx = 0
            self._set_state("IDLE")
            return
        if self._seg_idx + 1 < len(self._segments):
            self._start_segment(self._seg_idx + 1)
        else:
            self.get_logger().info("讲稿全部执行完毕")
            self._seg_idx = -1
            self._step_idx = 0
            self._set_state("IDLE")

    def _abort_pending(self) -> None:
        """取消待办任务"""
        if self._cue_timer is not None:
            self._cue_timer.cancel()
            self._cue_timer = None
        p = self._pending
        if p is not None and p.get("kind") == "cue":
            # cue 有动作/语音两个在途目标，逐个取消（含"已发但尚未被接受"的）
            for key in ("motion", "speak"):
                self._cancel_sub(p.get(key))
        elif p is not None and p.get("stage") == "goal":
            # 单步目标（speak/motion/nav）已发但尚未被接受：受理后立即补发取消
            p["future"].add_done_callback(self._cancel_late_goal)
        elif self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._goal_handle = None
        self._pending = None

    def _cancel_sub(self, sub) -> None:
        """取消 cue 的一个在途子目标；若目标尚未被受理，则挂回调在受理后立即取消。

        必须在受理后补一刀：goal 已发出、响应未回的窗口内无处可 cancel，
        若不处理，急停后该目标仍会被受理并执行（语音照常播完）。
        """
        if sub is None or sub.get("done"):
            return
        if sub.get("handle") is not None:
            sub["handle"].cancel_goal_async()
            sub["handle"] = None
        else:
            sub["future"].add_done_callback(self._cancel_late_goal)

    @staticmethod
    def _cancel_late_goal(future) -> None:
        """目标在"已请求作废"后才被受理时，立即补发取消。"""
        try:
            handle = future.result()
        except Exception:  # noqa: BLE001 — 回调边界，取消失败不应抛出
            return
        if handle is not None and getattr(handle, "accepted", False):
            handle.cancel_goal_async()


def main() -> None:
    rclpy.init()
    node = GreetingOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()