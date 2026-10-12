# greeting_voice

天轶 2.5 迎宾项目语音包（Owner: B / Voice Owner）。

## 用途

本包含**两个**语音节点：

| 节点 | 触发方式 | 说明 |
| --- | --- | --- |
| `simple_action_voice_node` | `H` 右拨 + 连按 `B`×N + 短按 `A` | 平台「简单动作模式」动作的配套语音（订阅 `/sbus_data/event`） |
| `speak_action_server` | 编排层自动调用 `/greeting/speak` | 讲稿语音服务端，由状态机驱动（支撑「动作比语音早 1s/3s 触发」的时序编排） |

> ⚠️ 讲稿的按键分段点播由 `joy_mapper`（`E` 上/下拨 + `A/B/C/D` → `goto:<段号>`）经
> 编排层驱动 `speak_action_server` 完成，语音包内不再重复实现按键播报逻辑
> （历史上曾有的 `voice_greet_tts_node`（`G` 左/右拨 + `A/B/C/D`）会与 `joy_mapper`
> 的 `goto:<段号>` 撞车并盖过讲稿语音，已删除）。

### simple_action_voice_node（平台「简单动作模式」语音）

平台 `joystick_bridge_node` 对「**H 右拨 + 连按 B×N + 短按 A**」的响应，是直接回放
`share/joystick/param/simpleaction/*.json` 的关节轨迹（**不经过本项目** joy_mapper / orchestrator），
因此这条路径默认没有任何语音。本节点在项目侧补齐：并行监听同一份按键事件流，用**与平台完全一致**的规则
识别序号 N，再调用本体 TTS 排队播报第 N 句文本。

识别规则（🔴 必须与平台 `bridge_config.yaml` 对齐，否则"按 2 次 B"会出现动作对、语音错位）：

- **待命**：H 拨到「右」（`KEY_H_RIGHT`）；H 离开右档（LEFT/MID）撤销待命；
- **计数**：`B` 按下（`KEY_B_DOWN`）累加——距上次有效按下 > `press_max_interval_s`(2.0) 则清零重计；
  距上次 < `press_min_interval_s`(0.15) 视为抖动忽略；
- **触发**：`A` **短按**（按下到松开 < `short_press_s`(1.0)，与 `long_press_thresholds.a` 一致）；
- **序号 N ∈ [1,4]** 且文本非空才播报（1=挥手 2=击掌 3=碰拳 4=敬礼）。

⚠️ 本节点**只发语音**，绝不发布 `/arm|/head|/waist/cmd`；动作仍由平台节点负责。
🛑 **急停**：订阅 `/greeting/panel_command`，收到 `stop`/`manual` 时以 `cmd=stop, audio_id=<audio_id>`
打断本节点播报（平台规则：stop 必须带 `audio_id`，且该实体须在 `audio_config` 开启 stop 权限）。

## 依赖

- `rclpy`
- `bodyctrl_msgs`、`interaction_msgs` —— **机器人专属消息包，来自算力板 `~/xos` 工作空间**，
  本机（开发 PC）没有，需在机器人上 `source ~/xos/setup.bash` 后再编译（协作规范 §11.6）。

## 编译（在机器人上）

```bash
source /opt/ros/jazzy/setup.bash
source ~/xos/setup.bash
cd 工作空间
colcon build --symlink-install --packages-select greeting_voice
source install/setup.bash
```

## 运行

`voice_greet.launch.py` 一次启动 `simple_action_voice_node` + `speak_action_server`：

```bash
# 正常播报
ros2 launch greeting_voice voice_greet.launch.py

# 无硬件干跑（只打日志，不调 TTS，用于确认按键链路）
ros2 launch greeting_voice voice_greet.launch.py dry_run:=true

# 只起按键语音、不起 /greeting/speak 动作服务端
ros2 launch greeting_voice voice_greet.launch.py speak_server:=false

# 单独跑某节点
ros2 run greeting_voice simple_action_voice_node
ros2 run greeting_voice speak_action_server
```

## 接口清单

| 接口 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/sbus_data/event` | Topic `bodyctrl_msgs/msg/SbusData` | 订阅 | 遥控器按键事件（`simple_action_voice_node`） |
| `/greeting/panel_command` | Topic `std_msgs/String` | 订阅 | 急停命令来源；收到 `stop`/`manual` 即打断播报（`simple_action_voice_node`） |
| `/greeting/speak` | Action `greeting_interfaces/action/Speak` | 服务端 | 编排层驱动的播报入口（`speak_action_server`） |
| `/intelligent_interaction/tts/play` | Service `interaction_msgs/srv/TtsService` | 调用 | `query` 查状态 / `append` 排队播报 / `stop` 打断 |

## 参数（config/voice.yaml）

`voice.yaml` 存放 `simple_action_voice_node` 的参数；`speak_action_server` 使用节点内置默认值。

### simple_action_voice_node

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `sbus_topic` | `/sbus_data/event` | 按键事件话题 |
| `tts_service` | `/intelligent_interaction/tts/play` | TTS 服务名 |
| `audio_id` | `audio_guide` | 播报/TTS 实体，须支持 `stop`（同急停要求） |
| `command_topic` | `/greeting/panel_command` | 急停命令来源（收到 `stop`/`manual` 打断播报） |
| `text_1` … `text_4` | 挥手/击掌/碰拳/敬礼台词 | 序号 N 对应文本（留空则该序号只做动作不播报）；🔴 序号须与平台 `SIMPLE_ACTION_FILES` 一一对应 |
| `press_min_interval_s` | `0.15` | B 连按抖动间隔下限（须与平台 `bridge_config` 一致） |
| `press_max_interval_s` | `2.0` | B 连按计数清零间隔（须与平台 `bridge_config` 一致） |
| `short_press_s` | `1.0` | A 短按判定阈值（须与 `long_press_thresholds.a` 一致） |
| `service_timeout_s` | `5.0` | TTS 服务无应答超时复位 |
| `dry_run` | `false` | 只打日志不调服务 |

## 已知限制

- 按键规则仅实现一种：`simple_action_voice_node` 的「H 右拨 + 连按 B×N + 短按 A → 播第 N 句」；
  mp3 文件播放等原 `audio_manager` 链路不在本包范围（按需在 `greeting_dialog`/编排层扩展）。
- **打断（stop）**：本体 TTS 实际**支持**停止，但有两个前提——`cmd='stop'` 必须带
  `audio_id`（纯 stop 会被平台忽略），且该 `audio_id` 需在 `audio_config.json` 中开启
  `"stop": true`（当前文本型实体 `audio_guide` 满足）。已实测可用。
  - `speak_action_server`（编排层自动播报）已用该机制：急停取消语音 goal 时下发
    `cmd='stop', audio_id='audio_guide'`，真正打断音频（见其 `audio_id` 参数）；
  - `simple_action_voice_node`（按键人工播报）也已接入：
    订阅 `/greeting/panel_command`，收到 `stop`/`manual` 即下发
    `cmd='stop', audio_id='audio_guide'` 打断播报；其 `append` 同样带该 `audio_id`，
    保证急停停得掉。可通过 `audio_id` / `command_topic` 参数调整。
- 实机验证前必须先 `ros2 topic echo /sbus_data/event` 核对 KEY 常量数值与固件一致。
