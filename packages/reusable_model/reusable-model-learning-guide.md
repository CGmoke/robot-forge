# reusable_model 全模块学习指南

> 面向 `packages/reusable_model` 的完整学习文档。既可作为入门者的第一份导读，
> 也可作为进阶开发者的模块速查手册。

**文档基线（已实测）**

| 项目 | 值 |
| --- | --- |
| 包版本 | `reusable_model 0.1.0` |
| 代码规模 | 22 个模块，约 19.7k 行（含 8 个子包 `__init__.py`） |
| 硬依赖 | `numpy`（唯一） |
| 可选依赖 | OpenCV、PyArrow、PySerial、minimalmodbus、SciPy（全部惰性导入）、PyYAML（模块顶层导入） |
| 单元测试 | 128 个用例，全部通过 |
| 构建 | `setuptools` + src 布局（`src/reusable_model/`、`tests/`） |

> **边界说明**：本包是**纯 Python 算法库**，不是 ROS 功能包——它不含 `package.xml`、
> 不含 launch 文件、不发布话题或 TF。它的定位是"被 ROS/机器人工程调用的算法构建块"。
> 要把它们接入 ROS 2 节点，请在 `projects/` 下新建工程（规范见
> [`docs/adding-a-project.md`](./adding-a-project.md)）。

---

## 一、整体架构与模块划分逻辑

### 1.1 三条设计原则

| 原则 | 含义 | 在代码中的体现 |
| --- | --- | --- |
| 复用下沉 | 多项目共用的算法抽成库，杜绝复制粘贴 | 8 个子包全部可独立复用，互不耦合 |
| 与机器人解耦 | 不含绝对路径、不读环境变量、不依赖具体机器人配置 | 全包无 `os.environ` 读取；无项目专属配置 |
| 自动化守护 | 单测 + doctest 双重保护 | 128 个单测 + 每个模块内嵌可执行 doctest |

两条贯穿全包的编码约定：**日志统一走 `logging.getLogger(__name__)`**（全包无 `print()`）；
**可选重型依赖一律在使用处惰性导入**，因此未安装 OpenCV/PyArrow/PySerial 时，包内其余部分仍可正常工作。

### 1.2 为什么这样划分？——按"机器人软件栈的纵向切面"切

模块不是按"项目"或"文件类型"划分，而是按**机器人系统自下而上的能力切面**划分。
每个子包对应一条明确的工程主线：

```
感知输入 ──► 几何/视觉 ──► 目标跟踪 ──► 地图与导航 ──► 运动执行 ──► 硬件驱动
                 │             │              │              │
                 └─────────────┴──────────────┴──────────────┘
                    数据落盘与配置 (io) ＋ 进程/原生库支撑 (runtime)
```

| 子包 | 对应切面 | 一句话定位 |
| --- | --- | --- |
| `geometry` | 数学基础 | 二维框几何 + 三维旋转代数，**全包的依赖基座** |
| `vision` | 视觉感知 | 深度/彩色图像 → 机器人可用的三维几何 |
| `tracking` | 目标跟踪 | 检测框跨帧关联 + 稳定判定 |
| `gridmap` | 地图与导航 | 占据栅格 → 连通区域 → 距离场 → 下降航向 |
| `motion` | 运动执行 | 平面逆运动学 + 制动感知速度曲线 |
| `hardware` | 硬件驱动 | 串口稳定发现 + Modbus 夹爪控制 |
| `io` | 数据与配置 | YAML 配置、二进制信封、检测批次 NPZ |
| `runtime` | 运行时支撑 | 子进程桥接、ctypes 原生库加载、输出流过滤 |

### 1.3 分层结构（依赖方向自上而下，箭头指向被依赖方）

```
                  ┌───────────────┐
 L0 基础层        │   geometry    │   ← 全包唯一被"跨子包"依赖的模块
                  └───────┬───────┘
              ┌───────────┴───────────┐
 L1 消费层    ▼                       ▼
        ┌──────────┐          ┌──────────┐
        │ tracking │          │    io    │
        └──────────┘          └──────────┘

 L2 自包含层（彼此独立，可任意顺序学习）
   ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐
   │ gridmap  │  │  motion  │  │  vision  │  │ runtime  │  │ hardware │
   └──────────┘  └──────────┘  └──────────┘  └──────────┘  └──────────┘
        （各自内部有严格单向的模块链，见第三节）
```

### 1.4 全模块清单（按代码行数排序，供学习优先级参考）

| 子包 | 模块 | 行数 | 外部依赖 |
| --- | --- | --- | --- |
| `gridmap` | distance_field.py | 1957 | numpy（+cv2 惰性） |
| `gridmap` | regions.py | 1110 | numpy（+cv2 惰性） |
| `gridmap` | occupancy.py | 832 | numpy |
| `gridmap` | descent.py | 609 | numpy |
| `motion` | planar_ik.py | 1492 | numpy（+scipy 惰性） |
| `motion` | speed_profile.py | 344 | 仅标准库 |
| `vision` | pointcloud.py | 1478 | numpy（+scipy 惰性） |
| `vision` | planes.py | 1202 | numpy（+cv2 惰性） |
| `vision` | depth.py | 990 | numpy（+cv2 惰性） |
| `vision` | pinhole.py | 854 | numpy |
| `io` | yaml_config.py | 1349 | numpy 无关；**PyYAML（顶层导入）** |
| `io` | binary_envelope.py | 1032 | numpy（+pyarrow/cv2 惰性） |
| `io` | detection_npz.py | 177 | numpy + geometry |
| `tracking` | iou_tracker.py | 268 | geometry |
| `tracking` | stable_streak.py | 125 | geometry |
| `geometry` | rotations3d.py | 1115 | numpy |
| `geometry` | boxes.py | 278 | numpy |
| `hardware` | modbus_gripper.py | 1328 | **PyYAML（顶层）** + pyserial/minimalmodbus（惰性） |
| `hardware` | serial_discovery.py | 928 | **PyYAML（顶层）** + pyserial（惰性） |
| `runtime` | subprocess_bridge.py | 998 | 仅标准库 |
| `runtime` | ctypes_loader.py | 980 | 仅标准库 |
| `runtime` | stdout_filter.py | 189 | 仅标准库 |

### 1.5 全包统一的领域约定（学任何模块前先掌握）

这些约定是理解代码的"公共词典"，散布在多个模块中但必须一致理解：

| 约定 | 具体内容 | 出现的模块 |
| --- | --- | --- |
| **边界框** | 恒为 `(x1, y1, x2, y2)`，左上角包含、右下角不包含（OpenCV/YOLO 约定）；第 4 个元素之后的多余元素被忽略 | `geometry.boxes`、`tracking`、`io.detection_npz` |
| **像素坐标** | `data[py, px]`，行 0 对应世界 y 最小值；图像/PGM/ROS `map_server` 约定是"行 0 在顶部"，用 `flip_vertically()` 互转 | `gridmap.occupancy` |
| **世界坐标转换** | 像素中心 = `origin + (pixel + 0.5) * resolution` | `gridmap`、`vision` |
| **相机坐标系** | `+x` 右、`+y` 下、`+z` 前（相机光学坐标系） | `vision.*` |
| **深度单位** | 原始 `uint16` 毫米，`depth_scale` = 每米对应原始单位数（默认 1000.0） | `vision.depth`、`vision.plane` |
| **旋转矩阵** | 右手系、列向量约定，`R[:, i]` 是第 i 个基向量的像；与 scipy 一致 | `geometry.rotations3d` |
| **异常契约** | 参数类型错 → `TypeError`；取值/形状不合法 → `ValueError`（或子类）；"无结果"用 `None` 而非抛异常 | 全包 |
| **哨兵值** | 距离场不可达 = `-1.0`（`DistanceField.UNREACHABLE`），不用 `inf` | `gridmap.distance_field` |

---

## 二、各模块功能定位、核心作用与适用场景

> 每节末尾给出**适用 / 不适用**边界，避免误用。

### 2.1 `geometry` —— 全包的数学基座

#### `boxes.py`：二维轴对齐边界框工具箱

- **功能定位**：纯函数工具箱，把"两个框有多重叠"和"框与掩码互转"这两件事做到无歧义。
- **核心作用**：提供 `iou` / `iou_min` / `overlap_score` 三种重叠度量、`clip_bbox` / `bbox_to_mask` 转换、`bbox_center` / `mask_center` 质心。
- **为什么有三种度量**：`iou` 对小框被大框包住的情形评分偏低，`iou_min`（交集÷较小框面积）则给出 1.0；`overlap_score = max(iou, iou_min)` 专为跟踪设计，避免"小目标贴合大目标反而失配"。
- **适用**：检测后处理、掩码裁剪、跟踪匹配打分。
- **不适用**：非轴对齐的旋转框、三维框。

#### `rotations3d.py`：三维旋转代数

- **功能定位**：不依赖 scipy 的完整 3D 旋转工具箱（自带 Rodrigues 公式与欧拉角提取）。
- **核心作用**：
  - 由轴约束构造旋转矩阵：`rotation_from_two_axes` / `rotation_from_axis`（用于"让工具 z 轴指向某方向"这类机器人姿态任务）；
  - 向量间最小旋转：`rotation_between`；
  - 欧拉角互转：`euler_to_matrix` / `matrix_to_euler`，**支持全部 24 种序列且对万向锁安全**；
  - 关节限位处理：`wrap_to_limits`（按 2π 折叠角度）、`select_euler_branches`（在分支中挑选既满足限位、又最接近参考角的解）。
- **适用**：机械臂姿态解算、标定外参、可视化姿态。
- **不适用**：需要微分/优化求解的场景（本模块只做解析几何）。
- **进阶难点**：`select_euler_branches` 是"先取主分支 → 构造次分支 → 逐角折叠进限位 → 按 (是否全在限位内, Σ|角-参考|) 字典序打分"的策略，需要读懂 `wrap_to_limits` 才能理解。

---

### 2.2 `vision` —— 从图像到三维几何

模块间的数据流严格单向：`pinhole` / `depth`（叶子）→ `pointcloud` / `planes`（消费方）。

| 模块 | 功能定位 | 核心作用 | 适用场景 |
| --- | --- | --- | --- |
| `pinhole.py` | 针孔相机模型 | `Intrinsics` 内参（可由矩阵、ROS `CameraInfo` 字典或缩放构造）；`pixel_to_3d` 与 `project` 双向转换；`ray_direction` 单位视线 | 相机标定接入、三维投影、像素↔射线 |
| `depth.py` | 深度图预处理 | 鲁棒采样 `sample_depth`（中位数/均值/分位数）、`bbox_depth` 区域深度、`valid_mask` 有效掩码、`depth_to_meters` 单位换算；**无测量时返回 `None` 而非可疑数值** | 深度空洞过滤、目标距离测量 |
| `pointcloud.py` | 点云短流程 | `backproject_depth` 反投影、`grid_sample`/`random_sample` 采样、`voxel_downsample` 降采样、`filter_by_height`/`filter_by_horizontal_distance` 几何滤波、`transform_points*` 坐标变换、`write_ply` 导出 | 避障点云生成、障碍物提取、点云导出 |
| `planes.py` | 平面几何 | 总体最小二乘 `fit_plane`/`fit_plane_oriented`、`ray_plane_intersection` 射线求交、`pixel_to_plane` 像素-平面落点、`sample_ground_ring` 地面环形采样、`bbox_3d_area` 物理面积 | 地面检测、物体落点定位、抓取面估计 |

- **适用**：RGB-D 感知、地面/平面估计、抓取目标定位。
- **不适用**：深度学习检测本身（本包只做后处理几何）；`render_depth`、`color_ratio_in_bbox`、`fit_ellipse_to_points`、`compute_normals` 需要 OpenCV 或 SciPy。

---

### 2.3 `tracking` —— 检测框的时间连续性

| 模块 | 功能定位 | 核心作用 |
| --- | --- | --- |
| `iou_tracker.py` | 贪心 IoU 多目标跟踪 | `IoUTracker.update()` 逐帧分配稳定 `track_id`；`max_age` 控制轨迹过期；`min_hits` 实现 SORT 式**确认门限**；`pick_primary_track_id` 按面积（并列时按置信度）挑主目标；`KeyedTrackerStore` 以 key 隔离多相机/多会话状态并加锁保证线程安全 |
| `stable_streak.py` | 稳定连续帧判定 | `StableStreak.update(track_id, bbox)` 统计"同一 track_id 且框间 IoU ≥ `stable_iou`"的连续帧数，达 `min_stable_frames` 即判稳定；**track_id 变化、位置跳变、无检测三者都会重置计数** |

- **设计取舍**：没有运动模型、外观描述子或线性分配（匈牙利算法），属于"够用即可"的轻量方案，适合算力受限或目标稀疏的场景。
- **适用**：抓取前的目标稳定确认、凝视锁定、忽隐忽现目标过滤。
- **不适用**：密集人群、长时间遮挡、需要外观重识别的场景。

---

### 2.4 `gridmap` —— 二维导航的完整主线

四个模块构成**严格单向链**：`occupancy` → `regions` → `distance_field` → `descent`。

#### `occupancy.py` —— 把坐标约定显式化

- **功能定位**：解决导航 bug 的头号来源——"图像哪个角是原点、y 轴朝哪增长"。
- **核心作用**：`GridGeometry`（不可变坐标系，可安全共享）、`OccupancyGrid`（像素数组 + 坐标系）、`world_to_pixel`/`pixel_to_world` 及其批量版、`value_at_world` 取值、`to_ros_yaml`/`from_ros_yaml` 与 ROS `map_server` 互操作、`flip_vertically()` 在两种行序约定间转换、`region_labels()` 产出可通行掩码。
- **灰度常量**：`FREE=254`、`OCCUPIED=0`、`UNKNOWN=205`（值越大越空闲）。
- **适用**：加载 ROS 静态地图、SLAM 输出后处理、可通行区域提取。
- **进阶难点**：`from_ros_yaml` 会**翻转图像**以适配内部约定，且会忽略（并告警）origin 的 yaw 分量。

#### `regions.py` —— 拓扑级区域分析

- **功能定位**：回答"哪些格子同属一个房间、房间之间怎么通过门洞相连、房间里哪点一定在内部"。
- **核心作用**：`UnionFind`（路径压缩 + 按大小合并，支持稠密/稀疏两种模式）、`label_connected`（普通连通分量）、`label_with_step_constraint`（拒绝高度差 > `max_step` 的相邻格，把楼梯/坡道切成独立房间）、`merge_regions_by_dilation`（经门洞焊接区域，**需 cv2**）、`region_point_inside`/`region_centroid_world`（区域代表点）、`connectivity_from_segments`（由门洞线段推导区域连通图）。
- **适用**：多房间导航、楼层平面拓扑、门洞检测。
- **注意**：`label_connected` 输出的 `id == 1` 是最大区域。

#### `distance_field.py` —— 米制最短路径距离场

- **功能定位**：为区域内**单个目标**构建到所有格子的米制距离。
- **核心作用**：`DistanceField.generate()`（裁剪到掩码包围盒 → 重采样 → 建速度掩码 → 单源 Dijkstra）；两种算法可选（堆版 `dijkstra_multi_seed` vs numpy 波前版 `dijkstra_multi_seed_numpy`，后者在大种子集合时更快）；带制动斜坡的速度掩码 `is_inside`；动态障碍时间戳 `update_obstacle_timestamps` + `valid_obstacle_mask`（配合 `obstacle_timeout` 实现障碍记忆）；`recompute_local` 局部重规划；`create_distance_field_for_region` 便捷入口（先合并区域再建场）。
- **关键设计**：不可达格子存 `UNREACHABLE = -1.0` 而非 `inf`——便于定点缓冲、不污染算术、有效性检查退化为 `value < 0`。
- **适用**：DWA/纯跟踪类局部规划的距离代价、可通行性评估、制导。
- **进阶难点**：这是全包最大、最复杂的模块（1957 行）。`from_array` 重建的场**不支持 `recompute_local`**（会抛明确错误）。

#### `descent.py` —— 连续下降航向

- **功能定位**：用"米制半径内加权最小二乘平面拟合"取代"看 8 个邻居挑最便宜的"，得到连续、无栅格量化、抗噪声的航向。
- **核心作用**：`descent_direction`（返回 `(-π, π]` 航向弧度，退化时返回 `None`）、`predict_path`（逐步重拟合预览路径）、`projected_motion_distance`（沿当前航向还能走多远）。
- **关键设计**：负值与非有限值视为"无数据"被排除在拟合外；`_MIN_SAMPLES = 3`，样本不足即返回 `None`。
- **调用方责任**：**必须把 `None` 当作"没有可信航向"**，而不是当作 0 弧度。

---

### 2.5 `motion` —— 位置映射与时序控制

两个模块完全独立，各自闭合。

#### `planar_ik.py` —— 平面逆运动学与通用最小二乘

- **功能定位**：竖直平面（x 前 / z 上，`q = 0` 表示沿 `+z`）内双连杆链的闭式解 IK，并附带肘部冗余参数化与通用 LM 求解器。
- **核心作用**：
  - `PlanarTwoLink`（几何：`l1`/`l2`/`base_x`/`base_z`）+ `PlanarTwoLinkIK.solve()` 解析求解，`q2_limits` 决定肘分支，不可达/超限**返回 `success=False` 而不抛异常**；`verify()` 回代 `forward` 计算往返残差；
  - `elbow_circle` → `lowest_point_on_circle` / `circle_plane_intersections` → `select_elbow_on_circle`：为 7 自由度手臂参数化肘部可达圆并选姿态（优先最低肘，否则滑到约束平面取交点）；
  - `numeric_jacobian`（中心差分）+ `levenberg_marquardt`（带 Armijo 线性搜索与自适应阻尼 λ，返回**迄今最优**点而非最后一点）。
- **适用**：足端/手端到关节角解算、冗余手臂姿态选择、通用非线性最小二乘精修。
- **注意**：`solve_numeric` 需要 SciPy（惰性导入）；实时控制路径只依赖 numpy。

#### `speed_profile.py` —— 制动感知速度曲线

- **功能定位**：纯运动学，不涉及传感器/障碍物/硬件。
- **核心作用**：`BrakingProfile.motion_strategy()` 综合"制动距离 `v = sqrt(2ad)` + 加速度斜坡钳制（防单步突变）+ 临近目标点指数阻尼减速（防超调）"，输出本控制周期应下发的速度；`target_velocity()` 单步斜坡；静态 `braking_velocity()` 反解；线性轴与角度轴统一接口（`angular=True`）。
- **适用**：底盘驶向目标点、关节/转向轴减速停止。
- **注意**：`distance` 与 `target_linear_velocity` 至少给一个，同时给出时 `distance` 优先。

---

### 2.6 `hardware` —— 硬件层的两个兜底

两个模块相互独立；典型配合是"先探测、后驱动"。

#### `serial_discovery.py` —— 串口稳定发现

- **功能定位**：解决"两个完全相同的 USB 转 RS-485 适配器如何稳定区分左右"。
- **核心作用**：
  - 底层：`list_by_id_ports`（glob/scandir 扫 `/dev/serial/by-id`，排序使左右分配可复现）、`list_ports`（PySerial 列举，可带 vid/pid 过滤）、`find_by_serial`（按序列号片段查找）；
  - 探测：`probe_port`（守护线程 + 超时 join，任何异常视为"否"）；
  - 四层解析器 `resolve_ports`：**显式路径 → 映射文件 → 代码内序列号 → 自动探测**，且**解析失败只告警、从不抛出**（只有 spec 格式错才抛）。
- **注意**：本模块在**模块顶层 `import yaml`**（非惰性），所以导入 `reusable_model.hardware` 需要 PyYAML。

#### `modbus_gripper.py` —— Modbus RTU 并联夹爪

- **功能定位**：驱动单个 Modbus-RTU 并联夹爪（内置 Robotiq 2F 寄存器映射）；换厂商只需换 `GripperProfile`。
- **核心作用**：`GripperProfile`（不可变映射，构造时校验协议范围）、`decode_status`（解码 `gACT`/`gGTO`/`gSTA`/`gOBJ`/`gFLT`/`gPO`/`position`）、`activate`（轮询到 `gSTA == 3`）、`go_to`/`open`/`close`、`wait_motion_complete`、`is_object_held`（`gOBJ == 2` + 位置测试）、`squeeze_more`（**只收紧、绝不放松**的抓取重试，遇重大故障短路）、`has_major_fault`。
- **关键设计**：**`dry_run=True`** 时不导入依赖、不打开串口、不发一个字节，改为对内部状态寄存器模拟整套命令序列，并记录到 `dry_run_journal`。
- **安全边界**：驱动本身**不提供急停**；`reset()`/`open()`/任何"更宽"指令都会松手；`MAJOR_FAULTS`（`{0xA..0xF}`）出现后禁止任何指令。

---

### 2.7 `io` —— 数据落盘与配置

三个实现文件彼此不 import，能力各自独立。

| 模块 | 功能定位 | 核心作用 |
| --- | --- | --- |
| `yaml_config.py` | "数据类即模式"的配置系统 | `load_yaml`/`dump_yaml`（**同目录临时文件 + `os.replace` 原子写**）；`from_dict`/`load_config` 沿数据类字段递归实例化，支持 `aliases`（旧扁平键 → 新点分路径，`setdefault` 语义）与 `strict` 模式；`env_prefix` 环境变量覆盖顶层标量字段；`resolve_path` 逻辑路径 → 运行期根；`YamlStore` 目录级缓存文档集合（`threading.RLock` 保护） |
| `binary_envelope.py` | JSON 头部 + N 个二进制附件的单 blob 信封 | `pack`/`unpack`（`raw` 后端用 `struct` 长度前缀帧，含幻数 `ENV1` 与版本号便于检测截断；`arrow` 后端走 Arrow IPC 流，**需 pyarrow**）；`require_content_type`（HTTP 415 映射）；`normalize_choice`；图像辅助 `pack_image`/`rgb888_to_array`（**零拷贝只读视图**）/`jpeg_to_array`（需 cv2） |
| `detection_npz.py` | 检测批次自描述压缩载荷 | `pack_detections`/`unpack_detections`，布局为 `count` + `masks(N,H,W)` + `bboxes(N,4)` + `confidences(N,)` + 各字符串字段 + `str_field_names`；**缺 `mask` 时用 `bbox_to_mask` 自动补齐**；空列表往返仍为 `[]` |

- **适用**：推理进程与决策进程之间传输检测结果、图像帧协议、机器人参数管理。
- **注意**：`yaml_config.py` **顶层 `import yaml`**（需 PyYAML）。

---

### 2.8 `runtime` —— 运行时底层支撑

三个模块**零第三方依赖**（纯标准库），彼此也互不导入。

| 模块 | 功能定位 | 核心作用 |
| --- | --- | --- |
| `subprocess_bridge.py` | 父/子解释器之间的行协议桥接 | 父侧 `SubprocessBridge` 启动外部运行时并写换行分隔命令帧；子侧 `run_bridge_child` 逐行分派给回调；内置 stderr 抽干线程（防管道填满死锁）、启动期存活校验；**链路断开即降级返回 `False`**（对控制回路友好）；`BridgeStartError` 携带 argv/退出码/stderr |
| `ctypes_loader.py` | 通过 ctypes 驱动 C/C++ 厂商 SDK | `load_library`（按绝对路径预加载依赖，`RTLD_GLOBAL`）；`declare` 声明签名；`CFunction` 包装并**校验返回码**（`ok_codes`/`error_map`/`try_call`）；`poll_until_ready` 轮询异步就绪；`render_c_shim` + `build_shared_library` 生成并编译 `extern "C"` 垫片（解决 ctypes 不能直接调 C++） |
| `stdout_filter.py` | 不改代码抑制噪声 | `FilteredStream`（按整行丢弃含禁用子串的行，可连带吞掉紧随空行，保留 `partial` 行在 `flush()` 时输出）；`install_filter`（幂等安装到 stdout/stderr）；`quiet_loggers`（`logging` 层级静音） |

- **适用**：把"不能同进程导入的外部依赖"隔离到子进程、驱动无 Python 绑定的原生库、压制控制循环刷屏。
- **注意**：`subprocess_bridge` 与 `ctypes_loader` 通常位于**不同进程/解释器**，代码层面无调用关系，只是概念上配合。

---

## 三、依赖关系、调用链路与协作逻辑

### 3.1 跨子包依赖：只有 3 处，且全部指向 `geometry.boxes`

| 出处 | 引用 |
| --- | --- |
| `tracking/iou_tracker.py:26` | `from ..geometry.boxes import bbox_area, iou as _box_iou` |
| `tracking/stable_streak.py:25` | `from ..geometry.boxes import iou as _box_iou` |
| `io/detection_npz.py:32` | `from ..geometry.boxes import bbox_to_mask` |

**修改 `boxes.py` 中这些函数的签名或行为会直接影响 `io` 与 `tracking`。** `geometry` 自身不反向依赖任何子包。

其余 6 个子包（`gridmap`、`motion`、`vision`、`hardware`、`runtime`）**没有任何跨子包 import**，可任意顺序学习：

```
geometry ──► tracking
         └─► io            （仅 boxes 被引用）
gridmap, motion, vision, hardware, runtime   （自包含，零跨子包依赖）
```

### 3.2 子包内部调用链

**gridmap（严格单向）**

```
occupancy.py   （无内部依赖）
   │  GridGeometry / OccupancyGrid.region_labels()
   ▼
regions.py     （依赖 occupancy.GridGeometry）
   │  label_connected / label_with_step_constraint → labels, RegionInfo[]
   │  merge_regions_by_dilation（可选，需 cv2）
   ▼
distance_field.py  （依赖 occupancy.GridGeometry + regions.merge_regions_by_dilation）
   │  generate(...) → distances / speed_mask / boundary_mask
   ▼
descent.py     （仅 numpy）
   descent_direction / predict_path / projected_motion_distance
```

**vision（单向无环）**

```
pinhole.py（叶子）        ─┐
                          ├─► pointcloud.py   （backproject/grid_sample/random_sample + valid_mask）
depth.py（叶子）          ─┘
                          └─► planes.py       （sample_ground_ring + ray_direction + ray_plane_intersection）
```

**其余子包内部**：`motion`（planar_ik ∥ speed_profile）、`io`（yaml_config ∥ binary_envelope ∥ detection_npz）、
`tracking`（iou_tracker ∥ stable_streak）、`hardware`（serial_discovery ∥ modbus_gripper）、
`runtime`（subprocess_bridge ∥ ctypes_loader ∥ stdout_filter）——**均为平行独立模块**，
唯一关联来自各子包 `__init__.py` 的 `from .xxx import *` 聚合。

### 3.3 三条端到端协作流水线（跨子包）

#### 流水线 A：视觉感知 → 跨帧跟踪 → 落盘

```
vision.Intrinsics ─► vision.sample_depth / bbox_depth      （取目标距离）
        │
        ▼
  检测框列表 [{bbox, label, confidence}, ...]
        │
        ▼  tracking.IoUTracker.update()            （贪心 IoU 关联，分配 track_id）
  带 track_id 的结果
        ├─► tracking.pick_primary_track_id()      （面积最大者为主目标）
        ├─► tracking.StableStreak.update()        （连续 N 帧稳定判定 → 触发抓取）
        └─► io.pack_detections() → bytes ─► 传输 ─► io.unpack_detections()
```

关键衔接：`detection_npz` 缺 `mask` 时用 `geometry.bbox_to_mask` 补齐；
`iou_tracker` 与 `stable_streak` 之间**无直接依赖**，由调用方传递 `(track_id, bbox)`。

#### 流水线 B：地图 → 距离场 → 连续航向

```
gridmap.OccupancyGrid.from_ros_yaml()        （加载 ROS 地图，内部翻转行序）
        │
        ▼  region_labels()
    可通行掩码
        ▼  regions.label_connected()              （连通区域 + RegionInfo）
        ▼  distance_field.create_distance_field_for_region()
          （内部 merge_regions_by_dilation 经门洞焊接，需 cv2）
        ▼  descent.descent_direction(field.distances, robot_grid_xy, resolution=...)
    连续航向（退化时 None）
```

`occupancy.pixel_to_world` 与 `DistanceField.world_to_grid/grid_to_world` 使用**同一套
`origin_x/origin_y/resolution`**，因此"占据栅格 ↔ 距离场 ↔ 下降航向"之间可无歧义换算。
注意：**距离场有自己的分辨率**（`DistanceField(resolution=0.06)` 默认），与地图分辨率解耦（已有单测覆盖）。

#### 流水线 C：串口发现 → 夹爪驱动

```
hardware.serial_discovery.list_by_id_ports()      （列 /dev/serial/by-id）
        ▼  resolve_ports({"left": "", "right": ""}, serial_map=..., prefer=...)
     {left: /dev/..., right: /dev/...}             （解析失败只告警）
        ▼  probe_fn = 构造 ModbusGripper(port).read_status()
        ▼  ModbusGripper(port)  reset() → activate() → close(0.9)
                               → read_status() / is_object_held() / squeeze_more()
                               → close_port()
```

`probe_port` / `resolve_ports` 只要求 `probe_fn` 是 `Callable[[str], Any]`，
由调用方实现——**这正是 `serial_discovery` 与 `modbus_gripper` 解耦却可协作的衔接点**。

#### 流水线 D（运行时）：隔离外部依赖 + 压制噪声

```
父进程
  ├─ stdout_filter.install_filter(...) / quiet_loggers(...)      （与桥接解耦）
  └─ SubprocessBridge(argv).start() ─► 子进程 run_bridge_child(handler)
                                              └─ handler → 原生 API（ROS 发布者 / SDK）
       bridge.send(...)            # 链路断开返回 False，控制回路降级
       bridge.close(final_values=(0,0,0))   # 显式停止帧 + 哨兵，防看门狗锁存

（可选独立进程）ctypes_loader：load_library(dep, RTLD_GLOBAL) → load_library(shim)
                              → declare/CFunction → poll_until_ready
```

### 3.4 依赖速查表

| 想用 → | 先装/先学 |
| --- | --- |
| `tracking` 任意模块 | `geometry.boxes` |
| `io.detection_npz` | `geometry.boxes`、numpy |
| `io.yaml_config` | **PyYAML** |
| `io.binary_envelope` arrow 后端 | PyArrow；JPEG 辅助需 OpenCV |
| `gridmap.*` 区域合并 / 可视化 | OpenCV（`merge_regions_by_dilation`、`save_visualization`） |
| `vision.pointcloud.compute_normals` | SciPy |
| `vision.render_depth` / `fit_ellipse_to_points` | OpenCV |
| `motion.planar_ik.solve_numeric` | SciPy |
| `hardware.*`（任何导入） | **PyYAML（顶层导入）**；`ModbusGripper` 非 dry-run 还需 PySerial + minimalmodbus |
| `runtime.*` | 无（纯标准库） |

---

## 四、快速上手步骤与核心使用示例

### 4.1 安装与环境（二选一）

```bash
cd packages/reusable_model

# 方式一：安装（仅核心依赖 = numpy）
python -m pip install -e .

# 方式一加强版：含全部可选依赖 + 测试
python -m pip install -e ".[vision,hardware,yaml,test]"

# 方式二：不安装，直接用（把 src 加进 PYTHONPATH）
PYTHONPATH=src python -c "from reusable_model.geometry.boxes import iou; print(iou([0,0,10,10],[2,2,12,12]))"
```

已验证的运行环境：Python 3.12.3 + numpy 1.26.4（要求 ≥ 3.10）。

### 4.2 十秒冒烟测试

```python
from reusable_model.geometry.boxes import iou, overlap_score
from reusable_model.tracking.iou_tracker import IoUTracker

tr = IoUTracker(min_hits=1)
frame = tr.update([{"bbox": [0, 0, 10, 10], "label": "person"}])
print(frame[0]["track_id"], iou([0, 0, 10, 10], [2, 2, 12, 12]))
```

### 4.3 按子包的最小可跑示例

**geometry.boxes**

```python
from reusable_model.geometry import boxes

print(boxes.iou([0, 0, 10, 10], [5, 5, 15, 15]))              # ≈0.142857
print(boxes.iou_min([0, 0, 20, 20], [5, 5, 15, 15]))          # 1.0（小框被完整包含）
print(boxes.overlap_score([0, 0, 10, 10], [0, 0, 100, 100]))  # 1.0
print(boxes.clip_bbox([-5, 0, 30, 10], width=20, height=20))   # (0, 0, 20, 10)
mask = boxes.bbox_to_mask([1, 1, 3, 3], width=4, height=4)
print(mask.shape, mask.dtype, int(mask.sum()))                # (4, 4) bool 4
```

**geometry.rotations3d**

```python
import numpy as np
from reusable_model.geometry import rotations3d as r3

m = r3.euler_to_matrix([0.2, 0.3, 0.4], "ZYX")
print(np.allclose(r3.matrix_to_euler(m, "ZYX"), [0.2, 0.3, 0.4]))       # True

R = r3.rotation_from_two_axes("z", [0.0, 0.0, -1.0], "x", [1.0, 0.0, 0.0])
print(np.allclose(R[:, 2], [0.0, 0.0, -1.0]), r3.is_rotation_matrix(R))  # True True
print(r3.project_onto_plane([1.0, 2.0, 3.0], [0.0, 0.0, 1.0]))          # [1. 2. 0.]
```

**tracking：先跟踪、后判稳**

```python
from reusable_model.tracking import IoUTracker, StableStreak, pick_primary_track_id

tracker = IoUTracker(min_hits=1, iou_threshold=0.3)
streak = StableStreak(min_stable_frames=2, stable_iou=0.5)

for bbox in ([0, 0, 10, 10], [1, 1, 11, 11], [2, 2, 12, 12]):
    det = tracker.update([{"bbox": bbox, "label": "target", "confidence": 0.9}])[0]
    tid = det.get("track_id")
    stable = streak.update(tid, det["bbox"]) if tid is not None else streak.update(None, None)
    print(tid, stable)
# 1 False
# 1 True
# 1 True
```

**gridmap：栅格 → 距离场 → 航向（无需 OpenCV）**

```python
import numpy as np
from reusable_model.gridmap import occupancy, distance_field, descent

data = np.full((40, 40), occupancy.OCCUPIED, dtype=np.uint8)
data[5:35, 5:35] = occupancy.FREE
data[:, 20] = occupancy.OCCUPIED                      # 中间一堵墙
grid = occupancy.OccupancyGrid.from_array(data, origin=(0.0, 0.0), resolution=0.1)

mask = grid.region_labels() > 0
field = distance_field.DistanceField(resolution=0.1)
field.generate(mask, (1.0, 1.0), geometry=grid.geometry)

print("到目标距离:", round(field.distance_at_world(1.5, 1.5), 3))
px, py = field.world_to_grid(1.5, 1.5)
heading = descent.descent_direction(field.distances, (px, py), resolution=field.resolution)
print("下降航向(度):", None if heading is None else round(float(np.degrees(heading)), 1))
```

跨房间导航（经门洞合并，需 OpenCV）：

```python
from reusable_model.gridmap import regions, distance_field
labels, infos = regions.label_connected(mask, connectivity=8, min_area_cells=10)
field = distance_field.create_distance_field_for_region(
    labels, grid.geometry, [r.id for r in infos], (1.0, 1.0), resolution=0.1,
)
```

**motion**

```python
from reusable_model.motion import PlanarTwoLinkIK, PlanarTwoLink, BrakingProfile

ik = PlanarTwoLinkIK(PlanarTwoLink(0.25, 0.25, base_z=0.5))
res = ik.solve(0.10, 0.80, q2_limits=(-2.5, 0.0))
print(res.success, res.q1, res.q2, ik.verify(res, 0.10, 0.80))

profile = BrakingProfile()
print(profile.motion_strategy(current_velocity=0.4, max_velocity=0.6,
                              time_interval=0.02, distance=0.5))
print(BrakingProfile.braking_velocity(0.3, 0.8))       # sqrt(2·a·d)
```

**vision：单点深度 → 点云 → 障碍物 → 平面**

```python
import numpy as np
from reusable_model.vision import Intrinsics, PointCloudProcessor, sample_depth, transform_points, fit_plane_oriented

intr = Intrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
depth = np.full((480, 640), 2000, dtype=np.uint16)      # 2 米处的平地
print(sample_depth(depth, 320, 240))                    # 2.0

proc = PointCloudProcessor(intr, min_depth=0.1, max_depth=10.0)
points, pixels, colors = proc.from_rgbd(None, depth, sampling="grid")
world_points = transform_points(points, np.eye(4))       # 实际来自 TF / 正运动学
print(proc.to_obstacles(world_points).shape[1])          # 3
print(fit_plane_oriented(points, reference_axis=2, sign=-1.0) is not None)
```

**io：三件事各自独立**

```python
from reusable_model.io.yaml_config import load_yaml, dump_yaml, load_config, resolve_path, YamlStore
from reusable_model.io.binary_envelope import pack, unpack, pack_image
from reusable_model.io.detection_npz import pack_detections, unpack_detections

dump_yaml({"port": 8080, "host": "localhost"}, "cfg.yaml")
print(load_yaml("cfg.yaml"))

blob = pack({"kind": "frame"}, b"pixels", b"mask")
header, parts = unpack(blob)
print(header["kind"], parts == [b"pixels", b"mask"])     # frame True

data = pack_detections([{"mask": [[1]], "bbox": [0, 0, 1, 1], "confidence": 0.5}])
print(unpack_detections(data)[0]["confidence"])          # 0.5
```

**hardware：先 dry-run，再上硬件**

```python
from reusable_model.hardware import ModbusGripper, list_by_id_ports, resolve_ports

# 无硬件演练：不导入依赖、不打开串口
with ModbusGripper("/dev/null", dry_run=True) as gripper:
    gripper.reset()
    gripper.activate(timeout=0.05)
    gripper.close(0.9)
    status = gripper.read_status()
    if not gripper.is_object_held(status, target=0.9):
        status = gripper.squeeze_more(1.0)               # 只收紧，绝不放松
    print(gripper.status_text(status))
    print(gripper.dry_run_journal[-1])                   # 查看模拟命令日志

# 四层解析（失败只告警不抛出）
ports = resolve_ports({"left": "", "right": ""},
                      serial_map={"left": "DAANTRS9", "right": "DAANTMYY"},
                      prefer=("left", "right"))
```

**runtime：桥接 + 过滤**

```python
import sys
from reusable_model.runtime import SubprocessBridge, install_filter, quiet_loggers

child = [sys.executable, "-c", "import sys; list(sys.stdin)"]   # 子进程读空 stdin 后退出
bridge = SubprocessBridge(child, startup_timeout=0.5)
bridge.start()
print(bridge.send(0.1, 0.0, 0.25))          # True
bridge.close(final_values=(0.0, 0.0, 0.0))
print(bridge.connected)                     # False

wrappers = install_filter(["perf timing", "params:"], streams="stdout,stderr")
print(wrappers["stdout"] is not None)
print(quiet_loggers(["reusable_model.runtime"], level=40))
```

### 4.4 三层验证手段

```bash
cd packages/reusable_model

# 1) 全量单测（128 个用例）
python -m pytest

# 2) 单文件聚焦
python -m pytest tests/test_gridmap.py -q

# 3) 模块内嵌 doctest（每个模块的 docstring 示例都可执行）
python -m doctest src/reusable_model/geometry/boxes.py
```

在受限/只读环境中建议加 `PYTHONDONTWRITEBYTECODE=1` 与 `-p no:cacheprovider`，
避免写入 `__pycache__` 与 `.pytest_cache`：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider
```

---

## 五、学习顺序建议与重点难点标注

### 5.1 五阶段路线（每阶段都有明确的"检验点"）

| 阶段 | 目标 | 读什么 | 检验点（做出来才算学会） |
| --- | --- | --- | --- |
| **① 地基** | 建立公共词汇 | `geometry/geometry.README.md` → `boxes.py`（278 行，先读）→ `rotations3d.py` | 能说清 `iou` / `iou_min` / `overlap_score` 的区别与各自的适用场合；能独立完成一次"工具轴指向 + 欧拉角提取" |
| **② 最小闭环** | 看懂"依赖如何被消费" | `tracking/tracking.README.md` → `iou_tracker.py` → `stable_streak.py` | 跑通"3 帧后判稳"示例，并能解释 `min_hits` 与 `min_stable_frames` 各自 gate 什么 |
| **③ 导航主线** | 掌握最长的一条链 | `gridmap/gridmap.README.md` → `occupancy.py` → `regions.py` → `distance_field.py` → `descent.py` | 复现 4.3 中的"墙两侧到目标距离 + 航向"示例；解释 `UNREACHABLE = -1.0` 的设计动机 |
| **④ 感知线** | 打通图像→几何 | `vision/vision.README.md` → `pinhole.py` → `depth.py` → `pointcloud.py` → `planes.py` | 从一张深度图生成点云、滤波、再拟合地面平面 |
| **⑤ 执行与支撑** | 按需选学 | `motion/*`、`io/*`、`hardware/*`、`runtime/*` 各自的 README → 源码 | 用 `dry_run=True` 走完夹爪全套命令序列；用 `SubprocessBridge` 起一个子解释器 |

**为什么顺序是这样**：`geometry` 是唯一被跨子包依赖的模块（3 处 import 全部指向它）；
`tracking` 是消费 `geometry` 的最短路径，用来巩固第 ① 阶段；`gridmap` 内部是一条严格
单向链，必须按链路顺序读；`vision` 同样是单向链（`pinhole`/`depth` 是叶子）；其余四个
子包零跨子包依赖，可按个人工作重点自由排序。

### 5.2 重点难点标注

| 难度 | 模块 | 难点在哪 | 建议读法 |
| --- | --- | --- | --- |
| ⭐⭐⭐ | `gridmap/distance_field.py` | 1957 行，全包最大；两套 Dijkstra（堆版 vs numpy 波前版）、速度掩码制动斜坡、障碍时间戳记忆、局部重算 | 先只读 `generate` + `distance_at_world` + `UNREACHABLE` 语义；动态障碍与局部重算留到第二阶段 |
| ⭐⭐⭐ | `motion/planar_ik.py` | 肘部圆的参数化与姿态选择（`elbow_circle` → `circle_plane_intersections` → `select_elbow_on_circle`）需要空间想象；LM 求解器的阻尼自适应与线性搜索偏数值 | 先跑通解析 IK 的 4 行示例，再单独拿 `levenberg_marquardt` 做一个二维方程组求根 |
| ⭐⭐⭐ | `geometry/rotations3d.py` | 24 种欧拉序列 + 万向锁安全 + `select_euler_branches` 的双分支打分 | 先掌握 `euler_to_matrix`/`matrix_to_euler` 往返，再看 `wrap_to_limits`，最后读分支选择 |
| ⭐⭐ | `gridmap/occupancy.py` | 坐标约定是"导航 bug 头号来源"；`from_ros_yaml` 会翻转图像、忽略 yaw | 动手做一次 `flip_vertically()` 前后对比，把两种约定画下来 |
| ⭐⭐ | `vision/planes.py` + `pointcloud.py` | 依赖链交叉（两者都用 `depth` + `pinhole`）；`fit_plane_oriented` 的法向定向约定 | 先读 `vision.README.md` 的管线图，再逐函数对照 |
| ⭐⭐ | `runtime/ctypes_loader.py` | ctypes 签名、`RTLD_GLOBAL` 预加载顺序、`extern "C"` 垫片的意义 | 先跑 `load_library("m")` + `declare(libm.sqrt)` 的 4 行示例建立体感 |
| ⭐ | `tracking/*` | 概念简单，但要理解 `KeyedTrackerStore` 的锁边界（`IoUTracker` 自身不加锁） | 读 README 的表格即可 |
| ⭐ | `motion/speed_profile.py` | 344 行，纯标准库，公式直白 | 直接读源码 |
| ⭐ | `io/detection_npz.py` | 177 行，最短 | 直接读源码 |
| ⭐ | `runtime/stdout_filter.py` | 189 行 | 直接读源码 |

### 5.3 读 README 还是读源码？

**先读 README，再读源码。** 每个子包都带一份内容完整的 `*.README.md`，已把每个函数
的**签名 / 返回 / 异常契约**列成表格，并画出内部调用链。推荐节奏：

1. 读子包 README 的「模块总览」与「依赖关系」两节 → 建立地图；
2. 挑一个「快速上手」示例直接跑（`PYTHONPATH=src`）；
3. 带着"这个函数的异常契约为什么这样设计"的问题回到源码；
4. 用 `python -m doctest <该模块>` 验证你改动的理解没跑偏。

---

## 六、常见问题排查与最佳实践

### 6.1 排查速查表

| 现象 | 根因 | 定位方法 | 修复 |
| --- | --- | --- | --- |
| `import reusable_model.hardware` 报 `ModuleNotFoundError: yaml` | `serial_discovery.py` 在**模块顶层 `import yaml`**（非惰性），而 `hardware/__init__.py` 会星号导入它 | `python -c "import reusable_model.hardware"` 单独验证 | `pip install -e ".[yaml]"`（或 `pip install PyYAML`） |
| `import reusable_model.io` 报 `ModuleNotFoundError: yaml` | `yaml_config.py` 同样顶层 `import yaml`；而 `io/__init__.py` 星号导入它 | 同上，单独 `import reusable_model.io.yaml_config` | 装 PyYAML |
| `merge_regions_by_dilation` 抛 `RuntimeError`（提及 OpenCV） | 该功能与 `descent`/`save_visualization` 一样按需惰性导入 `cv2` | `python -c "import cv2"` | `pip install -e ".[vision]"` 或 `pip install opencv-python-headless` |
| `pack(..., backend="arrow")` 抛 `ImportError` | Arrow 后端需 PyArrow | `python -c "import pyarrow"` | `pip install -e ".[vision]"`（pyarrow 在该 extra 中） |
| `compute_normals` / `solve_numeric` 抛 `ImportError` | 需 SciPy（惰性导入） | `python -c "import scipy"` | `pip install scipy`（注意：SciPy **不在** `[vision]` extra 里） |
| `ModbusGripper(port)` 抛 `ImportError` | 非 dry-run 模式需 PySerial + minimalmodbus | 用 `dry_run=True` 先验证逻辑 | `pip install -e ".[hardware]"` |
| `list_ports()` 抛 `ImportError` | PySerial 惰性导入 | 用 `list_by_id_ports()`（纯 glob，不需 PySerial）替代 | 装 pyserial |
| 距离场查询返回 `-1.0`，误以为是"很近" | `-1.0` 是 `UNREACHABLE` 哨兵值，不是距离 | 检查 `value < 0`；对比 `DistanceField.UNREACHABLE` | 按不可达处理；检查目标点是否在区域内、掩码是否连通 |
| `descent_direction` 返回 `None`，被当成 0 弧度 | 采样圆盘内有效单元 < `_MIN_SAMPLES(3)` 或拟合退化 | 打印该点附近的距离场取值 | **必须把 `None` 当"没有可信航向"**，回退到其他策略或停止 |
| 地图上下颠倒 / 左右手不一致 | `OccupancyGrid` 内部约定（行 0 = y 最小）与图像/PGM 约定（行 0 = 顶部）不同 | 对比 `flip_vertically()` 前后的 `.data.tolist()` | 用 `from_ros_yaml()` 加载（内部已翻转），或用 `flip_vertically()` 显式转换 |
| `joints`/姿态角落在限位外 | 未做角度折叠 | 用 `wrap_to_limits(angle, reference, lower, upper)` | 见 `rotations3d` 的 6 号示例 |
| 距离场与地图分辨率不一致导致坐标偏差 | `DistanceField(resolution=0.06)` 默认值与地图分辨率**解耦** | 打印 `field.resolution` 与 `grid.geometry.resolution` | 显式传入相同或明确记录差异；换算统一走 `world_to_grid`/`grid_to_world` |
| `from_array` 重建的场调用 `recompute_local` 报错 | 该路径**不支持**局部重算（只支持只读查询） | 读 `from_array` 的 docstring | 改用完整 `generate()` 流程 |
| `SubprocessBridge.send()` 返回 `False` | 子进程已退出或管道不可写（设计上"链路断开即降级"，不抛异常） | `bridge.poll()`、`bridge.returncode`、`bridge.stderr_tail()` | 检查子进程是否崩溃；用 `stderr_tail()` 取最近 stderr 定位 |
| 左/右夹爪在重启后互换 | 使用了 `/dev/ttyUSB*` 这类会变号的路径 | 对比重启前后的 `ls /dev/serial/by-id/` | 改用 `/dev/serial/by-id/...`；用 `resolve_ports` 四层解析 |
| 打开串口失败 / `SerialException` | Linux 下访问串口通常需要 `dialout` 组权限或 udev 规则 | `ls -l /dev/ttyUSB*`；`groups` | 把用户加入 `dialout` 组，或配置 udev 规则 |
| 单测在只读环境报写缓存失败 | 需要写 `__pycache__` / `.pytest_cache` | — | `PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider` |

### 6.2 最佳实践

**使用层面**

1. **先 `dry_run`，再上硬件**：任何新的夹爪逻辑都先在 `ModbusGripper(..., dry_run=True)`
   下验证命令序列、位置打包与 `squeeze_more` 的收紧方向，确认无误再去掉 `dry_run`。
2. **绝不依赖"更宽的指令"恢复抓取**：`reset()`、`open()` 以及任何"更宽"的位置指令都会松手。
   出现 `MAJOR_FAULTS`（`{0xA..0xF}`：过流、欠压、自动释放等）时应停止、上报、交人处理。
3. **用稳定设备路径**：优先 `/dev/serial/by-id/...`，并在系统层面配备**硬件急停与机械限位**
   ——驱动本身不提供急停。
4. **把"无结果"当无结果**：`descent_direction` 的 `None`、`sample_depth` 的 `None`、
   `fit_plane` 的 `None`、`resolve_ports` 的告警——都是**有意的降级信号**，
   调用方必须显式分支处理，不要默认填 0。
5. **检查哨兵值**：距离场判有效一律用 `value < 0`，不要与 `inf`/`0` 混用。
6. **坐标约定显式化**：进入 `gridmap` 前先确认行序（必要时 `flip_vertically()`），
   并统一使用 `GridGeometry` 的 `origin_x/origin_y/resolution` 做换算。
7. **一个物理串口只给一个逻辑设备**：同一串口被两个进程打开会相互破坏 Modbus 事务。

**开发/扩展层面**

8. **改 `boxes.py` 前先想清楚影响面**：它是全包唯一被跨子包依赖的模块，
   `tracking`（2 处）与 `io`（1 处）会直接受影响。
9. **保持"惰性导入"约定**：新增可选重型依赖时，在使用处导入并抛带安装提示的
   `ImportError`，不要提升到模块顶层（`serial_discovery`/`yaml_config` 的顶层
   `import yaml` 是**已有例外**，新增代码不要效仿）。
10. **遵守异常契约**：类型错 `TypeError`、取值/形状错 `ValueError`、"无结果"返回 `None`
    或 `success=False`。这是全包一致的设计，破坏它会连带破坏上层调用方的分支逻辑。
11. **不要 `print()`**：统一 `logging.getLogger(__name__)`。
12. **新增模块要配齐三件套**：模块级 docstring（含可执行 doctest）+ `tests/test_<模块>.py`
   用例 + 子包 README 表格条目；随后 `python -m pytest` 与
    `python -m pytest --doctest-modules` 双双保持全绿。

**边界与风险声明**

13. **本包不替代实机测试**：所有算法均为与机器人解耦的构建块，**最终运行效果必须在真实
    环境中验证**。单测与 doctest 只能证明数学与契约正确，不能证明机械/电气安全。
14. **不含 ROS 集成层**：本包无 `package.xml`、无 launch、无话题/TF/QoS。若需要 QoS 策略、
    TF 发布频率、生命周期节点等 ROS 2 语义，请在 `projects/` 下的 ROS 工程中实现（规范见
    [`docs/adding-a-project.md`](./adding-a-project.md)）。

---

## 附录 A：模块 → README 索引

| 子包 | 子包说明文档 |
| --- | --- |
| 全包 | [`packages/reusable_model/README.md`](../packages/reusable_model/README.md) |
| `geometry` | [`src/reusable_model/geometry/geometry.README.md`](../packages/reusable_model/src/reusable_model/geometry/geometry.README.md) |
| `gridmap` | [`src/reusable_model/gridmap/gridmap.README.md`](../packages/reusable_model/src/reusable_model/gridmap/gridmap.README.md) |
| `motion` | [`src/reusable_model/motion/motion.README.md`](../packages/reusable_model/src/reusable_model/motion/motion.README.md) |
| `vision` | [`src/reusable_model/vision/vision.README.md`](../packages/reusable_model/src/reusable_model/vision/vision.README.md) |
| `io` | [`src/reusable_model/io/io.README.md`](../packages/reusable_model/src/reusable_model/io/io.README.md) |
| `tracking` | [`src/reusable_model/tracking/tracking.README.md`](../packages/reusable_model/src/reusable_model/tracking/tracking.README.md) |
| `hardware` | [`src/reusable_model/hardware/hardware.README.md`](../packages/reusable_model/src/reusable_model/hardware/hardware.README.md) |
| `runtime` | [`src/reusable_model/runtime/runtime.README.md`](../packages/reusable_model/src/reusable_model/runtime/runtime.README.md) |
| 测试 | [`tests/tests.README.md`](../packages/reusable_model/tests/tests.README.md) |

## 附录 B：一句话记住每个子包

| 子包 | 一句话 |
| --- | --- |
| `geometry` | 全包地基，二维框 + 三维旋转，被 `tracking`/`io` 依赖 |
| `vision` | 深度/彩色图像 → 三维点云与平面（标定→深度→点云→平面） |
| `tracking` | 检测框的时间连续性：IoU 关联 + 稳定门限 |
| `gridmap` | 栅格 → 区域 → 距离场 → 连续航向（一条严格单向的导航链） |
| `motion` | 位置↔关节角（IK）与 距离→速度（制动曲线），两条独立链路 |
| `hardware` | 先稳定发现串口，再驱动夹爪；一切先 `dry_run` |
| `io` | 配置、信封、检测批次三种落盘/传输格式 |
| `runtime` | 子进程隔离、原生库加载、输出流消音——零第三方依赖 |
