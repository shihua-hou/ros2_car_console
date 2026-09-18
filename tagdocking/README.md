# tagdocking — AprilTag 自动停靠框架

工业级 ROS2 Humble AprilTag 视觉自动停靠系统，支持多底盘（差速/全向/四足）、走停式视觉伺服、时间同步、里程计校验、完整状态机。停靠由外部服务直接触发（`/docking_node/start_docking`），预停靠导航由调用方负责把机器人送入 Tag 范围。

---

## 目录

1. [系统架构](#1-系统架构)
2. [快速开始](#2-快速开始)
3. [触发方式](#3-触发方式)
4. [状态机详解](#4-状态机详解)
5. [底盘适配器](#5-底盘适配器)
6. [参数配置](#6-参数配置)
7. [调试方法](#7-调试方法)
8. [仿真测试](#8-仿真测试)
9. [注意事项](#9-注意事项)

---

## 1. 系统架构

### 整体流程

```
                  开始停靠 (/docking_node/start_docking)
                             |
                             v
                      [SEARCH_TAG]
                    转-停扫描搜索 Tag
                             |
                             v
                        [ALIGN]
                     粗对中 (仅 yaw)
                             |
                             v
                       [APPROACH]
                   走停式视觉伺服逼近 Tag
                             |
                             v
                     [FINAL_SERVO]
               减速精确逼近 + 稳定确认
                             |
                             v
                        [DOCKED] ✓
```

> 停靠节点本身不负责把机器人导航到 Tag 附近——由外部服务（Nav2 / 业务节点 /
> 遥控）把机器人送到 Tag 视野内后，再调用 `/docking_node/start_docking` 进入
> 上面的视觉停靠流程。

### 模块结构

```
tagdocking/
├── action/Dock.action           # ROS2 Action 定义
├── config/docking.yaml          # 全部参数 (50+)
├── launch/docking.launch.py     # 启动文件
├── scripts/docking_node         # 入口脚本
├── tagdocking/
│   ├── docking_node.py          # 主节点 — 集成所有子系统
│   ├── state_machine.py         # 完整状态机 + 超时/异常处理
│   ├── visual_servo.py          # PID 视觉伺服控制器
│   ├── pid_controller.py        # PID (含 anti-windup)
│   ├── pose_buffer.py           # 时间戳位姿缓冲 (延迟过滤)
│   ├── motion_monitor.py        # 里程计运动校验 (堵转检测)
│   ├── utils.py                 # 角度/四元数工具函数
│   └── base_adapter/
│       ├── base_adapter.py      # 抽象基类: send_velocity(vx, vy, wz)
│       ├── diff_drive.py        # 差速轮 (unicycle 控制)
│       ├── omni.py              # 全向轮 (3-DOF 直接映射)
│       └── quadruped.py         # 四足 SDK (回调桥接)
└── CMakeLists.txt
```

### 数据流

```
/tag_detections ──► TF lookup (camera→tag) ──► EMA滤波 ──► PoseBuffer
                                                               │
                                                               ▼
                        ┌────────────────────── 状态机 ◄───────┘
                        │
   /odom ──► MotionMonitor ──► 堵转检测
                        │
                        ▼
               VisualServoController (PID)
                        │
                        ▼
                  BaseAdapter
                        │
               ┌───────┼───────┐
               ▼       ▼       ▼
          DiffDrive   Omni   Quadruped
               │       │       │
               ▼       ▼       ▼
            /cmd_vel /cmd_vel  SDK move()
```

---

## 2. 快速开始

### 2.1 前置条件

- ROS2 Humble 已安装
- `apriltag_ros` 已安装并能正常检测 Tag
- 相机已标定，发布 `/image_raw` 和 `/camera_info`
- Tag 贴在停靠目标上，且 TF 树连通（`camera_optical_frame → tag36h11:0`）
- 机器人已被外部服务（Nav2 / 业务节点 / 遥控）送到 Tag 视野范围内

### 2.2 编译

```bash
cd ~/ros2_ws
colcon build --packages-select tagdocking --symlink-install
source install/setup.bash
```

### 2.3 启动

```bash
# 终端 1: 启动停靠系统
ros2 launch tagdocking docking.launch.py \
    base_type:=diff_drive

# 终端 2: 先启动你的机器人底层驱动 (若未启动)
# ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

# 终端 3: 确保 apriltag_ros 有图像输入 (若 launch 中的 camera 话题不匹配)
# ros2 run tagdocking camera_info_bridge
```

> **注意**: launch 文件默认启动 `apriltag_node`，它会订阅 `image_rect` 话题（实际 remap 到 `image_topic` 参数指定的值，默认 `/image_raw`）。如果你的相机话题不是 `/image_raw`，需要通过 launch 参数指定。

> 停泊节点只做视觉停靠。若需要先把机器人导航到 Tag 附近，由外部 Nav2 / 业务
> 节点完成，到位后再触发 `/docking_node/start_docking`。

### 2.4 底盘类型选择

```bash
# 差速轮 (默认) — 只输出 linear.x + angular.z
ros2 launch tagdocking docking.launch.py base_type:=diff_drive

# 全向轮 / Mecanum — 输出 linear.x + linear.y + angular.z
ros2 launch tagdocking docking.launch.py base_type:=omni

# 四足机器人 — 通过 SDK move(vx, vy, yaw_rate) 控制
# 需要在 config/docking.yaml 中设置 base.type: "quadruped"
# 并在代码中注入 move_callback
```

### 2.5 自定义 Tag

```bash
ros2 launch tagdocking docking.launch.py \
    dock_tag_id:=5 \
    tag_size:=0.21 \
    family:=36h11 \
    camera_frame:=camera_color_optical_frame
```

---

## 3. 触发方式

系统提供 **三种** 触发方式：

### 方式一：Service (推荐，最简单)

```bash
# 启动停靠
ros2 service call /docking_node/start_docking std_srvs/srv/Trigger

# 取消停靠
ros2 service call /docking_node/cancel_docking std_srvs/srv/Trigger
```

**返回示例**:
```
success: True
message: "state=search_tag"
```

### 方式二：ROS2 Action (适合业务系统集成)

```bash
# 发送 Action Goal
ros2 action send_goal /docking_node/dock tagdocking/action/Dock "{dock_id: 'charger_0'}"

# 取消
ros2 action send_goal /docking_node/dock tagdocking/action/Dock "{dock_id: 'charger_0'}" --cancel
```

**Feedback 实时输出**:
```
distance_error: 0.342   # 距离误差 (m)
yaw_error: 0.087        # 朝向误差 (rad)
state: "approach"       # 当前状态
```

### 方式三：从 Python 代码调用

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class MyDockingClient(Node):
    def __init__(self):
        super().__init__('my_docking_client')
        self._start_cli = self.create_client(Trigger, '/docking_node/start_docking')
        self._undock_cli = self.create_client(Trigger, '/docking_node/start_undock')

    def start_docking(self):
        req = Trigger.Request()
        future = self._start_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        print(future.result().message)

    def undock(self):
        req = Trigger.Request()
        future = self._undock_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        print(future.result().message)
```

### 泊出 (Undock)

停泊完成后 (`DOCKED`)，调用 `~/start_undock` 把车退出停靠位：

```bash
ros2 service call /docking_node/start_undock std_srvs/srv/Trigger
```

泊出是一个固定的两段式盲动序列，纯里程计闭环、不看二维码：

1. **盲退** `undock.backup_distance` (默认 0.5 m，同重试倒车的 negative jog)
2. **原地转 180°** (`undock.turn_angle_deg`，正=CCW / 负=CW)

两段到位后进入 `UNDOCKED` 终态。`~/state` 依次发布 `undocking` → `undocked`，
webapp 据此弹"泊出完成"提示。泊出可从任意非停泊态触发 (IDLE / DOCKED / 错误终态)，
停泊进行中 (`SEARCH_TAG` / `APPROACH` …) 调用会被拒绝。

> 泊出是盲动 (后退 + 转向)，**调用方需确保车后方无障碍**。

---

## 4. 状态机详解

### 4.1 正常流程状态

| 状态 | 枚举值 | 说明 | 超时 |
|------|--------|------|------|
| `IDLE` | 0 | 空闲，等待启动命令 | — |
| `SEARCH_TAG` | 2 | 转-停交替扫描搜索 AprilTag | 60s |
| `ALIGN` | 3 | 原地旋转粗对中 (仅 yaw) | 15s |
| `APPROACH` | 4 | PID 视觉伺服逼近 Tag | 60s |
| `FINAL_SERVO` | 5 | 减速精确逼近 + 稳定确认 | 30s |
| `DOCKED` | 6 | 停靠成功 | — |
| `UNDOCKING` | 9 | 泊出: 盲退 + 原地转 180° | 30s (`undock.timeout_sec`) |
| `UNDOCKED` | 14 | 泊出成功 | — |

### 4.2 异常状态

| 状态 | 枚举值 | 触发条件 |
|------|--------|----------|
| `TAG_LOST` | 10 | 视觉跟踪状态下 Tag 丢失超过 1 秒 |
| `TIMEOUT` | 11 | 全局超时 (120s) 或各阶段超时 |
| `MOTION_FAILED` | 12 | 发送速度指令但里程计无位移超过 2 秒 |
| `CANCELLED` | 13 | 用户主动取消 |

### 4.3 各状态行为详解

#### SEARCH_TAG — 转-停扫描

```
rotate 0.8s → pause 1.5s → rotate 0.8s → pause 1.5s → ...
```

- **rotate 阶段**: 以 `search.angular_speed` (0.3 rad/s) 原地旋转
- **pause 阶段**: 完全静止，给 apriltag_ros 清晰图像用于检测
- **Tag 锁定**: Tag 在 pause 期间持续可见 `search.hold_time_sec` (0.5s) 后锁定
- **搜索方向**: `search.search_direction` (+1=CCW 逆时针, -1=CW 顺时针)

#### ALIGN — 粗对中

- 仅做 yaw 轴 P 控制：`wz = kp_yaw * error_yaw`
- 连续 5 帧 (250ms) yaw 误差 < `tolerance.yaw_deg` (3°) 后进入 APPROACH
- 作用：把 Tag 放到机器人正前方，避免 APPROACH 阶段机头朝侧方

#### APPROACH — PID 视觉伺服

- 三轴独立 PID：`vx = PID_x(error_x)`, `vy = PID_y(error_y)`, `wz = PID_yaw(error_yaw)`
- 差速轮：`vy` 被忽略，通过 unicycle 控制器 (atan2 分解 + 转向补偿) 间接消除横向误差
- 全向轮：三轴同时控制
- 到达 `final_servo.distance` (0.2m) 范围内后转入 FINAL_SERVO

#### FINAL_SERVO — 减速精确定位

**降速策略**：
```
正常速度:  vx ≤ 0.3 m/s,  wz ≤ 0.8 rad/s
Final:     vx ≤ 0.05 m/s,  wz ≤ 0.2 rad/s
```

**成功判定 (三个条件同时满足，持续 1 秒)**：
1. 位置误差 `< 3cm` (`tolerance.position_m`)
2. 朝向误差 `< 3°` (`tolerance.yaw_deg`)
3. 机器人速度 `< 0.01` (稳定静止)

**安全保护**：距离 < `safety.minimum_distance_m` (0.15m) 时强制判定 DOCKED，防止碰撞。

### 4.4 状态发布

```bash
# 监听状态变化
ros2 topic echo /docking_node/state
# 输出: data: "approach"
```

---

## 5. 底盘适配器

### 5.1 统一接口

```python
class BaseAdapter(ABC):
    def send_velocity(self, vx: float, vy: float, yaw_rate: float): ...
    def stop(self): ...
```

所有底盘类型通过同一接口控制，状态机和视觉伺服无需知道底盘差异。

### 5.2 DiffDriveAdapter (差速轮)

**控制律** (unicycle):
```
distance    = sqrt(error_x² + error_y²)
target_angle = atan2(error_y, error_x)
vx = K_distance * distance           # 前向速度
wz = K_angle * target_angle + K_yaw * error_yaw   # 角速度
```

- **vy 恒为 0** — 差速轮无侧移能力
- 通过 `atan2` 将横向误差转为朝向角修正
- 输出: `Twist.linear.x` + `Twist.angular.z`

**参数**: `pid.kp_x` (K_distance), `pid.kp_yaw` (用作 K_angle 和 K_yaw)

### 5.3 OmniAdapter (全向轮 / Mecanum)

**控制律** (直接映射):
```
vx = Kx * error_x
vy = Ky * error_y
wz = Kyaw * error_yaw
```

- 三轴独立控制
- 输出: `Twist.linear.x` + `Twist.linear.y` + `Twist.angular.z`

**参数**: `pid.kp_x`, `pid.kp_y`, `pid.kp_yaw`

### 5.4 QuadrupedAdapter (四足机器人)

**控制律** (同 Omni):
```
vx = Kx * error_x, vy = Ky * error_y, wz = Kyaw * error_yaw
```

- **不发布 cmd_vel**，改为调用 SDK 回调
- 构造函数接收 `move_callback(vx, vy, yaw_rate)`

**集成示例**:
```python
from tagdocking.base_adapter import QuadrupedAdapter
from unitree_sdk import ChannelFactory  # 示例 (实际 SDK 接口不同)

robot = YourRobotSDK()
adapter = QuadrupedAdapter(
    move_callback=robot.move,   # SDK 的移动接口
    node=ros_node,
    k_x=0.8, k_y=0.8, k_yaw=1.5,
    max_linear_speed=0.5,
    max_y_speed=0.3,
    max_yaw_speed=0.8,
)
```

---

## 6. 参数配置

全部参数在 `config/docking.yaml` 中，运行时可被 launch 参数覆盖。

### 6.1 Tag 参数

```yaml
tag.family: "36h11"                   # AprilTag 家族
tag.size: 0.16                        # Tag 边长 (m)
tag.id: 0                             # 要停靠的 Tag ID
tag.frame: "tag36h11:0"               # TF 中的 Tag 坐标系名
tag.fresh_timeout_sec: 1.0            # 超过 N 秒无检测 → 视为丢失
tag.ema_alpha: 0.5                    # EMA 平滑 (0=重滤波, 1=原始值)
tag.max_pose_jump_m: 0.3              # 位姿跳变阈值 (防误识别抖动)
```

### 6.2 停靠目标

```yaml
dock_target.distance: 0.30            # 最终距 Tag 多远停下 (m)
dock_target.lateral_offset: 0.0       # 横向偏移 (m), 0=居中
dock_target.yaw_offset_deg: 0.0       # 朝向偏移 (°), 0=正对 Tag
```

**场景示例**:

| 场景 | distance | lateral_offset | yaw_offset_deg | 说明 |
|------|----------|---------------|----------------|------|
| 正对接充电桩 | 0.30 | 0.0 | 0.0 | 停在 Tag 正前方 30cm |
| 横向靠边卸货 | 0.50 | 0.3 | 0.0 | 停在 Tag 前方 50cm 偏左 30cm |
| 90° 侧向停靠 | 0.40 | 0.0 | 90.0 | 停在 Tag 正前方 40cm，但侧向对齐 |

### 6.3 PID 参数

```yaml
pid.kp_x: 0.8                         # X 轴比例增益
pid.ki_x: 0.0                         # X 轴积分 (通常不需要)
pid.kd_x: 0.0                         # X 轴微分
pid.kp_yaw: 1.5                       # Yaw 轴比例增益 (通常比 X 轴高)
pid.integral_limit: 0.5               # 积分抗饱和上限
```

**调参建议**:
- 先调 `kp_x`：从小到大，直到出现震荡后回退 30%
- 再调 `kp_yaw`：同上，yaw 通常比位移更灵敏
- `ki` 通常保持 0，除非有系统性稳态误差（如地面倾斜）
- 差速轮 `kp_yaw` 同时影响 K_angle 和 K_yaw

### 6.4 安全参数

```yaml
safety.minimum_distance_m: 0.15       # 最小安全距离 (m) — 防止撞 Tag
timeout_sec: 120.0                    # 全局超时 (s)
motion_monitor.enable: true           # 启用运动监控
motion_monitor.timeout_sec: 2.0       # 堵转判定时间 (s)
motion_monitor.min_motion_m: 0.01     # 堵转判定最小位移 (m)
```

**堵转检测逻辑**:
```
IF 下发速度 ≠ 0
AND 里程计位移 < 0.01m
AND 持续时间 > 2.0s
THEN → MOTION_FAILED
```

---

## 7. 调试方法

### 7.1 监控状态

```bash
# 实时状态 (1 Hz 日志)
ros2 run tagdocking docking_node

# 状态话题
ros2 topic echo /docking_node/state

# 误差话题
ros2 topic echo /docking_node/error
# x=距离误差, y=横向误差, z=yaw误差
```

### 7.2 绘图观察误差收敛

```bash
# 安装 rqt_plot (若未安装)
sudo apt install ros-humble-rqt-plot

# 绘制距离误差和 yaw 误差
ros2 run rqt_plot rqt_plot \
    /docking_node/error/x \
    /docking_node/error/z
```

### 7.3 查看 TF 树

```bash
# 检查 TF 连通性
ros2 run tf2_tools view_frames.py
# 查看 frames.pdf

# 实时查看 tag→base_link 的变换
ros2 run tf2_ros tf2_echo base_link tag36h11:0
```

### 7.4 检查 Tag 检测

```bash
# 确认 apriltag_ros 在发布检测结果
ros2 topic echo /detections --once

# 确认 TF 广播 (应该看到多帧 tag36h11:X)
ros2 topic echo /tf | grep tag36h11
```

### 7.5 RViz 可视化

1. 添加 **TF** 显示：确认 `camera_optical_frame → tag36h11:0` 连线
2. 添加 **Image** 显示：查看相机画面，确认 Tag 在视野内
3. 添加 **Odometry** 显示：确认里程计箭头随机器人运动

### 7.6 常见问题排查

| 问题 | 原因 | 解决 |
|------|------|------|
| 状态机停在 SEARCH_TAG 不动 | Tag 未检测到 | 检查相机话题、Tag ID、TF 连通性 |
| `TF lookup failed` | 坐标系不连通 | 检查 `camera_frame` 参数是否与实际一致 |
| 机器人原地摇摆不前进 | `kp_yaw` 太大 / `kp_x` 太小 | 降低 `kp_yaw` 至 1.0，提高 `kp_x` 至 1.0 |
| 机器人冲过 Tag | 速度太快 / 没进入 FINAL_SERVO | 降低 `limits.max_linear_speed`，增大 `final_servo.distance` |
| 对接精度不够 | PID 增益偏低 / Tag 图像质量差 | 提高 `kp_x` 和 `kp_yaw`；改善光照和相机对焦 |
| `MOTION_FAILED` | 底盘未响应 `/cmd_vel` | 检查 cmd_vel 话题名是否匹配，底盘驱动是否正常 |

---

## 8. 仿真测试

### 8.1 Gazebo + 静态 Tag

```bash
# 1. 启动 Gazebo 仿真世界 (含机器人和 Tag 模型)
ros2 launch my_sim world.launch.py

# 2. 启动停靠
ros2 launch tagdocking docking.launch.py

# 3. 将机器人手动放到距 Tag ~2m 处
#    (在 Gazebo 中拖动机器人模型)

# 4. 触发停靠
ros2 service call /docking_node/start_docking std_srvs/srv/Trigger

# 5. 观察机器人自动驶向 Tag 并停下
```

### 8.2 无 GPU 仿真 (纯 mock)

如果不想启动完整仿真，可以用 Python 脚本手动发布假检测：

```python
# mock_detection.py — 发布假的 Tag 检测和 TF
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection

class MockDetector(Node):
    def __init__(self):
        super().__init__('mock_detector')
        self._pub = self.create_publisher(AprilTagDetectionArray, '/detections', 10)
        self._timer = self.create_timer(0.1, self._publish)  # 10Hz

    def _publish(self):
        msg = AprilTagDetectionArray()
        det = AprilTagDetection()
        det.id = 0
        det.family = "36h11"
        msg.detections = [det]
        self._pub.publish(msg)

rclpy.init()
rclpy.spin(MockDetector())
```

---

## 9. 注意事项

### 9.1 坐标系约定 (REP-103)

本系统严格遵循 REP-103 坐标系：

```
       x 前向
         ^
         |
         |
         +------> y 左向
```

- **error_x > 0**: 机器人需要往前走 (Tag 在正前方)
- **error_y > 0**: Tag 偏向机器人左边，需要左移 (全向轮) 或左转 (差速轮)
- **error_yaw > 0**: 机器人需要逆时针 (CCW) 旋转来正对 Tag

### 9.2 TF 依赖

系统通过 **tf2** 查询 Tag 位姿，不直接使用 apriltag_ros 的检测坐标。必须满足：

```
map → odom → base_footprint → base_link → camera_optical_frame → tag36h11:0
```

- `apriltag_ros` 的 `publish_tf` 参数必须设为 `true`
- `camera_frame` 参数必须与实际的相机光学坐标系名一致
- 如果设置了 `measure_frame` (如 `base_footprint`)，则直接从该坐标系查询 Tag 变换

### 9.3 视觉延迟

- 系统默认丢弃超过 **200ms** 的检测数据 (`camera.max_latency_ms`)
- 对低帧率 (2~3Hz) 相机，将 `tag.fresh_timeout_sec` 调到 1.5s
- PoseBuffer 保留最近 30 帧 (`pose_buffer.size`)，Controller 始终取最新有效帧

### 9.4 差速轮无法消横向误差

差速轮 **没有侧移能力 (vy=0)**。系统通过 unicycle 控制器将横向误差转为朝向修正：

```
离 Tag 越近时，theta 容差越紧，避免末段不打横
```

如果差速轮精停效果不好，可以：
1. 增加 `pid.kp_yaw` 让转向更灵敏
2. 在 `dock_target` 中设置 `lateral_offset: 0.0` (要求居中停靠)
3. FINAL_SERVO 状态会做精细的 yaw 校正

### 9.5 信号安全

系统注册了 **SIGINT 和 SIGTERM** 信号处理器：
- 收到 Ctrl+C 时，阻塞式发布 **1 秒零速指令** (100Hz × 1s) 覆盖底盘看门狗
- ROS context 关闭时也会触发 `_safe_stop()`
- **不会出现 Ctrl+C 后机器人还继续前进的情况**

### 9.6 性能要求

- 控制循环: **20 Hz** (50ms 周期)
- 推荐相机帧率: **≥ 10 Hz** (apriltag_ros 在 CPU 上约 15~30 Hz)
- 视觉延迟: **< 200ms** 以获得稳定控制效果
- PoseBuffer 容量: 30 帧 (覆盖 1.5s @ 20Hz 或 3s @ 10Hz)

### 9.7 构建说明

- 使用 **ament_cmake** 构建（非 ament_python），因为包含自定义 Action 接口
- `Dock.action` 由 `rosidl_generate_interfaces` 编译生成 Python 模块
- Python 代码通过 CMake 的 `install(DIRECTORY ...)` 安装到 `dist-packages`
- 入口脚本 (`docking_node`) 通过 CMake 的 `install(PROGRAMS ...)` 安装到 `lib/tagdocking/`

---

## 依赖

```
rclpy, std_msgs, std_srvs, geometry_msgs, nav_msgs
action_msgs, apriltag_msgs
tf2_ros, tf2_geometry_msgs
apriltag_ros (运行时)
```

## License

Apache-2.0
