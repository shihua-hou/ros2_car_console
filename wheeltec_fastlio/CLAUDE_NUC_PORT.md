# CLAUDE.md — MID360s + FAST-LIO2 建图导航项目移植指南（NUC 版）

> **本文档的使用方式**：把本文件放到 NUC 的 ROS2 工作空间根目录（如 `~/wheeltec_ros2/src/CLAUDE.md`），
> 在该目录启动 Claude Code。你（Claude）的任务是按本文档把源机小车上已完成的
> MID360s 3D 雷达建图导航系统完整复现到本机（NUC）。
> 本文档由源机上完成原始开发的 Claude 编写，包含全部实测踩坑记录——**遇到问题先查第 6 节陷阱手册**。

---

## 1. 任务目标与工作方式

### 最终验收标准（按顺序全部通过）

1. `/livox/lidar` 点云 ~10Hz，`/livox/imu` ~200Hz
2. FAST-LIO 建图：`/Odometry` ~10Hz，无 "No Effective Points" 刷屏；建图中每约 10 秒落盘一个 `scans_N.pcd`
3. `save_map` 生成 2D 地图，日志显示地面拟合倾角 <3°、雷达离地高度合理
4. 导航启动后自动重定位 `score > 0.9`
5. RViz 下发 Nav2 Goal 小车顺利到点，行驶无扭摆
6. 抱起小车搬到别处，10 秒内自动重新定位

### 工作方式（重要）

- **每个 Phase 验证通过再进入下一个**，不要跳步。
- 测试用的后台 launch **用完立即停干净**（见陷阱手册"双实例灾难"）。
- 杀 ROS 进程只用 Ctrl+C/SIGINT；被迫强杀后必须清理 `/dev/shm`（第 6 节）。
- 建议把本机踩到的新坑追加记录到你自己的 memory。

---

## 2. 源机信息（所有源码从这里拷贝）

| 项 | 值 |
|----|-----|
| 源机 | wheeltec 小车，RDK X5 主控（aarch64，与本机不同架构，**所有编译产物不可复用，只拷源码**） |
| SSH | 用户 `wheeltec`，密码 `CHANGE_ME`，IP 在源机上 `hostname -I` 查（WiFi 动态分配） |
| 工作空间 | `/home/wheeltec/wheeltec_ros2/`（src 约 2GB，含 39 个包目录） |

### 需要拷贝的内容

```bash
# 在 NUC 上执行 (先 mkdir -p ~/wheeltec_ros2/src)。SRC_IP 替换为源机实际 IP
rsync -av --progress \
  --exclude 'build' --exclude 'install' --exclude 'log' \
  --exclude 'FAST_LIO/PCD/*.pcd' --exclude '__pycache__' --exclude '.git' \
  wheeltec@SRC_IP:/home/wheeltec/wheeltec_ros2/src/ ~/wheeltec_ros2/src/

rsync -av --exclude 'build' wheeltec@SRC_IP:/home/wheeltec/Livox-SDK2 ~/
scp wheeltec@SRC_IP:/home/wheeltec/stop_nav.sh ~/ && chmod +x ~/stop_nav.sh
```

2GB 走 WiFi 需要一段时间；rsync 支持断点续传，中断了重跑同一命令即可。

---

## 3. 系统架构速览

```
建图链路:  MID360s --CustomMsg(xfer_format=1)--> FAST-LIO2 --> 3D点云(PCD分段)
                                                     |
                                             Ctrl+C 退出后
                                                     v
           pcd2pgm: RANSAC地面校平 --> 高度带投影 --> WHEELTEC3D.pgm/yaml

导航链路:  MID360s --PointCloud2(xfer_format=0)--> pointcloud_to_laserscan --> /scan
           auto_relocalize: 启动自动重定位 + 绑架检测看门狗 --> /initialpose
           TF: map --(AMCL)--> odom_combined --(EKF 轮式+IMU)--> base_footprint
           Nav2(NavFn规划 + MPPI控制) 复用 wheeltec_nav2 框架
```

- FAST-LIO 的 TF（camera_init→body）与底盘 TF 是**两棵独立的树**，互不冲突；建图时 RViz Fixed Frame 用 `camera_init`，导航时用 `map`。
- 建图与导航**不能同时运行**（驱动数据格式不同）。

### 关键包职责

| 包 | 职责 |
|----|------|
| `wheeltec_fastlio` | **核心集成包**（原创）：pcd2pgm（PCD→2D地图，含地面校平）、auto_relocalize（扫描匹配重定位+看门狗）、lidar_test/mapping/save_map/navigation 四个 launch |
| `livox_ros_driver2` | MID360s 驱动，**已打时间戳补丁** |
| `FAST_LIO` | hku-mars 官方 ROS2 分支，**已打 PCD 保存补丁** |
| `turn_on_wheeltec_robot` | 底盘串口 + EKF（odom_combined→base_footprint） |
| `wheeltec_robot_nav2`（包名 wheeltec_nav2） | nav2 launch/param/map，**param 已做 5 项修复** |
| `navigation2-humble` | nav2 源码（工作空间自带，随 colcon 一起编译） |

---

## 4. 分阶段移植步骤

### Phase 0：环境侦查

```bash
uname -m                 # 应为 x86_64
ls /opt/ros/             # 应有 humble
ip link show             # 记下网卡名: NUC 通常是 enpXsY/enoX, 不是 eth0!
whoami                   # 记下用户名
```

**若 NUC 用户名不是 `wheeltec`**：拷贝完源码后必须修正硬编码路径：

```bash
grep -rln "/home/wheeltec" ~/wheeltec_ros2/src/wheeltec_fastlio/ ~/stop_nav.sh
# 逐个 sed 替换为实际 home 路径。涉及: mapping.launch.py 的 PCD_DIR、
# save_map.launch.py 的 pcd_dir/src_map_dir、pcd2pgm.cpp 默认参数、navigation.launch.py
```

FAST_LIO 的 PCD 保存路径是编译期宏（ROOT_DIR=源码目录），在 NUC 上重新编译后自动正确，无需修改。

### Phase 1：拷贝源码并校验补丁

执行第 2 节的 rsync，然后**校验关键补丁存在**（防止拷到未打补丁的版本）：

```bash
grep -c "WHEELTEC patch" ~/wheeltec_ros2/src/livox_ros_driver2/src/comm/pub_handler.cpp   # 必须=2
grep -c "WHEELTEC patch" ~/wheeltec_ros2/src/FAST_LIO/src/laserMapping.cpp                 # 必须=1
ls ~/wheeltec_ros2/src/FAST_LIO/include/ikd-Tree/ikd_Tree.cpp   # 子模块必须存在
grep -c "behavior_server:" ~/wheeltec_ros2/src/wheeltec_robot_nav2/param/wheeltec_params/param_S100_diff.yaml  # ≥1
mkdir -p ~/wheeltec_ros2/src/FAST_LIO/PCD    # PCD输出目录（rsync排除了pcd文件,目录可能没建）
```

任何一项不符 → 见第 5 节手工补齐。

### Phase 2：系统依赖

```bash
sudo apt update
sudo apt install -y ros-humble-pointcloud-to-laserscan ros-humble-robot-localization \
  libpcl-dev ros-humble-pcl-ros ros-humble-pcl-conversions libeigen3-dev \
  ros-humble-joint-state-publisher ros-humble-robot-state-publisher python3-colcon-common-extensions
# 兜底: cd ~/wheeltec_ros2 && rosdep install --from-paths src --ignore-src -r -y
#（rosdep 可能报个别 wheeltec 自定义包 key 找不到, 可忽略）

# Livox-SDK2 编译安装（MID360s 必需 SDK2）
cd ~/Livox-SDK2 && rm -rf build && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) && sudo make install && sudo ldconfig

# 底盘串口 udev 规则（生成 /dev/wheeltec_controller）
cd ~/wheeltec_ros2/src/turn_on_wheeltec_robot && sudo bash wheeltec_udev.sh
# 之后重新插拔底盘 USB 线, ls /dev/wheeltec_controller 验证
```

### Phase 3：编译

**⚠️ 绝对禁止运行 `livox_ros_driver2/build.sh`——它会 `rm -rf` 整个工作空间的 build/install！**

livox_ros_driver2 需要先做一次性文件准备（rsync 拷来的应该已就绪，检查一下）：

```bash
cd ~/wheeltec_ros2/src/livox_ros_driver2
ls package.xml launch/ 2>/dev/null || { cp package_ROS2.xml package.xml; cp -r launch_ROS2 launch; }
grep -q "livox_ros_driver2" package.xml && echo OK
```

编译（x86 上比源机快很多，全量预计 20-40 分钟）：

```bash
source /opt/ros/humble/setup.bash
cd ~/wheeltec_ros2
# 先编基础依赖包
colcon build --packages-select livox_ros_driver2 --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
source install/setup.bash
colcon build --packages-select fast_lio
# 再全量（含 navigation2-humble、wheeltec 全家桶、wheeltec_fastlio）
colcon build 2>&1 | tail -20
```

个别 wheeltec 附属包（语音/摄像头等）编译失败不影响本项目，用 `--packages-skip` 跳过即可；
必须成功的包：`livox_ros_driver2, fast_lio, wheeltec_fastlio, wheeltec_nav2, turn_on_wheeltec_robot, nav2_*`。

### Phase 4：雷达网络

```bash
# 1. 找出接雷达的网口(插着网线的那个): ip link show 看哪个口有 CARRIER
# 2. 配静态IP (IFACE 替换为实际网卡名):
sudo nmcli con add type ethernet ifname IFACE con-name livox-mid360 \
  ipv4.method manual ipv4.addresses 192.168.1.5/24 ipv4.never-default yes connection.autoconnect yes
sudo nmcli con mod livox-mid360 connection.autoconnect-priority 100
sudo nmcli con up livox-mid360

# 3. 找雷达IP: 出厂 IP = 192.168.1.1XX, XX=本台雷达SN码后两位(和源机那台不同!)
#    看雷达底部标签的SN, 或探测:
for i in $(seq 100 199); do ping -c1 -W0.3 192.168.1.$i &>/dev/null && echo "雷达IP: 192.168.1.$i"; done

# 4. 把实际雷达IP写进配置(lidar_configs.ip 字段), 然后重编译:
vim ~/wheeltec_ros2/src/wheeltec_fastlio/config/MID360s_config.json
cd ~/wheeltec_ros2 && colcon build --packages-select wheeltec_fastlio
```

**验证**：

```bash
source ~/wheeltec_ros2/install/setup.bash
ros2 launch wheeltec_fastlio lidar_test.launch.py &
ros2 topic hz /livox/lidar    # ~10Hz
ros2 topic hz /livox/imu      # ~200Hz
# 验完 Ctrl+C/kill 干净!
```

### Phase 5：本车标定（必做，数值不能照抄源机！）

1. **车型**：`grep car_mode ~/wheeltec_ros2/src/turn_on_wheeltec_robot/config/wheeltec_param.yaml`
   - 若 car_mode ≠ `S100_diff`：把 param_S100_diff.yaml 里的 5 项修复移植到对应的
     `param_<car_mode>.yaml`（清单见第 5 节第 3 条）。
2. **量取（卷尺）**：
   - 驱动轮轴心 → 雷达中心的水平距离 = `lidar_x`（雷达在轴心**前**为正、**后**为负——务必确认前后方向，源机第一次就搞反了）
   - 车长 L、车宽 W、轴心到车头距离 F → footprint = `[[F-L, ±W/2], [F, ±W/2]]`
3. **改入配置**：
   - `lidar_x`/`lidar_z` 默认值：`wheeltec_fastlio/launch/mapping.launch.py` 和 `navigation.launch.py`（两处必须一致）。lidar_z 先填估计值，建完第一张图后用 pcd2pgm 日志里的"雷达离地高度约 X.XXm"精确回填。
   - footprint：`param_<car_mode>.yaml` 里 local/global costmap 两处。
4. 改完 `colcon build --packages-select wheeltec_fastlio wheeltec_nav2`。

### Phase 6：功能验证（按验收标准逐项）

```bash
# --- 建图 ---
ros2 launch wheeltec_fastlio mapping.launch.py
# 另开终端: ros2 topic hz /Odometry (~10Hz); ls src/FAST_LIO/PCD/ 每~10秒多一个分段
# 遥控(ros2 run wheeltec_robot_keyboard wheeltec_keyboard)慢速走一圈, Ctrl+C 退出

# --- 转图 ---
ros2 launch wheeltec_fastlio save_map.launch.py
# 检查日志: "地面校平完成: 拟合倾角 X.X°, 雷达离地高度约 X.XXm" → 回填 lidar_z
# 地图存至 wheeltec_nav2/map/WHEELTEC3D.{pgm,yaml} (install+src两份)

# --- 导航 ---
ros2 launch wheeltec_fastlio navigation.launch.py
# 预期日志: "Managed nodes are active" ×2 → "重定位成功: ... score=0.9x"
# 每30秒打印 "定位健康度(严格分): 0.6~0.8" 为正常
# RViz(远程PC)下发 Nav2 Goal 验证到点; 最后做搬车绑架测试(等5~10秒自动恢复)
```

RViz 配置要点：`/map` 显示必须 Reliability=**Reliable** + Durability=**Transient Local**；
`/scan` 显示必须 Reliability=**Best Effort**。设反了就是"收不到数据"。

---

## 5. 关键补丁与修复清单（校验/手工补齐用）

### 5.1 livox_ros_driver2 时间戳补丁（2 处，`src/comm/pub_handler.cpp`）

**症状（不打的后果）**：FAST-LIO 随机报 `No Effective Points!` / `IMU and LiDAR not Synced`。
**根因**：MID360s 的点云包与 IMU 包 time_type 不一致 → 原版驱动点云用雷达内部时钟、IMU 用主机时钟，双时钟漂移。
**补丁内容**：① `OnLivoxLidarPointCloudCallback` 里无条件 `is_timestamp_sync_.store(false);`（替换原来按 time_type 判断的 if/else）；② `GetEthPacketTimestamp` 里无视 time_type 一律 `return std::chrono::high_resolution_clock::now().time_since_epoch().count();`。两处都有 `WHEELTEC patch` 注释标记。

### 5.2 FAST_LIO PCD 保存补丁（`src/laserMapping.cpp`）

**症状**：建图退出后 PCD 目录为空。
**根因**：ROS2 移植版把 `publish_frame_world` 里整个 `if (pcd_save_en){...}` 累积块用 `/* */` 注释掉了。
**补丁**：取消该块注释（搜 `WHEELTEC patch` 标记）。配套配置：`wheeltec_fastlio/config/wheeltec_mid360.yaml` 中 `pcd_save.interval: 100`（分段保存防 OOM，**不要用 -1**）。

### 5.3 nav2 参数 5 项修复（`param_<car_mode>.yaml`）

1. **behavior_server 段**（原厂整段被注释 → spin/backup 报 "target frame odom does not exist"）：
   ```yaml
   behavior_server:
     ros__parameters:
       local_frame: odom_combined
       global_frame: map
       robot_base_frame: base_footprint
       transform_tolerance: 0.2
   ```
2. **规划器换 NavFn**（原厂给差速车配了阿克曼用的 SmacPlannerHybrid+DUBIN → 频繁 no valid path / Starting point in lethal space）：
   ```yaml
   GridBased:
     plugin: "nav2_navfn_planner/NavfnPlanner"
     tolerance: 0.3
     use_astar: false
     allow_unknown: false
   ```
3. **MPPI 防扭摆**：`wz_std: 0.2`（原0.4）、`wz_max: 1.2`（原2.0）、`temperature: 0.2`（原0.3）。
4. **MPPI ObstaclesCritic**：`cost_scaling_factor` 和 `inflation_radius` 必须与 local_costmap 的 inflation_layer **完全一致**（humble 版换算依赖）；车身非圆形/有悬伸时 `consider_footprint: true`。
5. **footprint 实测尺寸**（本车量取，两处 costmap 都要改）。

另外 NUC 算力充裕，MPPI `batch_size` 可用 2000（源机 RDK X5 只能跑 1000）。

### 5.4 wheeltec_fastlio 包内已含的机制（了解即可，随包拷贝）

- pcd2pgm：RANSAC 地面校平（消除安装倾角，1° 倾角在 15m 外会把地面翘成障碍）、自动合并 scans*.pcd 分段、高度带以地面为 z=0（min_z=0.10 / max_z=0.45=车可通行高度）
- auto_relocalize：启动自动重定位 + `/relocalize` 服务 + 绑架看门狗（**双 σ 似然场**：全局搜索 σ=0.25，健康检查 σ=0.08——宽 σ 下错误位姿也能蒙 0.7 分，必须用窄 σ 区分）
- mapping/navigation launch：启动时防重复实例 + 自动清理 /dev/shm 残留 + 建图前清空旧 PCD 分段

---

## 6. 陷阱手册（按症状索引，全部为源机实测）

| 症状 | 根因 | 处置 |
|------|------|------|
| nav2 容器 100% CPU、零日志、生命周期节点全不创建 | 被 kill -9 的进程在 /dev/shm 残留**已锁定的 `sem.fastrtps_*` 信号量**，新进程加锁死锁 | `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*`（注意 `fastrtps_*` 匹配不到 `sem.` 前缀！须两条都删，先停全部 ROS 进程）。launch 已内置自动清理 |
| Ctrl+C 关不掉 launch | humble nav2 容器关闭时 bond 死锁（社区已知） | 等10秒不退就 `~/stop_nav.sh`；连按 Ctrl+C 没用 |
| 启动报"检测到已有建图/导航实例" | 防重复启动保护（双实例会互相打架：组件挤进同名容器、串口冲突、CPU爆） | 按提示先停旧实例 |
| ros2 CLI 报 `Failed init_port ... open_and_lock_file failed` 或 daemon 报 `!rclpy.ok()` | 被 timeout/SIGKILL 强杀的 CLI 进程污染 SHM/daemon | 清 SHM + `ros2 daemon stop && ros2 daemon start` |
| 驱动起了但无数据，或握手失败 | 用了 MID360（无s）的配置格式 | MID360s 配置键名是 `Mid360s`、`host_net_info` 是数组带 `host_ip`，用 `MID360s_config.json` |
| FAST-LIO 满屏 No Effective Points | 时间戳补丁没打 / 双驱动实例并存 | 见 5.1；`pgrep -af livox` 查重复实例 |
| RViz 收不到 /map 或 /scan | QoS 不匹配 | /map: Reliable+Transient Local；/scan: Best Effort |
| 2D 地图满地黑噪点、导航 no valid path | 地面倾斜切进障碍带（校平失效/未启用）或建图太快覆盖差 | 检查 pcd2pgm 拟合日志；慢速重建图 |
| "Starting point in lethal space" 时好时坏 | ① /scan min_height 太低，车身俯仰把远处地面扫成幻影障碍环（已设0.15）② 规划器还是 SmacHybrid（见5.3-2） | 症状持续可把 min_height 提至 0.18-0.20 |
| 行驶左右扭摆/甩头 | MPPI wz_std/temperature 过大、ObstaclesCritic 膨胀参数与 costmap 不一致 | 见 5.3-3/4 |
| 搬动小车后不自动重定位 | 看门狗阈值/σ 不对（宽 σ 下错误位姿也有 0.7 分） | 确认 auto_relocalize 是双 σ 版本；手动兜底 `ros2 service call /relocalize std_srvs/srv/Trigger` |
| 建图后 save_map 找不到 scans*.pcd | 建图没有 Ctrl+C 正常退出 / 时长不足10秒 / PCD 目录不存在 | 正常退出；`mkdir -p src/FAST_LIO/PCD` |
| 想保留上次建图点云 | mapping.launch 启动会自动清空 PCD 目录 | 启动前备份 |
| 内存持续上涨最终 OOM（建图） | pcd_save.interval 被改回 -1 | 保持 interval: 100 |

---

## 7. NUC 与源机（RDK X5）的差异备忘

- **架构**：x86_64 vs aarch64——所有 build/install 不可复用，必须全部重编；代码本身无需任何改动。
- **编译速度**：快很多（fast_lio 源机 7.5 分钟，NUC 预计 1-2 分钟；放心全量编译）。
- **网卡名**：不是 `eth0`，用 `ip link` 现查，nmcli 命令里替换。
- **算力余量**：MPPI `batch_size` 可回调 2000；`consider_footprint: true` 无压力；如日志出现 `Control loop missed its desired rate` 再酌情下调。
- **雷达 IP**：本台雷达 SN 不同，IP 大概率不是源机的 192.168.1.12，按 Phase 4 探测。
- **sudo 密码/用户名**：可能与源机不同，硬编码路径处理见 Phase 0。
- **底盘串口**：udev 规则要重装（Phase 2），车型 car_mode 要确认（Phase 5）。

---

## 8. 日常使用速查（移植完成后）

```bash
ros2 launch wheeltec_fastlio mapping.launch.py     # 建图(完后 Ctrl+C 存PCD)
ros2 launch wheeltec_fastlio save_map.launch.py    # PCD → 2D导航地图
ros2 launch wheeltec_fastlio navigation.launch.py  # 导航(自动重定位)
ros2 launch wheeltec_fastlio lidar_test.launch.py  # 雷达单独测试
ros2 service call /relocalize std_srvs/srv/Trigger # 手动重定位
~/stop_nav.sh                                      # 关不掉时强停+清理
```

更多细节见 `wheeltec_fastlio/README.md`（随包拷贝）。
