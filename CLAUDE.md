# CLAUDE.md — MID360s + FAST-LIO2 建图导航项目移植指南（鲁班猫4 / RK3588S 版）

> **本文档的使用方式**：把本文件复制到鲁班猫4 的 ROS2 工作空间根目录（`~/wheeltec_ros2/src/CLAUDE.md`），
> 在该目录启动 Claude Code。你（Claude）的任务是按本文档把源机小车上已完成的
> MID360s 3D 雷达建图导航系统完整复现到本机（鲁班猫4，RK3588S，**4GB 内存 + 32GB 存储**）。
> 本文档由源机上完成原始开发的 Claude 编写，包含全部实测踩坑记录——**遇到问题先查第 6 节陷阱手册**。
>
> **本机最大的约束是 4GB 内存**：编译和运行的每一步都要考虑内存，相关措施已写入各 Phase，不要省略。

---

## 📡 网络架构：双无线网卡（2026-07-24 定稿，已实机验证）

**板载网卡跑站点模式连现有WiFi，USB网卡常开热点，两块卡同时在线。**
这不是"二选一切换"——两条路一直都在，所以开关热点不会影响任何走WiFi/有线的连接。

| 网卡 | 设备 | 芯片/驱动 | 角色 | 连接名 |
|------|------|-----------|------|--------|
| 板载 | `wlan0` | RTL8852BE / `8852be.ko`(Realtek树外) | 站点，连现有WiFi | `dmx-public` |
| USB | `wlan1` | RTL8821CU / `8821cu.ko`(Realtek树外) | **热点 WHEELTEC-CAR** | `wheeltec-ap-usb` |
| 有线 | `eth0` | RTL8211F | 调试通道(`autoconnect=no`，需手动激活) | `eth0-debug` |

实测三者可同时在线，热点 `ipv4.never-default=yes` 不抢默认路由：

```
wlan1  type AP   channel 6   WHEELTEC-CAR   192.168.0.100     # 热点(秒级启动)
wlan0  managed   channel 44  dmx-public     192.168.122.225   # 家里WiFi
eth0                                        192.168.122.137   # 有线
default via 192.168.122.1 dev eth0                            # 热点没抢路由
```

一次性配置：`sudo bash wheeltec_webapp/wifi_ap_setup.sh`，然后 `sudo nmcli con up wheeltec-ap-usb`。
profile 是 `autoconnect yes`，之后开机自动起热点。手机连 `WHEELTEC-CAR`/`CHANGE_ME`
访问 `http://192.168.0.100:8080`。网页"网络"卡片分两行显示两块卡状态，热点可随时开关。

### ⚠️ 绝对不要用板载卡（wlan0）做热点

**这是本项目踩过的最大的坑，会导致整机卡死、只能物理断电重启。** 2026-07-23 实测过三次，
从轻到重依次是：安静地切换失败 → 切换成功但彻底失联(只能物理重启) → **整机直接卡死**。

根因（三条硬证据，都是只读命令查出来的）：

```
lspci -nnk     → Kernel driver in use: rtl8852be
modinfo 8852be → depends:          ← 空! 说明不走 mac80211, 只挂 cfg80211,
                                      AP 是 Realtek 私有实现
iw list        → interface combinations are not supported   ← 上报了零个合法接口组合
                 Available Antennas: TX 0 RX 0              ← 明显是假值
iw reg get     → phy#2 (self-managed) country 99            ← Realtek"未设置国家"占位符
```

对比 USB 卡（`iw phy phy1 info`），同为 Realtek 树外驱动但 AP 路径是完整的：

```
valid interface combinations:
  * #{ managed, P2P-client } <= 2, #{ AP, P2P-GO } <= 1, total <= 2, #channels <= 1
```

**`interface combinations are not supported` 就是判断一张卡能不能做AP的关键指标**，
`wifi_ap_setup.sh` 的 `0)` 段会自动检查这一项，遇到零组合的卡直接拒绝配置。

社区已确认 RTL8852BE 起热点会卡在 "Creating virtual interface" 整机挂死：
[lwfinger/rtw89#402](https://github.com/lwfinger/rtw89/issues/402)、
[morrownr/USB-WiFi#352](https://github.com/morrownr/USB-WiFi/issues/352)。
注意查资料要认准 `rtl8852be`/`8852be.ko`，**搜 `rtw89` 是白费功夫**——mainline 的 rtw89
虽然编进了内核，但 `/boot/config-6.1.84` 里 `# CONFIG_RTW89_8852BE is not set`，
这颗芯片的支持根本没编出来。真想换 mainline 驱动需要另外下载鲁班猫内核源码
（`/lib/modules/6.1.84/build` 只是 headers 软链），但既然USB卡方案已经跑通，没必要折腾。

### 其它相关踩坑

- **`sudo iw reg set CN` 对板载卡是完全无效的空操作**：`phy#2` 是 self-managed regdomain，
  只接受驱动自己下发的 regd。改国家码只能走模块参数 `rtw_country_code`(当前 `(null)`)。
  USB 卡(`phy#1`)不是 self-managed，跟随全局 `country 00`，但该域下 5GHz 全部是
  `PASSIVE-SCAN`(禁止主动发射)，**所以热点只能做在 2.4G ch1-11**，信道固定为 6。
- **整机卡死后 `/sys/fs/pstore/` 是空的，不能证明没有内核 panic**：pstore 根本没配后端
  （`/proc/cmdline` 里无 `ramoops=`）。要抓卡死原因得配 ramoops，或接 USB-TTL 到
  debug UART（cmdline 里有 `console=ttyFIQ0`）。
- **硬件看门狗**：`/dev/watchdog0` 存在但默认没启用。2026-07-23 为排查卡死临时开过
  （`RuntimeWatchdogSec=30`，RK 的 dw_wdt 实际取整到 44s），**大编译前务必关掉**，
  否则 `colcon build` 重度换页时 PID 1 喂不上狗会导致机器莫名重启（见陷阱手册）。
  当前状态：已关闭。查证：`systemctl show -p RuntimeWatchdogUSec`（`0` 才是关闭）。

### 给后续会话的提醒

- Claude Code 的 Bash 工具是通过 SSH 会话跑的，**不是独立于小车网络的通道**。现在热点
  做在独立网卡上，常规操作不再有失联风险；但如果要动 `wlan0`/`dmx-public` 本身
  （比如改WiFi密码、重连），仍然会切断走WiFi的会话。走的是WiFi还是有线**每个会话都要实查**：
  ```bash
  cat /proc/$$/cgroup | grep -o 'session-[0-9]*'      # 例如 session-1
  loginctl show-session 1 -p Leader -p RemoteHost
  ss -tnp | grep ':22 '     # 本端IP是 eth0 的地址(192.168.122.137)才算安全
  ip -br -4 addr show
  ```
- **webapp 由 systemd 托管**（`wheeltec-webapp.service`，已 enabled 开机自启）。
  不要用 `pkill` 去杀它——那样 systemd 不会自动拉起来，要 `sudo systemctl restart wheeltec-webapp`。
  改完 `src/` 的代码必须 `colcon build --packages-select wheeltec_webapp` 才会进 `install/`，
  webapp 跑的是 `install/` 里的副本。
---

## 1. 任务目标与工作方式

### 最终验收标准（按顺序全部通过）

1. `/livox/lidar` 点云 ~10Hz，`/livox/imu` ~200Hz
2. FAST-LIO 建图：`/Odometry` ~10Hz，无 "No Effective Points" 刷屏；建图中每约 10 秒落盘一个 `scans_N.pcd`
3. `save_map` 生成 2D 地图，日志显示地面拟合倾角 <3°、雷达离地高度合理
4. 导航启动后自动重定位 `score > 0.9`
5. RViz（远程 PC）下发 Nav2 Goal 小车顺利到点，行驶无扭摆
6. 抱起小车搬到别处，10 秒内自动重新定位
7. **全程 `free -h` 无 OOM、无进程被内核杀掉**（`dmesg | grep -i "killed process"` 为空）

### 工作方式（重要）

- **每个 Phase 验证通过再进入下一个**，不要跳步。
- 测试用的后台 launch **用完立即停干净**（见陷阱手册"双实例灾难"）。
- 杀 ROS 进程只用 Ctrl+C/SIGINT；被迫强杀后必须清理 `/dev/shm`（第 6 节）。
- 长命令（编译）注意内存：另开终端 `watch free -h` 盯着。
- 建议把本机踩到的新坑追加记录到你自己的 memory。

---

## 2. 源机信息（所有源码从这里拷贝）

| 项 | 值 |
|----|-----|
| 源机 | wheeltec 小车，RDK X5 主控（aarch64——**与本机同架构，但编译产物仍不可复用**：glibc/依赖库版本不同，全部重编） |
| SSH | 用户 `wheeltec`，密码 `CHANGE_ME`，IP 在源机上 `hostname -I` 查（WiFi 动态分配） |
| 工作空间 | `/home/wheeltec/wheeltec_ros2/`（src 约 2GB，含 39 个包目录） |

### 需要拷贝的内容

```bash
# 在鲁班猫上执行 (先 mkdir -p ~/wheeltec_ros2/src)。SRC_IP 替换为源机实际 IP
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
| `navigation2-humble` | nav2 源码（工作空间自带，随 colcon 一起编译，**编译内存大户**） |

---

## 4. 分阶段移植步骤

### Phase 0：环境侦查与资源准备（本机专属，务必先做）

```bash
uname -m                  # 应为 aarch64
cat /etc/os-release       # 需要 Ubuntu 22.04 (jammy)
ls /opt/ros/ 2>/dev/null  # 有 humble 则跳过下面安装步骤
free -h                   # 4GB 内存确认
df -h /                   # 32GB 存储，需保证 >=12GB 可用(源码2G+build/install~4G+swap6G)
ip link show              # 记下网卡名(RK平台一般是 eth0/eth1; WiFi 视型号)
whoami                    # 记下用户名(鲁班猫镜像常见 cat 或 lubancat，非 wheeltec!)
```

**① 安装 ROS2 Humble（若 `/opt/ros/humble` 不存在）**：

```bash
sudo apt update && sudo apt install -y locales curl gnupg2 lsb-release
sudo locale-gen en_US.UTF-8

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu \
  $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
# 选 ros-base（无 RViz/桌面，本机不跑 RViz，省 ~500MB 安装和内存）
sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
ls /opt/ros/humble   # 确认存在
```

**② 创建 swap（4GB 内存编译必须，没有 swap 编译 nav2/fast_lio 必 OOM）**：

```bash
free -h | grep -qi swap.*[1-9] || {
  sudo fallocate -l 6G /swapfile && sudo chmod 600 /swapfile
  sudo mkswap /swapfile && sudo swapon /swapfile
  echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
}
free -h   # 确认 Swap 6G
```

**③ 关闭桌面环境（省 ~1GB 内存，本机是无头小车主控，桌面无用）**：

```bash
sudo systemctl set-default multi-user.target && sudo systemctl isolate multi-user.target
free -h   # available 应明显增加
```

**④ 硬编码路径**：若本机用户名不是 `wheeltec`（鲁班猫常见是 `cat`），拷贝完源码后必须修正：

```bash
grep -rln "/home/wheeltec" ~/wheeltec_ros2/src/wheeltec_fastlio/ ~/stop_nav.sh
# 逐个 sed 替换为实际 home 路径。涉及: mapping.launch.py 的 PCD_DIR、
# save_map.launch.py 的 pcd_dir/src_map_dir、pcd2pgm.cpp 默认参数
# 示例(USERNAME 替换为实际用户名):
# sed -i "s|/home/wheeltec|/home/USERNAME|g" ~/wheeltec_ros2/src/wheeltec_fastlio/launch/mapping.launch.py
# sed -i "s|/home/wheeltec|/home/USERNAME|g" ~/wheeltec_ros2/src/wheeltec_fastlio/launch/save_map.launch.py
# sed -i "s|/home/wheeltec|/home/USERNAME|g" ~/wheeltec_ros2/src/wheeltec_fastlio/src/pcd2pgm.cpp
# sed -i "s|/home/wheeltec|/home/USERNAME|g" ~/stop_nav.sh
```

FAST_LIO 的 PCD 保存路径是编译期宏（ROOT_DIR=源码目录），本机重新编译后自动正确，无需修改。

### Phase 1：拷贝源码、瘦身、校验补丁

执行第 2 节的 rsync，然后：

**① 屏蔽本项目用不到的重型包（省编译时间/内存/磁盘，不物理删除）**：

```bash
cd ~/wheeltec_ros2/src
for p in largemodel tts_make_ros2 wheeltec_mic wheeltec_mic_aiui ollama_ros_chat-ros2 \
         wheeltec_bodyreader ros2_astra_camera-master usb_cam-ros2 web_video_server-ros2 \
         aruco_ros-humble-devel wheeltec_robot_kcf wheeltec_robot_rtab wheeltec_robot_rrt2 \
         simple_follower_ros2 msc; do
  [ -d "$p" ] && touch "$p/COLCON_IGNORE"
done
# 注意: 不要屏蔽底盘基础包
#（turn_on_wheeltec_robot、wheeltec_robot_msg、wheeltec_imu、wheeltec_robot_urdf、
#  wheeltec_robot_keyboard、nav2_waypoint_cycle、wheeltec_robot_nav2、navigation2-humble、
#  interfaces、depend 必须保留编译）
```

**② 校验关键补丁存在**（防止拷到未打补丁的版本）：

```bash
grep -c "WHEELTEC patch" ~/wheeltec_ros2/src/livox_ros_driver2/src/comm/pub_handler.cpp   # 必须=2
grep -c "WHEELTEC patch" ~/wheeltec_ros2/src/FAST_LIO/src/laserMapping.cpp                 # 必须=1
ls ~/wheeltec_ros2/src/FAST_LIO/include/ikd-Tree/ikd_Tree.cpp   # 子模块必须存在
grep -c "behavior_server:" ~/wheeltec_ros2/src/wheeltec_robot_nav2/param/wheeltec_params/param_S100_diff.yaml  # ≥1
mkdir -p ~/wheeltec_ros2/src/FAST_LIO/PCD    # PCD输出目录
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

# Livox-SDK2 编译安装（MID360s 必需 SDK2，限 -j4 防 OOM）
cd ~/Livox-SDK2 && rm -rf build && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j4 && sudo make install && sudo ldconfig

# 底盘串口 udev 规则（生成 /dev/wheeltec_controller）
cd ~/wheeltec_ros2/src/turn_on_wheeltec_robot && sudo bash wheeltec_udev.sh
# 之后重新插拔底盘 USB 线, ls /dev/wheeltec_controller 验证
```

### Phase 3：编译（4GB 内存专用流程）

**⚠️ 绝对禁止运行 `livox_ros_driver2/build.sh`——它会 `rm -rf` 整个工作空间的 build/install！**

livox_ros_driver2 一次性文件准备（rsync 拷来的应已就绪，检查）：

```bash
cd ~/wheeltec_ros2/src/livox_ros_driver2
ls package.xml launch/ 2>/dev/null || { cp package_ROS2.xml package.xml; cp -r launch_ROS2 launch; }
```

**编译策略**：RK3588S 是 4×A76+4×A55，CPU 不弱，瓶颈是 4GB 内存。
限制并行度 + swap 兜底，不要用默认 `-j8`（必 OOM）：

```bash
source /opt/ros/humble/setup.bash
cd ~/wheeltec_ros2
export MAKEFLAGS="-j3"     # 单包内并行3，配合下面包级串行

# 1) 驱动
colcon build --packages-select livox_ros_driver2 \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
source install/setup.bash

# 2) FAST-LIO（模板重、单文件内存峰值高，单独编，预计10-20分钟）
MAKEFLAGS="-j2" colcon build --packages-select fast_lio

# 3) 其余全量，包级并行限为2（navigation2-humble 是大头，预计1-2小时）
colcon build --parallel-workers 2 2>&1 | tail -20
```

编译中若终端卡死/进程被杀（`dmesg | grep -i killed`）→ 内存不够：降为
`MAKEFLAGS="-j2" colcon build --parallel-workers 1 --executor sequential`，删掉失败包的 build 目录重来。

必须成功的包：`livox_ros_driver2, fast_lio, wheeltec_fastlio, wheeltec_nav2, turn_on_wheeltec_robot, nav2_*`；
被 COLCON_IGNORE 屏蔽之外的附属包失败可 `--packages-skip` 跳过。

编译完检查磁盘：`df -h /`（build+install 约 3-4GB；紧张时可 `rm -rf build/`，install 独立可用）。

### Phase 4：雷达网络

```bash
# 1. 找出接雷达的网口(插网线的那个): ip link show 看哪个有 CARRIER
# 2. 配静态IP（IFACE 替换为实际网卡名，RK 平台一般是 eth0/eth1）:
sudo nmcli con add type ethernet ifname IFACE con-name livox-mid360 \
  ipv4.method manual ipv4.addresses 192.168.1.5/24 ipv4.never-default yes connection.autoconnect yes
sudo nmcli con mod livox-mid360 connection.autoconnect-priority 100
sudo nmcli con up livox-mid360
# 若系统用 netplan 而非 NetworkManager: 在 /etc/netplan/ 加静态配置后 netplan apply

# 3. 找雷达IP: 出厂 IP = 192.168.1.1XX，XX=本台雷达SN码后两位(和源机那台不同!)
for i in $(seq 100 199); do ping -c1 -W0.3 192.168.1.$i &>/dev/null && echo "雷达IP: 192.168.1.$i"; done

# 4. 把实际雷达IP写进配置(lidar_configs.ip 字段), 然后重编译:
#    编辑 ~/wheeltec_ros2/src/wheeltec_fastlio/config/MID360s_config.json
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
# 同时 watch free -h 盯内存（FAST-LIO 建图占用应稳定，持续上涨说明分段保存失效）
# 遥控(ros2 run wheeltec_robot_keyboard wheeltec_keyboard)慢速走一圈, Ctrl+C 退出

# --- 转图 ---
ros2 launch wheeltec_fastlio save_map.launch.py
# 检查日志: "地面校平完成: 拟合倾角 X.X°, 雷达离地高度约 X.XXm" → 回填 lidar_z
# 地图存至 wheeltec_nav2/map/WHEELTEC3D.{pgm,yaml} (install+src两份)

# --- 导航 ---
ros2 launch wheeltec_fastlio navigation.launch.py
# 预期日志: "Managed nodes are active" ×2 → "重定位成功: ... score=0.9x"
# 每30秒 "定位健康度(严格分): 0.6~0.8" 为正常
# 检查CPU/控制频率: 日志若频繁刷 "Control loop missed its desired rate of 20Hz"
#   → 把 param 里 MPPI batch_size 降到 1000（见第7节性能调优）
# RViz(远程PC)下发 Nav2 Goal 验证到点; 最后做搬车绑架测试（等5~10秒自动恢复）
```

RViz 配置要点（RViz 只在远程 PC 上跑，**不要在鲁班猫上跑**，4GB 内存吃不消）：
`/map` 显示必须 Reliability=**Reliable** + Durability=**Transient Local**；
`/scan` 显示必须 Reliability=**Best Effort**。设反了就是"收不到数据"。

---

## 5. 关键补丁与修复清单（校验/手工补齐用）

### 5.1 livox_ros_driver2 时间戳补丁（2 处，`src/comm/pub_handler.cpp`）

**症状（不打的后果）**：FAST-LIO 随机报 `No Effective Points!` / `IMU and LiDAR not Synced`。
**根因**：MID360s 的点云包与 IMU 包 time_type 不一致 → 原版驱动点云用雷达内部时钟、IMU 用主机时钟，双时钟漂移。
**补丁内容**：① `OnLivoxLidarPointCloudCallback` 里无条件 `is_timestamp_sync_.store(false);`（替换原按 time_type 判断的 if/else）；② `GetEthPacketTimestamp` 无视 time_type 一律 `return std::chrono::high_resolution_clock::now().time_since_epoch().count();`。两处都有 `WHEELTEC patch` 注释标记。

### 5.2 FAST_LIO PCD 保存补丁（`src/laserMapping.cpp`）

**症状**：建图退出后 PCD 目录为空。
**根因**：ROS2 移植版把 `publish_frame_world` 里整个 `if (pcd_save_en){...}` 累积块用 `/* */` 注释掉了。
**补丁**：取消该块注释（搜 `WHEELTEC patch` 标记）。配套配置：`wheeltec_fastlio/config/wheeltec_mid360.yaml` 中 `pcd_save.interval: 100`（分段保存防 OOM——**本机 4GB 内存，绝对不要改回 -1**，全量累积 13 分钟就要 2.5GB）。

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
5. **footprint 实测尺寸**（本车量取，local/global 两处 costmap 都要改）。

### 5.4 wheeltec_fastlio 包内已含的机制（了解即可，随包拷贝）

- pcd2pgm：RANSAC 地面校平（消除安装倾角，1° 倾角在 15m 外会把地面翘成障碍）、自动合并 scans*.pcd 分段、高度带以地面为 z=0（min_z=0.10 / max_z=0.45=车可通行高度）
- auto_relocalize：启动自动重定位 + `/relocalize` 服务 + 绑架看门狗（**双 σ 似然场**：全局搜索 σ=0.25，健康检查 σ=0.08——宽 σ 下错误位姿也能蒙 0.7 分，必须用窄 σ 区分）
- mapping/navigation launch：启动时防重复实例 + 自动清理 /dev/shm 残留 + 建图前清空旧 PCD 分段

---

## 6. 陷阱手册（按症状索引，全部为源机实测）

| 症状 | 根因 | 处置 |
|------|------|------|
| 编译中进程被杀/机器卡死 | **4GB 内存 OOM**（本机最常见问题） | 确认 swap 已启用；降 MAKEFLAGS 到 -j2、`--parallel-workers 1`；`dmesg \| grep -i killed` 确认 |
| 编译到一半机器**莫名重启**（不是卡死、不是被 kill，是干净地重启了） | **硬件看门狗**（2026-07-23 为排查WiFi热点卡死而启用，`/etc/systemd/system.conf` 的 `RuntimeWatchdogSec=30`，RK 的 dw_wdt 实际取整到 **44s**）。本机 4GB 内存 + swap 在 eMMC 上，`colcon build` 重度换页时 PID 1 可能喂不上狗 → SoC 自动复位 | 跑大编译前先关掉：`sudo sed -i 's/^RuntimeWatchdogSec=30/#RuntimeWatchdogSec=0/' /etc/systemd/system.conf && sudo systemctl daemon-reexec`。查证当前值：`systemctl show -p RuntimeWatchdogUSec`（`0` 才是关闭） |
| nav2 容器 100% CPU、零日志、生命周期节点全不创建 | 被 kill -9 的进程在 /dev/shm 残留**已锁定的 `sem.fastrtps_*` 信号量**，新进程加锁死锁 | 停全部 ROS 进程后 `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*`（两条都要，`fastrtps_*` 匹配不到 `sem.` 前缀）。launch 已内置自动清理 |
| Ctrl+C 关不掉 launch | humble nav2 容器关闭时 bond 死锁（社区已知） | 等10秒不退就 `~/stop_nav.sh`；连按 Ctrl+C 没用 |
| 启动报"检测到已有建图/导航实例" | 防重复启动保护（双实例互相打架：组件挤进同名容器、串口冲突、CPU爆） | 按提示先停旧实例 |
| ros2 CLI 报 `Failed init_port ... open_and_lock_file failed` | 被强杀的 CLI 进程污染 SHM/daemon | 清 SHM + `ros2 daemon stop && ros2 daemon start` |
| 驱动起了但无数据/握手失败 | 用了 MID360（无s）的配置格式 | MID360s 配置键名是 `Mid360s`、`host_net_info` 是数组带 `host_ip`，用 `MID360s_config.json` |
| FAST-LIO 满屏 No Effective Points | 时间戳补丁没打 / 双驱动实例并存 | 见 5.1；`pgrep -af livox` 查重复实例 |
| RViz 收不到 /map 或 /scan | QoS 不匹配 | /map: Reliable+Transient Local；/scan: Best Effort |
| 2D 地图满地黑噪点、导航 no valid path | 地面倾斜切进障碍带（校平失效）或建图太快覆盖差 | 检查 pcd2pgm 拟合日志；慢速重建图 |
| "Starting point in lethal space" 时好时坏 | ① /scan min_height 太低，车身俯仰把远处地面扫成幻影障碍环（已设0.15）② 规划器还是 SmacHybrid（见5.3-2） | 症状持续可把 min_height 提至 0.18-0.20 |
| 行驶左右扭摆/甩头 | MPPI wz_std/temperature 过大、ObstaclesCritic 膨胀参数与 costmap 不一致 | 见 5.3-3/4 |
| `Control loop missed its desired rate of 20Hz` 频繁刷屏 | MPPI 算力不足 | batch_size 降 1000；仍不行降 controller_frequency 15 |
| 搬动小车后不自动重定位 | 看门狗阈值/σ 不对（宽 σ 下错误位姿也有 0.7 分） | 确认 auto_relocalize 是双 σ 版本；手动兜底 `ros2 service call /relocalize std_srvs/srv/Trigger` |
| 建图后 save_map 找不到 scans*.pcd | 建图没有 Ctrl+C 正常退出 / 时长不足10秒 / PCD 目录不存在 | 正常退出；`mkdir -p src/FAST_LIO/PCD` |
| 想保留上次建图点云 | mapping.launch 启动会自动清空 PCD 目录 | 启动前备份（注意 32GB 磁盘，备份后及时清理） |
| 建图内存持续上涨 | pcd_save.interval 被改回 -1 | 保持 interval: 100，本机 4GB 绝不能用 -1 |
| 磁盘满 | 32GB 小盘：PCD 分段、日志、build 目录累积 | `rm -rf ~/wheeltec_ros2/build` 可安全删；清 `~/.ros/log`、旧 PCD |
| 想在**板载卡 wlan0** 上起热点 → 整机卡死/彻底失联 | RTL8852BE + Realtek树外驱动 `8852be.ko`（**不是 `rtw89`**），不走 mac80211，`iw list` 上报零个 interface combinations，社区已知起热点会整机挂死 | **热点只做在USB卡 wlan1 上**（`wheeltec-ap-usb`），板载卡永远只做站点模式。详见文档最前面的"📡 网络架构"。查资料认准 `rtl8852be`/`8852be.ko`，搜 `rtw89` 是白费功夫 |
| 换了别的USB无线网卡后热点起不来/卡死 | 不是所有卡都能做AP，Realtek 树外驱动尤其容易上报零个接口组合 | 先查 `iw phy <phy> info \| grep -A2 "interface combinations"`，必须能看到含 `AP` 的合法组合。`wifi_ap_setup.sh` 的 `0)` 段会自动拦截 |
| 热点起来后本机上不了外网 | 热点抢了默认路由 | `ipv4.never-default yes`（脚本已设）。查证 `ip route \| grep default` 应指向 eth0/wlan0 |
| 热点只能用2.4G，设5G不生效 | USB卡跟随全局 `country 00`，该域下 5GHz 全是 `PASSIVE-SCAN`(禁止主动发射) | 只能用 2.4G ch1-11（已固定 ch6）。想开5G需要正确设置国家码，但树外驱动的 regdomain 支持不可靠 |
| `sudo iw reg set CN` 设了没反应 | 板载卡 `phy#2` 是 **self-managed** regdomain(`iw reg get` 显示 `country 99`)，只接受驱动自己下发的 regd，`iw reg set` 对它是空操作 | 改国家码只能走模块参数 `rtw_country_code`(当前是 `(null)`)。USB卡 `phy#1` 不是 self-managed，跟随全局设置 |
| `pkill` 杀掉 webapp 后它没自己起来 | webapp 由 systemd 托管(`wheeltec-webapp.service`)，`pkill` 绕过了 systemd，`Restart=on-failure` 对被信号杀死不生效 | `sudo systemctl restart wheeltec-webapp`；查状态 `systemctl status wheeltec-webapp` |
| 改了 webapp 代码但网页没变化 | webapp 跑的是 `install/` 里的副本，不是 `src/` | `colcon build --packages-select wheeltec_webapp` 后重启服务 |
| 整机卡死后 `/sys/fs/pstore/` 是空的，以为"没有内核panic" | pstore **根本没配后端**(`/proc/cmdline` 里无 `ramoops=`)，硬件看门狗也没开(`RuntimeWatchdogUSec=0`，但 `/dev/watchdog0` 是存在的) | 空的 pstore 不能证明没 panic。要抓卡死原因：配 ramoops，或接 USB-TTL 到 debug UART(`console=ttyFIQ0`)看串口输出 |

---

## 7. 鲁班猫4 与源机（RDK X5）的差异备忘 + 性能调优

- **架构**：同为 aarch64，代码无需任何改动，但编译产物不可复用（系统库版本不同），全部重编。
- **CPU**：4×A76(2.4G)+4×A55 vs 源机 8×A55——**单核性能强于源机**，FAST-LIO/MPPI 运行余量更大；瓶颈只在内存。
- **内存 4GB（源机 7GB）**：
  - 编译必须 swap + 限并行（Phase 0/3）
  - 运行期各节点合计约 1.5-2GB，关桌面后够用；不要在本机跑 RViz
  - MPPI `batch_size`：先用源机的 1000 跑通；CPU 富余（无 Control loop missed）可尝试 1500 提控制质量
- **存储 32GB eMMC**：编译后删 build 目录；PCD 及日志定期清理；swap 放 eMMC 会加速磨损，属可接受代价
- **网卡名**：RK 平台一般保留 eth0/eth1 命名，仍以 `ip link` 实查为准
- **用户名/密码**：鲁班猫镜像常见 `cat`/`CHANGE_ME`（现场确认）；硬编码路径处理见 Phase 0④
- **温控**：RK3588S 编译满载发热大，确认散热片/风扇在位；`cat /sys/class/thermal/thermal_zone0/temp` 超 85000（85°C）会降频，编译变慢属正常
- **雷达 IP**：本台雷达 SN 不同，IP 大概率不是源机的 192.168.1.12，按 Phase 4 探测
- **底盘串口**：udev 规则重装（Phase 2），车型 car_mode 确认（Phase 5）

---

## 8. 日常使用速查（移植完成后）

```bash
ros2 launch wheeltec_fastlio mapping.launch.py     # 建图(完后 Ctrl+C 存PCD)
ros2 launch wheeltec_fastlio save_map.launch.py    # PCD → 2D导航地图
ros2 launch wheeltec_fastlio navigation.launch.py  # 导航(自动重定位)
ros2 launch wheeltec_fastlio lidar_test.launch.py  # 雷达单独测试
ros2 service call /relocalize std_srvs/srv/Trigger # 手动重定位
~/stop_nav.sh                                      # 关不掉时强停+清理
free -h && df -h /                                 # 内存/磁盘巡检
cat /sys/class/thermal/thermal_zone0/temp          # 检查 CPU 温度（>85000 则降频）
```

更多细节见 `wheeltec_fastlio/README.md`（随包拷贝）。
