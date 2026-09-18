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
3. `save_map` 生成 2D 地图，日志显示"与期望法向偏差" <3°、雷达离地高度合理（水平安装时该值即原来的"拟合倾角"）
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

导航链路:  MID360s --PointCloud2--> pointcloud_to_laserscan --> /scan (0.15~0.45m)
                                                              --> AMCL + 两个 costmap
           (Astra 深度相机**已退出局部代价地图**, 只留驱动给网页看画面; 见 5.7.0)
           auto_relocalize: 启动自动重定位 + 绑架检测看门狗 --> /initialpose
           TF: map --(AMCL)--> odom_combined --(EKF 轮式+IMU)--> base_footprint
           Nav2(NavFn规划 + MPPI控制) 复用 wheeltec_nav2 框架
```

- 相机**只喂 costmap 避障, 不参与定位**: 定位仍然全靠雷达+AMCL, 相机拔了照样能导航
  (`use_camera:=false`)。建图阶段完全不用相机。

- FAST-LIO 的 TF（camera_init→body）与底盘 TF 是**两棵独立的树**，互不冲突；建图时 RViz Fixed Frame 用 `camera_init`，导航时用 `map`。
- 建图与导航**不能同时运行**（驱动数据格式不同）。

### 关键包职责

| 包 | 职责 |
|----|------|
| `wheeltec_fastlio` | **核心集成包**（原创）：pcd2pgm（PCD→2D地图，含地面校平）、auto_relocalize（扫描匹配重定位+看门狗）、lidar_test/mapping/save_map/navigation 四个 launch |
| `livox_ros_driver2` | MID360s 驱动，**已打时间戳补丁** |
| `ros2_astra_camera-master` | Orbbec Astra S 驱动（导航时起，**已打 TF 去重补丁**，见 5.5） |
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
# 注意: ros2_astra_camera-master 从 2026-08-05 起是**必编包**(导航相机避障),
# 不要再屏蔽它; N10Plus/Odin1 相关包则已归档屏蔽(见下面第二个循环)
for p in largemodel tts_make_ros2 wheeltec_mic wheeltec_mic_aiui ollama_ros_chat-ros2 \
         wheeltec_bodyreader usb_cam-ros2 web_video_server-ros2 \
         aruco_ros-humble-devel wheeltec_robot_kcf wheeltec_robot_rtab wheeltec_robot_rrt2 \
         simple_follower_ros2 msc \
         wheeltec_n10plus wheeltec_odin1 odin_ros_driver wheeltec_lidar_ros2; do
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

# Astra 相机(导航避障)依赖。注意 glog 的包名是 libgoogle-glog-dev, 不是 libglog-dev；
# image-geometry/image-publisher 会拉 libopencv-dev → 撞 RKMPP ffmpeg 冲突,
# 装不上就 apt-get download 后 sudo dpkg-deb -x xxx.deb / 手工解包(见第6节)
sudo apt install -y libuvc-dev libgoogle-glog-dev ros-humble-camera-info-manager \
  ros-humble-image-geometry ros-humble-image-publisher
# 相机 udev 规则(生成 /dev/astra_s, 否则驱动打不开设备):
sudo cp ~/wheeltec_ros2/src/ros2_astra_camera-master/astra_camera/scripts/56-orbbec-usb.rules \
  /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger

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

- pcd2pgm：RANSAC 地面校平（消除安装倾角，1° 倾角在 15m 外会把地面翘成障碍）、自动合并 scans*.pcd 分段、高度带以地面为 z=0（min_z=0.10 / max_z=0.45=车可通行高度；倾装时期曾改成 2.0/1.2，已回退，见 5.7.0）
- auto_relocalize：启动自动重定位 + `/relocalize` 服务 + 绑架看门狗（**双 σ 似然场**：全局搜索 σ=0.25，健康检查 σ=0.08——宽 σ 下错误位姿也能蒙 0.7 分，必须用窄 σ 区分）
- mapping/navigation launch：启动时防重复实例 + 自动清理 /dev/shm 残留（2026-09-18 起只删没有任何进程映射/打开的文件组，不再按进程名猜）+ 建图前清空旧 PCD 分段
- depth_obstacle_filter（2026-08-05 新增）：Astra 深度图 → 导航用稀疏障碍点云，见 5.5

### 5.5 Astra 深度相机避障（2026-08-05 新增，已实机验证）

> **⚠️ 本节的相机俯角已于 2026-08-06 作废**：相机改成平视（`camera_pitch` 21.1°→**1.0°**），
> 雷达也移到了相机上方并下倾 42°。最新外参和标定方法见 **5.7**，本节余下内容
> （链路设计、三个关键设计、驱动补丁、验证方法）仍然有效。
>
> **⚠️ 2026-08-05 雷达被挪过位置，外参已更新**（这一段也已被 5.7 取代，保留作历史记录）：`lidar_x` 由 -0.20 改为 **-0.135**
> （车体几何中心 x=-0.15 往前 1.5cm），`lidar_z` 由 0.34 改为 **0.30**
> —— 0.30 是当天重建图后由 `save_map` 地面拟合回填的实测值
> （"拟合倾角 0.7°, 雷达离地高度约 0.30m", 内点 36684）；底座卷尺量到 0.22m，
> 即 MID360 光心比底座面高约 8cm（曾按 4cm 估成 0.26，实测差 4cm，**别再用估的**）。
> mapping/navigation 两个 launch 都已改。旧地图（用 -0.20/0.34 建的）已作废，
> 当天已重建 `WHEELTEC3D`（723x408）。


**为什么要加**：MID360s 装在 0.30m 高，`/scan` 只取地面上 0.15~0.45m 的一层，
车前 0.5m 内和地面矮物（门槛、拖鞋、凳子横档）完全看不见。相机补的就是这块。

**链路**：`astra_camera_node`（导航 launch 起，深度 640x480@30 + 彩色 320x240@15）
→ `wheeltec_fastlio/depth_obstacle_filter` → `/camera/obstacle_points`
→ nav2 **local costmap** 的 `camera` 观测源（PointCloud2）。

三个关键设计（改之前先读 `src/depth_obstacle_filter.cpp` 的文件头注释）：

1. **不用驱动自带的 `/camera/depth/points`**（`enable_point_cloud: false`）：
   那是 640x480 稠密 XYZ，3.7MB/帧 @18Hz ≈ 66MB/s，光 DDS 搬运就够本机喝一壶。
   直接订阅深度图（614KB/帧）按 stride=4 抽样反投影，实测输出 ~700 点/帧 @8~10Hz。
2. **输出点云的 frame 保持相机光心系**（不是 base_footprint）：nav2 用观测帧原点
   当 raytrace 起点，发成车体系会让清除射线从车中心发出，清错格子。
   高度/距离判据仍在 base_footprint 系里算。
3. **只进 local costmap，不进 global**：全局图不是滚动窗口、相机 FOV 只有 ~58°，
   误检的点没有射线去清，会在全局图上留下永久幻影障碍（能把门口堵死）。
   理由写在 `param_S100_diff.yaml` 的 global_costmap 段里。

**安装位置（2026-08-05 本车实测，已写死在 navigation.launch.py 默认值）**：
`camera_x=0.09`（车头 x=+0.10 往回 1cm）、`camera_z=0.21`（镜头离地）、`y/yaw/pitch=0`。
**故意不读** `robot_model.yaml` 的 `base_to_camera`（出厂支架值 `[0.166, 0.001, 0.090]`）——
本车相机不在原厂支架上，高度差 12cm，照抄会把地面整片当障碍。
实测验证（相机开着跑一遍反投影统计）：车前 0.4~0.8m 的地面点读到的高度是 -0.006~-0.011m，
即**近场标定误差在 1cm 以内、无系统性俯仰偏差**；车体轮廓内(x<0.10)零点，不会扫到自己。
倾角错 1~2°，3m 外的地面就会被抬进障碍带，所以动过相机支架就要重测。
launch 发的静态 TF 子 frame 是 `camera_mount_link` 而**不是 `camera_link`**：
底盘 launch（`robot_mode_description.launch.py` 的 `base_to_camera`）和 URDF 各自
已经在发 `camera_link` 了，再发一个就是一个 child 多个 parent，tf2 树会被反复重挂。

**驱动补丁（5.5 补丁，`ob_camera_node.cpp` 的 `calcAndPublishStaticTransform`）**：
原版把 `camera_color_frame` 同时挂在 `camera_depth_frame` 和 `camera_link` 下（双 parent），
已注释掉后一行，标记 `WHEELTEC patch`。校验：
`grep -c "WHEELTEC patch" ~/wheeltec_ros2/src/ros2_astra_camera-master/astra_camera/src/ob_camera_node.cpp` 必须 ≥1。

**验证方法**（不用真拿东西挡在车前）：往 `/camera/obstacle_points` 灌一片假点
（相机光心系 x右/y下/z前，z=1.0、y=相机高-目标高），看 `/local_costmap/costmap` 的
lethal 格子数是否增加。实测 364 → 378（注入 208 点 ≈ 0.5m 宽的墙）。

**编译依赖**（本机 apt 有坑，见第 6 节）：`libuvc-dev`、`libgoogle-glog-dev`（不是 `libglog-dev`）、
`ros-humble-camera-info-manager`、`ros-humble-image-geometry`、`ros-humble-image-publisher`。

---

### 5.6 ⚠️ 本场地地面是镜面抛光的（2026-08-05 发现，影响三个环节）

**现象**：抛光地面在掠射角下把激光/红外整片镜面反射走，探测器收不到回波。
Astra（红外结构光）和 MID360（905nm 激光）**都吃这个亏**，不是某个传感器坏了。

实测证据：
- Astra 压到 46°（接近垂直入射）时地面拟合残差 1mm、点很密；改成 21° 浅角度后
  同一片地面**一个点都测不到**（下 3/4 画面有效像素 0%）
- MID360 建图后 `save_map` 的地面拟合内点只有 36684/630985 = **5.8%**
  （正常室内地面通常占 20~40%）

**三个后果和对策**：

| 环节 | 后果 | 对策 |
|------|------|------|
| pcd2pgm 建图 | "可通行"完全靠地面点判定，地面测不到 → 地图 94.4% 未知、自由区仅 3.9%，房间内部全是麻点空洞 | 已加**形态学闭运算 + 填封闭空洞**（`free_close_radius:=2`、`fill_enclosed:=true`），实测 3.9%→5.6%、最大连通自由区 37.4m²。日志会打印 "自由区: 原始…→闭运算…→填内部空洞…" |
| NavFn 规划 | `allow_unknown:false` 过不去那些未知格 → **下了目标点车纹丝不动**，看起来像"导航坏了" | 已改 `allow_unknown: true`。墙壁仍是占据格挡着，未知区当可走，进去后靠局部代价地图实时避障 |
| 相机避障 | 好事：地面测不到 ⇒ "地面被误判成障碍"的风险归零 | 所以 `min_z` 敢放到 0.05 去抓门槛/拖鞋。**换到地毯/哑光地面要提回 0.08~0.10** |

**换场地时记得回退**：地面能正常回波的地方，`allow_unknown` 可以改回 `false`（更安全），
`min_z` 提回 0.08~0.10。

---

### 5.7 雷达安装的两次变更与最终状态（2026-08-06）

> **⚠️ 先看这里：本节大部分内容记录的是"雷达下倾 42°"那个中间状态，当天晚些时候
> 已经全部回退——雷达改回水平安装(360°)、装在相机正上方。**
> 下面 5.7.0 是最终状态；5.7.1 起是倾装时期的记录，**数值已作废，但方法学仍然有效**
> （标定手段、交叉验证思路、踩过的坑），再改安装时照着做。

#### 5.7.0 最终状态（雷达水平，装在相机正上方）

| 参数 | 值 | 依据 |
|------|-----|------|
| `lidar_x` | **0.08** | 卷尺实测（相机镜头在 0.09，雷达光心比镜头靠后 1cm） |
| `lidar_y` | 0.0 | 未测 |
| `lidar_z` | **0.28** | 倾装后由点云三条线索一致给出（见下）；卷尺给的是 0.25 |
| `lidar_pitch` | **0.191 (10.94°)** | IMU 重力法。**不取地面拟合值**——见下 |
| `lidar_roll` | **0.006 (0.32°)** | 同上 |

> 雷达最后一次改装是**下倾约 11°**（为了让地面有回波：水平时镜面地面几乎收不到
> 地面点，FAST-LIO 的 z 缺约束会漂，实测点云 z 直方图里地面从 −0.35 糊到 +0.5）。
> 11° 下 `0.15~0.45m` 带方位覆盖 **99%**、正后方 4622 点，**不需要**像 42° 那次拆两路。
| `camera_pitch` | 0.017 (1.0°) | 用真墙标定，两条判据交汇（见 5.7.3） |

##### ⚠️ 教训：镜面地面场地里，水平雷达的 `lidar_z` 不要信点云，用卷尺

2026-08-06 实测踩了这个坑。雷达改回水平后，镜面地面在掠射角下几乎没有回波
（5.6 的老问题；倾装 40° 时入射角陡、地面内点率 28%，能测准，改回水平就退回去了），
而镜面产生的"地下鬼点"会把低处点 RANSAC 整个带偏。三种点云方法的结果：

| 方法 | 结果 | 判定 |
|------|------|------|
| 低处点 RANSAC（内点3812/残差13.6mm） | 0.443 | **错** —— 锁在地面以下的镜像鬼点上 |
| 远处墙脚高度中位数 | 0.474 | **错** —— 各扇区 std=0.35，参照物是家具不是墙 |
| 最密的水平面拟合 | 0.230 | **对**（卷尺 0.25，差 2cm），但当时被误判成"锁在0.21m高的台面上"而否掉了 |

注意残差小、内点多**并不能说明拟合对**——错误的那个残差只有 13.6mm。
高度直方图在 3~4m 处的峰按 0.443 算是 +0.206，换成 0.25 就是 +0.013，正是地面；
**证据一直都在，是选错了**。

##### ⚠️ `lidar_z` 能不能信点云，取决于雷达倾没倾

这是当天最反复的一条，最后结论很明确：

| 安装方式 | 点云拟合出的 `lidar_z` | 真值 | 判定 |
|---------|---------------------|------|------|
| 水平 | 0.443 / 0.446（两次） | 0.28 | **完全不可信**，差 16cm |
| 水平 + 离群点未裁 | 0.10 | 0.28 | 更离谱 |
| **下倾 11°** | **0.27** | 0.28 | **可信**，差 1cm |

水平时镜面地面在掠射角下几乎无回波，镜面产生的"地下鬼点"把低处点 RANSAC 整个带偏；
**而且残差只有 13mm、内点数千——残差小、内点多完全不说明拟合对**，当天就是被这两个
数字说服而采信了错值。倾 11° 之后地面回波够了（点云 z 直方图里出现清晰的地面尖峰，
水平时完全没有峰），拟合值立刻对上。

最终取 **0.28**：点云 z 直方图尖峰 0.2875、RANSAC 0.27、后向视场几何也是 0.28 更贴合，
三条一致。卷尺给的 0.25 差 3cm，多半来自"光心比底座高 8cm"这个经验值（实际可能 11cm）。

**所以：倾装后可以信点云；水平安装时只能信卷尺。**
`save_map` 的日志会打印拟合值与所用值的差并给判语——倾装 >10° 时差超过 5cm 就要查
（要么 `lidar_z` 填错，要么建图时 z 漂了）。

##### 用视场几何验证 `lidar_z`（不依赖地面）

MID360 垂直视场 −7°~+52°。**倾装之后前后不对称，要用后向那一侧验证**：
前向的最低可见高度会被地面本身卡住（看到地面就到底了），而**后向是纯视场受限**，
最低可见高度 ≈ `lidar_z + d·tan(倾角−7°)`，直接由 `lidar_z` 决定。

下倾 11° 时实测（正后方扇区）：

| 距离 | 1.0~1.5m | 2.0~3.0m |
|------|----------|----------|
| 实测最低可见 | +0.40m | +0.45m |
| 理论(z=0.25) | +0.34m | +0.42m |
| 若 z=0.446 则应为 | +0.54m | +0.62m |

实测贴着 0.25 的理论线，与 0.446 差 14~17cm ⇒ `lidar_z=0.25` 成立。

水平安装时没有后向不对称，改用各距离环的最低 1% 分位对 `lidar_z − d·tan(7°)`：

| 距离 | 0.8~1.0m | 1.0~1.5m | 1.5~2.0m | 2.0~3.0m |
|------|----------|----------|----------|----------|
| 实测最低可见 | +0.168m | +0.110m | +0.050m | −0.044m |
| 理论(z=0.25) | +0.139m | +0.097m | +0.035m | −0.057m |

全程差约 3cm。`lidar_z` 要是填错 19cm，这四个数会整体偏 19cm——**这条检验非常灵敏，
而且完全绕开了镜面地面**。改过雷达高度就跑一遍这个。

##### 近场盲区

由同一条公式：碰撞带下沿 0.15m 在 `d = (0.25−0.15)/tan(7°) ≈ 0.81m` 处进入视野
（实测 `/scan` 最近点 0.82m，吻合）。即 **0.8m 外就能看全 0.15~0.45m 整条带**，
0.8m 以内是盲区。装得越高盲区越大：`lidar_z` 若为 0.443，这个距离会变成 2.4m。
深度相机已退出局部避障，0.8m 以内靠底盘超声波的固件级急停兜底。

##### ⚠️ pcd2pgm 的两个静默故障：离群点撑爆包围盒 + 地面高度不可信

2026-08-06 建完图后 `save_map` 的输出：地图 **960x813(48x40m)**、自由区 **2.2%**、
未知 **97.1%**、"雷达离地高度约 **0.10m**"（实际 0.25）。四个数全不对，根因两条：

**① 几十个离群点把包围盒从 23x19m 撑到 150x173x54m。** 实测这批点云 99.9% 的点在
17.8m 内、中位数才 2.88m，但最远的到 95m、z 到 49m（镜面地面/玻璃的多路径反射 +
Livox 野点）。后果**两个都是静默的**：

- `VoxelGrid` 用 int32 做体素索引，盒子一大就溢出 → PCL 只在 stderr 打一行
  `Leaf size is too small for the input dataset`，然后**原样返回、不降采样**。
  日志里"降采样后 N 点"和输入一模一样就是这个信号。
- 地图按包围盒开尺寸 → 960x813 格，真实内容缩在角落，未知率虚高到 97%。

已修：新增 `outlier_percentile`（默认 0.1）参数，按分位数裁剪后再降采样。
用分位数而不是固定量程，地图多大都自适应。修完：

| | 修前 | 修后 |
|---|---|---|
| 包围盒 | 150x173x54m | 18.8x20.8x5.4m |
| 降采样 | 静默失效(4042949→4042949) | 4042949→594155 |
| 地图 | 960x813 | 386x381 |
| 自由区 / 未知 | 2.2% / 97.1% | 11.4% / 85.6% |
| 法向偏差 | 1.7° | 0.3° |

**② 地面高度别信点云拟合，传 `lidar_z` 进去。** 镜面地面场地地面回波极少，
拟合出的高度不可靠（修前 0.10 vs 卷尺 0.25，差 15cm；修后也还差 6cm）。
高度错 15cm 会让障碍带 `[min_z,max_z]` 整体平移，矮障碍漏检、可通行区误判。
已修：`pcd2pgm` 新增 `lidar_z` 参数，>0 时**直接用它定地面高度**，
RANSAC 只用来拟合**法向**做校平（法向受镜面影响小得多，修后偏差 0.3°）。
日志会同时打印拟合值供对照。`save_map.launch.py` 默认传 0.25。

##### 相机已退出局部代价地图（2026-08-06 用户决定）

`param_S100_diff.yaml` 的 `observation_sources` 已把 `camera` 摘掉，launch 新增
`camera_avoid` 开关默认 `false`（不启动 `depth_obstacle_filter`）。`use_camera` 仍默认
true 只管相机驱动，所以网页"实时画面"不受影响。**两边都要打开才会真的生效**，这是故意的。

##### 回退掉的东西（倾装时期的补救，现已撤销）

`/scan` 拆两路 → 撤回单路 0.15~0.45（实测方位覆盖 100%、正后方 30303 点）；
地图 `max_z` 2.0/1.2 → 回到 0.45、`min_z` → 0.10；两个 costmap 的观测源 → `/scan`；
`pcd2pgm` 的 `lidar_pitch`/`lidar_roll` 参数**保留**但默认 0/0，此时期望法向就是 +Z，
行为与原版完全一致，以后再倾装不必改代码。

---

### 5.7.1 ⚠️（历史）雷达下倾 42° + 相机改平视

改装内容：雷达移到相机正上方并**下倾约 45°**（实测 41.8°）；相机位置不动、由下倾 21° 改成**平视**；
底盘 32 控制器加装了 6 路超声波做固件级急停。

#### 实测外参（全部有独立交叉验证，不是卷尺量的）

| 参数 | 旧值 | **新值** | 怎么测出来的 |
|------|------|---------|-------------|
| `lidar_x` | -0.135 | **+0.111** | 雷达和相机看同一个平面，比较各自到该面的**垂距** |
| `lidar_y` | 0.0 | 0.0（未测） | 两传感器看同一平面的偏航差 <1.5°，沿用 0 |
| `lidar_z` | 0.30 | **0.307** | 点云 RANSAC 地面拟合 |
| `lidar_pitch` | （无此参数） | **0.702 rad = 40.2°** | ①IMU 重力 ②地面拟合，**两法只差 0.71°** |
| `lidar_roll` | （无此参数） | **−0.004 rad = −0.24°** | 同上（两法都 <0.25°，基本已扶正） |

> 上表是 **2026-08-06 当天第二次标定**的终值。当天雷达先移到相机上方下倾 45°（标出
> x=0.078 / pitch=0.729 / roll=0.034），随后又被**前移约 3.3cm**，于是全部重测。
> 前一版数值已作废，但那次的方法学记录仍然有效。
| `camera_pitch` | 0.368 (21.1°) | **0.017 (1.0°)** | 拟合车前 0.5m 的墙面：法向竖直分量归零 + 墙脚落地，两条判据在 1° 交汇 |
| `camera_x/y/z` | 0.09/0/0.21 | 不变（`camera_z` 已复核，误差 1cm） | 同上 |

三种标定手段各自独立，且互相印证：地面拟合给 `lidar_z`，IMU 重力给倾角，看同一平面给
`lidar_x`。最后 TF 实测复核：`lookup_transform` 出来 roll −0.23°/pitch +40.22°/yaw 0，
地面在 0.5~8m 全程落在 **+6mm ~ −10mm**，且不随距离单调漂移（前一版在 3m 处是 −5cm，
roll 扶正 + pitch 修正后明显改善）。

**求 `lidar_x` 必须用完整向量投影，不能简单相减**：设平面 `n·p = c`（`n` 指向传感器一侧），
两个原点到该面的垂距差满足 `d_L − d_C = n·(O_L − O_C)`，其中 `O_L−O_C = (dx, 0, lidar_z−camera_z)`，
解出 `dx` 再加 `camera_x`。参照面**不需要**垂直、也不需要正对车头（实测纸箱偏航 2.6°），
简单相减会把这些偏差算进 x 里。实测：`d_C=0.477`(内点213772/RMS 0.7mm)、
`d_L=0.459`(内点24729/RMS 4.7mm)、`n=(−0.9990, 0.0322, 0.0325)` → `dx=+0.021` → `lidar_x=+0.111`。

**⚠️ 纸箱可以用来求 `lidar_x`，但不能用来标 `camera_pitch`**：相机俯角的"法向竖直"判据
要求参照面真的垂直，纸箱后仰几度判据就偏几度。实测同一个箱子，判据①给 +3.9°、
判据②（面底贴地）给 −2°，**两条根本不交汇**；而用真墙时两条干净地交汇在 1.0°。
两条判据不交汇就是参照物不合格的信号，这时候要换真墙，不要取平均。
（另外相机平视、目标在 0.5m 时，判据②本身也会被视野下沿截断：0.21m 高、半视场 22.5°，
在 0.477m 处最低只能看到离地 0.012m，"面最低点"根本不是箱子底边。）

**标定脚本的两个坑**（重标时会再踩）：
- **Livox 对无回波点填 (0,0,0)**，拟合前必须 `norm > 0.15` 滤掉。不滤的话原点处 24 万个
  假点会把平面拟合整个带偏——第一次跑就算出了 `lidar_x=+0.546` 的鬼值。
- `np.linalg.svd(Q-c)` 对几十万点会去申请 453GB（要 `full_matrices=False`，或直接对
  协方差矩阵 `eigh`）。

#### 连锁改动一：/scan 必须拆成两路（**这是本次最大的改动**）

雷达倾下去之后，后方视野整片翻到天花板上，**原来的 0.15~0.45m 碰撞带在正后方一个点都没有**。
实测各高度带的方位角覆盖（72 个 5° 扇区）：

| 高度带 | 方位覆盖 | 正后方点数 |
|--------|---------|-----------|
| 0.15~0.45m | 57% | **0** |
| 0.15~0.80m | 76% | 1227 |
| 0.15~1.20m | 81% | 2751 |
| **0.15~2.00m** | **100%** | 15380 |

后方最低能看到的点在 0.5~1.76m 高，所以带顶必须抬到 2m；天花板实测 2.7~2.8m，2.0 不会扫进天花板。
于是拆成两路，各管各的：

- **`/scan` 0.15~1.2m** → 只给 **AMCL 和 auto_relocalize**。地图（`save_map` 的 `max_z`）
  必须**同为 1.2**，否则 scan 里有而图上没有的点会被似然场当作失配惩罚。
  > 一度取 2.0（方位覆盖 100%），但 0.45~2.0m 之间的桌面/窗台/挂架全被画进地图当障碍。
  > 2026-08-06 退回 **1.2** 折中（覆盖 81%，远好于原来的 57%）。
  > **注意 pcd2pgm 的图幅是按障碍带里的点算包围盒的**：压低 max_z 会让图幅跟着内缩，
  > 连带裁掉外围的自由区（实测 25.2×19.7m → 20.4×13.2m）。改这个参数要顺便确认
  > 机器人的作业范围没有被裁到图外。
- **`/scan_obstacle` 0.05~0.45m** → 只给 **两个 costmap**（`param_S100_diff.yaml` 里
  local 的 voxel_layer 和 global 的 obstacle_layer，雷达源都已改成这个话题）。
  高于车顶的桌沿、挂架不该拦路，所以避障不能用高带。

改完实测：`/scan` 有效束 94.4%、方位覆盖 100%、正后方 260 束；`/scan_obstacle` 覆盖 55.6%、
最近 0.35m。**代价**：`/scan_obstacle` 正后方 0 束，倒车时后方是瞎的（局部图里进场时标记的
障碍还在，不是全瞎）——加装的超声波正好补这块，见下面。

#### 连锁改动二：pcd2pgm 的地面拟合会直接失败

FAST-LIO 的 `camera_init` **不是重力对齐的**：`IMU_Processing` 初始化时 `rot=单位阵`、
`grav=-mean_acc`，世界系直接取开机瞬间的 IMU 姿态——雷达怎么装，这个系就怎么歪。
倾 42° 之后地面法向离 +Z 有 42°，而原代码写死 `setAxis(UnitZ)` + 20° 容差，
**一个内点都找不到，`save_map` 直接失败**。

已改：`pcd2pgm` 新增 `lidar_pitch`/`lidar_roll` 参数，按安装角算出地面法向的期望方向当搜索轴；
选"低处点"也改成沿该方向投影（原来用原始 z，系是歪的，选出来的根本不是低处）。
日志相应改成打印**与期望法向的偏差**——这不再是"雷达装歪了多少"（它就是故意歪的），
而是**安装角标定的残差，>3° 说明该重标了**。

#### 连锁改动三：相机的 min_z 必须提回来, 否则车会被自己围死

**这是 2026-08-06 实机踩到的坑, 症状是"下了目标点走不动、像是把地面当成了障碍"。**

5.6 当初敢把 `depth_obstacle_filter` 的 `min_z` 放到 0.05, 理由是"地面镜面抛光、
掠射角下压根测不到地面点, 所以地面不可能被误判成障碍"。**相机改平视之后这个前提塌了**:
镜面地面不再是"测不到", 而是**测出一堆假点**——实测反投影出 −0.17 ~ −0.36m 的
"地下"鬼点(镜像反射), 其中一部分会落进 0.05~0.15 这个带里被当成障碍。

定位过程(这套方法可复用):

1. 车停着不动, 清空 local costmap(`clear_entirely_local_costmap` 服务), 然后盯着
   lethal 格数随时间变化。**真障碍会立刻回到某个值然后稳住; 误标记会持续累积。**
   实测: 清空后立刻回到 81, 7 秒内涨到 126 并继续涨。
2. A/B 隔离: 停掉 `depth_obstacle_filter` 再清空重测 → lethal 稳定在 80~92 不涨。
   **凶手是相机, 不是雷达。**(同期查证雷达是干净的: 0.3~1.5m 内 17778 个点落在
   |z|<0.02m, 0.02~0.05m 只有 307 个, 地面老老实实在 z=0。)
3. 为什么累积不掉: 相机 FOV 只有 58°, marked 的格子背后没有射线去清,
   车一转向就永远留在那儿, 膨胀层把车围死。

已改: `min_z` 0.05 → **0.15**, `max_range` 4.0 → **2.5**(平视后远处角度误差被放大,
且 2.5m 以外雷达覆盖得比相机好); `param_S100_diff.yaml` 里 camera 源的
`min_obstacle_height` 同步 0.05 → 0.15。改完实测 lethal 稳定在 80~90, 与"完全关掉相机"
一致, 而近距离真障碍仍在(相机每帧点数 ~580 → 114)。

**静态点云统计看不出这个问题**(静止时地面确实干净), 必须用上面第1步那个测试。
换到地毯/哑光地面可以再往下试 0.10, 但要重做该测试。

#### ⚠️ 独立的大坑：地图里"已经搬走的障碍"会让导航中途失败

**这是 2026-08-06 折腾一整天的真正根因**，和雷达倾装无关，是 nav2 的固有行为。

**机理**：全局代价地图里 `static_layer` 把地图栅格直接写进主图，而 `obstacle_layer`
的清除射线**只作用于它自己那一层，碰不到 static_layer**。所以建图时录进去的东西，
即使实际已经搬走，在全局规划眼里永远还在。而**局部**代价地图（`global_frame:
odom_combined`，只有 voxel+inflation 两层）会正确地清掉它。两边不一致 →

```
车按局部图判断"这儿是空的"开进去
  -> 一进到旧障碍的位置, 全局图认为车正站在障碍里
  -> NavFn 起点非法 -> 规划失败
```

**日志特征**（照着这个认，非常好认）：

```
INFO  [bt_navigator]      Begin navigating from current location to (x, y)
***   /plan: 113 点, 长 3.07m          <- 一开始规划是成功的!
ERR   [controller_server] Optimizer fail to compute path      <- MPPI 先失败
WARN  [behavior_server]   Collision Ahead - Exiting Spin      <- 恢复行为也被挡
WARN  [planner_server]    GridBased: failed to create plan with tolerance 0.30
WARN  [planner_server]    Planning algorithm GridBased failed to generate a valid path
INFO  [global_costmap]    Received request to clear entirely the global_costmap  <- 无用功
WARN  [behavior_server]   Collision Ahead - Exiting DriveOnHeading / backup failed
ERR   [bt_navigator]      Goal failed
```

**关键点：`clear_entirely_global_costmap` 对这种情况完全无效**——它只清 `obstacle_layer`。
日志里能看到恢复行为反复调它、反复没用，这本身就是识别信号。

**处置**：`wheeltec_fastlio/scripts/clean_map.py`，用实时激光反查把这些格子从 pgm 里擦掉。

```bash
# 干跑, 只报告不写盘
ros2 run wheeltec_fastlio clean_map.py --duration 60
# 确认后写盘(自动备份 + 调 map_server/load_map 热加载, 不用重启导航)
ros2 run wheeltec_fastlio clean_map.py --duration 180 --write
```

运行期间用遥控把车开过那些"障碍已搬走"的区域。判据（三条同时满足才擦）：
① 被射线穿过 ≥15 次；② **从未**成为过任何一帧的射线终点；③ 8邻域内也没有终点格。
条件②③是安全阀——真墙一定会被打中，一旦打中就永久排除。实测候选数会先涨后跌
（515→253），就是这个否决机制在收敛，**方向永远是偏保守的**。

前提是**定位要准**（看 auto_relocalize 的健康度，正常 0.6~0.8）。定位飘的时候跑这个
会擦错格子。彻底的办法仍然是重新建图。

#### 连锁改动四：相机的职责变小了

平视之后镜头在 0.21m 高、竖直半视场约 22.5°，视野下沿打到地面已在 0.51m 之外，
**车前 0.5m 内的矮物相机看不到了**。好在雷达补上了：下倾 42° 后视野下沿落地在雷达前
0.27m 处，即车体系 x≈0.35m（车头在 0.10m），**车头前 25cm 就有点**。
所以低矮障碍现在主要靠雷达，相机退化成中距离补充。

#### 附带的好消息：镜面地面的问题缓解了

5.6 记的抛光地面掠射反射问题，主要吃亏在入射角太平。倾 42° 之后入射角陡了很多，
地面拟合内点率从 **5.8%（36684/630985）涨到 28%（4919/17808）**。
重建图后 `save_map` 日志里的"自由区"那行应该会比上次好看不少，
`free_close_radius`/`fill_enclosed` 这些补救参数或许可以放松。

#### 超声波（2026-08-06 加装，**ROS 侧尚未接入**）

固件级急停已在 32 控制器里做好，ROS 侧 `turn_on_wheeltec_robot` 已经在发
`/ultrasonic_data_A~F`（`sensor_msgs/Range`，20Hz，六路都有真实数据）。

**用户加装的实际布局**（与原厂 `ultrasonic_enum.py` 写死的"前排 4 路"**完全不同**）：

| 通道 | 位置 | 通道 | 位置 |
|------|------|------|------|
| D | 前左 | E | 前右 |
| C | 左侧 | F | 右侧 |
| B | 后左 | A | 后右 |

⚠️ 原厂 `ultrasonic_pose.launch.py` 那套（`ultrasonic_enum.py` 三角测量 → `/ultrasonic/points`）
**不能直接用**：它假定 A/B/C/D 是前排一横排、横向位置 `[-0.205,-0.065,0.065,0.205]`，
拿实际这套前/侧/后的布局去解，出来的障碍坐标是错的。要接入得另写节点，
每路 Range 沿自己朝向反投影成一个点。**接入前需要每个探头的实测安装位置和朝向**（尚未量）。

判读提示：车静止正对平墙时，前向的 D/E 读数**一动不动**属正常（超声波打平整垂直面就是死稳的），
不要误判成"传感器坏了"。当时 D/E 读 0.458/0.449，与雷达和相机各自独立测出的墙距
（车头前约 0.486m）吻合。

---

### 5.8 建图时屏蔽雷达正后方扇区（2026-08-07 新增）

**为什么**：遥控建图时人跟在车后面走，不屏蔽的话人会被当成静态结构建进地图；
而且人一直在动，喂给 FAST-LIO 的里程计也是负担。

**做法**：`FAST_LIO/src/preprocess.cpp` 的 `avia_handler` 里按方位角丢点
（`WHEELTEC patch` 标记，两个分支都打了）。判据不用 `atan2`——MID360 每帧两万点，
每点一次三角函数太亏；点在 `-x` 半空间且与 `-x` 轴夹角小于半角，等价于

```
x < 0 && x² > cos²(半角)·(x²+y²)
```

参数 `preprocess.blind_back_deg`（扇区**全角**，度，0=关闭），
`wheeltec_mid360.yaml` 默认 **90**，可用 `mapping.launch.py blind_back_deg:=0` 临时关掉。

**实测**（2026-08-07，静止）：该扇区占全部点的 25.8%；打开后 `/cloud_registered_body`
在 ±150~180° 两个格子里**正好 0 点**，整体只剩 236/118545 = 0.19% 漏在边界上
（离 135° 边界中位 0.32°、最大 1.26°）。这点漏是 IMU↔LiDAR 外参平移
（`extrinsic_T ≈ 2.6cm`）造成的边界模糊，属正常，等效屏蔽角是 ±43.7°。
参考：人肩宽 0.5m 站在车后 1m 只占 ±14°，远在屏蔽范围内。

**只作用于建图链路**。导航的 `/scan` 走 `pointcloud_to_laserscan`，不受影响——
导航时后方仍然看得见。屏蔽后 FAST-LIO 还有 270° 视野，约束绰绰有余。

### 5.9 雷达安装角（2026-08-07 傍晚重新固定后的终值）

**当天雷达松过一次、又被重新固定，一天内测了三轮**，这本身就是最重要的记录：
**支架会松，标定值会过期**。

| 时间 | 状态 | IMU重力法 | 点云地面拟合 | 两法差 |
|------|------|-----------|-------------|--------|
| 下午 | 螺丝松了 | +5.73° | +4.63° | 1.15° |
| 傍晚早些 | 拧紧(角度偏小) | +7.12° | +5.48°（**随距离漂 ±0.83°**）| 1.65° |
| **傍晚重装** | **终值** | **+17.85°** | **+16.71°**（各环稳定）| 1.15° |

**三个 launch 已改为 `lidar_pitch=0.292`（16.7°）**，`lidar_roll=0.006`、`lidar_z=0.28` 不变
（实测 roll 0.34°、z 0.284，与原值吻合）。

#### 倾得越多，点云拟合越可信

这是当天最有用的一条规律，和 5.6 的镜面地面直接相关：

| 下倾角 | 地面拟合内点 / RMS | 随距离稳定性 | 判定 |
|--------|-------------------|-------------|------|
| 5~7° | 1 万级 / 22.8mm | 全场 5.48° vs 6m内 4.17°，**漂 1.3°** | 不可信 |
| **17°** | **17.8 万 / 8.9mm** | 0.5~3m 16.67°、全场 16.71°，**几乎不动** | 可信 |

掠射角下镜面地面几乎没有回波，拟合被"地下鬼点"带偏；倾角一大，入射角变陡，
回波立刻正常。**判据就是"换个取点范围结果动不动"**——漂就是不可信，别看 RMS
（漂的那次 RMS 只有 8.7mm 照样是错的）。

倾角小的时候只能信 IMU 重力法（5000 样本分 5 段互检，散布 **±0.002°**，
镜面地面上照样准）。

> ⚠️ IMU 测的是**雷达相对重力**，`lidar_pitch` 要的是**雷达相对底盘**。
> 只有车停在水平地面上两者才相等；两法差 1.15° 多半就出在这里。
> 本项目约定：两法差 <1.5° 时**取地面拟合**（它才是"相对地面"）。

#### 改完的验证方法（不用重新建图就能查）

拿一帧点云，按 `up=(-sinθ, sinφcosθ, cosφcosθ)` 投影，看各距离环的地面点离 z=0 多远：

| | 0.5~2m | 2~4m | 4~8m |
|---|---|---|---|
| 旧值 0.191 | +11.0cm | +20.4cm | +16.5cm |
| **新值 0.292** | **−0.3cm** | +4.7cm | +9.8cm |

近场基本归零即为正确。远处还剩几厘米是地面本身起伏 + 那 1° 残差，属正常。

**改了这个值必须重新建图**——旧地图是按旧 TF 建的，`/scan` 的高度带和地图对不上，
定位分会一直上不去（见第 6 节"改了 /scan 高度带但没重建地图"那条）。

**网页 3D 视图不受影响**——它每次会话实时实测（见 5.10），不读这些死数。
这正是为什么要做成自测：状态条的"地面"芯片能一眼看出当前实际多少度，
雷达再松一次也能立刻发现。

### 5.10 网页建图页 3D 点云视图（2026-08-07 大修）

#### ⚠️ 校平矩阵的符号——踩过两次的地方

地面法向在 `camera_init` 里是 **`(-sinθ, sinφ·cosθ, cosφ·cosθ)`**，`x` 分量是**负的**
（θ=pitch>0 表示下倾）。推导：`base→livox` 的旋转是 `Ry(θ)·Rx(φ)`，所以
`ẑ` 落到雷达系是 `Rx(-φ)·Ry(-θ)·ẑ`。`pcd2pgm.cpp` 的 `up` 向量就是这个约定；
2026-08-07 用 IMU 重力法实测 `(-0.0998, +0.0168, +0.9949)` 证实。

`v2.html` 之前写成了 `+sinθ`，于是"校平"不是消掉倾角而是**把倾角加倍**（10.94°→21.90°，
已写成单测复现）。症状：网格怎么调都贴不上地面。**改这段务必先重推一遍符号。**

现在校平用的是"把地面法向转到 +Z 的**最小旋转**"（Rodrigues，等价于 pcd2pgm 的
`FromTwoVectors`），不按 pitch/roll 拼 `Ry·Rx`——最小旋转不引入绕 Z 的分量，
校平不会顺带把地图转一个偏航。

#### 地面平面改为实时实测（`app.py` 的 `GroundFitter`）

死数会过期（见 5.9），所以网页不再信 launch 的标称角，**两个量分开估**：

- **法向** ← `/livox/imu` 的加速度（静止即重力反方向）用 `/Odometry` 的姿态转到
  `camera_init`（FAST-LIO 的 body 系就是 IMU 系）。实测 0.1 秒稳定到 0.013°，
  35 秒漂移 0.000°，人为加 ±0.05g 运动噪声只动 0.01°。**镜面地面一个地面点都收不到时它照样准**。
- **高度** ← 沿该法向投影，在雷达下方 `[-1.5,-0.05]m` 做直方图，取**最低的那个强峰**
  （不是最高峰——取最高峰会被贴地的矮台面顶掉）。实测 2 秒（10帧）收敛，88 秒内抖动 ±1mm。

⚠️ **不要退回"直接对 `/cloud_registered` 做 RANSAC 平面拟合"**：那个话题是
`filter_size_surf=0.5` 体素降采样过的，每帧只剩 ~600 点，加上镜面地面，
用标称法向当种子时地面在直方图里整个糊开（峰锐度 1.85x），迭代拟合被墙面带跑，
2026-08-07 试过一次都没收敛。换 IMU 法向后同一批数据锐度 4.57x。

估不出来就退回标称值，状态条的"地面"芯片会显示 **标称**（正常时显示 `28cm 5.6°`，
悬停看法向/高度各自的来源和地面点数）。新建图会话会 `reset()`——`camera_init` 换了。

#### 手势（这是用户反馈"拖动效果还是有问题"的根因）

旧版平移写的是 `tx -= dx·cos(yaw+90°)·s; ty -= (dx·sin(yaw+90°) − dy·cos(pitch))·s`
——横向那半是对的，**纵向那半只改 `ty`，也就是上下拖永远沿世界 Y 走、不跟视角转**。
现在沿相机自己的 X/Y 基向量平移，步长按透视算成"目标平面上 1 像素多少米"，
手指按住哪个点那个点就跟着走。`cam3d` 加了 `tz`（相机平面平移有竖直分量）。

其它一起修的：`touchstart` 改 `passive:false`（iPadOS 不在 start 里 preventDefault
会把手势判给页面滚动）；手指数变化时**重新取基准而不是丢掉手势**（旧版双指松开一根
剩下那根就失灵）；捏合以两指中点为锚点缩放；`touchcancel` 也处理。

**pad 操作**：单指拖=旋转，双指拖=平移，双指捏合=缩放，双击=复位。
工具列有个 ⟳/✋ 按钮把单指切成平移（单手拿遥控时用）。第一次进建图页会弹一张
手势速查卡（`localStorage.v3ges2`，改手势就把这个键的版本号加一让老用户再看一次）。

#### 实景开关

工具列新增 **实景**：后端点云从 1Hz/1200点 提到 5Hz/2500点（实测到手 3.9Hz，
受 0.2s 推送循环限制），前端另存最近 12 帧画成亮层、累积点云压暗当背景，
并强制打开校平 + `[-0.12, 2.0]m` 高度带 = "地面到地面上方 2 米的实时场景"。
累积点云回答"这块扫到没有"，实景回答"我周围现在什么样"，需求相反所以分两个缓冲。
费带宽，所以离开建图页自动关、不做持久化、断线重连要重发 `{t:'scene'}`。

#### 另外修掉的两个隐患

- WebGL 不可用时旧代码 `$('cv3d').outerHTML=...` 把 canvas 整个换掉 → `const cv3`
  指向脱离文档的节点，`clientHeight` 变 0，平移比例尺被 `||1` 兜成 19.66 m/px，
  轻轻一拖飞出去几公里（headless 实测位移 1966m）。改成盖一层提示，canvas 留着。
- `bindTip` 每次调用都挂一遍监听，而 `syncVTool` 会对同一个按钮反复调它换文案 →
  按一下弹好几层。改成文案存 `dataset`、监听只挂一次。

---

## 6. 陷阱手册（按症状索引，全部为源机实测）

| 症状 | 根因 | 处置 |
|------|------|------|
| 编译中进程被杀/机器卡死 | **4GB 内存 OOM**（本机最常见问题） | 确认 swap 已启用；降 MAKEFLAGS 到 -j2、`--parallel-workers 1`；`dmesg \| grep -i killed` 确认 |
| 编译到一半机器**莫名重启**（不是卡死、不是被 kill，是干净地重启了） | **硬件看门狗**（2026-07-23 为排查WiFi热点卡死而启用，`/etc/systemd/system.conf` 的 `RuntimeWatchdogSec=30`，RK 的 dw_wdt 实际取整到 **44s**）。本机 4GB 内存 + swap 在 eMMC 上，`colcon build` 重度换页时 PID 1 可能喂不上狗 → SoC 自动复位 | 跑大编译前先关掉：`sudo sed -i 's/^RuntimeWatchdogSec=30/#RuntimeWatchdogSec=0/' /etc/systemd/system.conf && sudo systemctl daemon-reexec`。查证当前值：`systemctl show -p RuntimeWatchdogUSec`（`0` 才是关闭） |
| nav2 容器 100% CPU、零日志、生命周期节点全不创建 | 被 kill -9 的进程在 /dev/shm 残留**已锁定的 `sem.fastrtps_*` 信号量**，新进程加锁死锁 | 停全部 ROS 进程后 `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*`（两条都要，`fastrtps_*` 匹配不到 `sem.` 前缀）。launch 已内置自动清理 |
| 导航跑着跑着进程间数据断了（发现正常但收不到数据、EKF 发散），导航进程 `/proc/<pid>/maps` 里全是 `/dev/shm/... (deleted)` | **systemd-logind `RemoveIPC=yes`（默认）**：webapp 和它拉起的导航都在 `wheeltec-webapp.service` 里，不算登录会话；cat 最后一个 SSH/终端会话关闭 10 秒后，logind 删掉 cat 名下全部 /dev/shm。FastDDS 见 `*_el` 锁文件没了，还会把活着的段当僵尸删掉（2026-09-18 查明） | 已加 `/etc/systemd/logind.conf.d/10-keep-ros-shm.conf`：`[Login]` `RemoveIPC=no`（不额外占内存；`loginctl enable-linger cat` 也行但常驻约 29MB）。重装系统后要重新加。查证：`busctl get-property org.freedesktop.login1 /org/freedesktop/login1 org.freedesktop.login1.Manager RemoveIPC` 应为 `b false` |
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
| 磁盘满 | 32GB 小盘：**PCD 分段是头号大户——建图每分钟约 100MB**（每 10 秒落一段、每段约 16MB）。2026-08-06 建图忘了关跑了 47 分钟 → 286 段 4.6GB 直接写满盘 | 存过图就 `rm -f src/FAST_LIO/PCD/*.pcd`；`sudo journalctl --vacuum-size=50M`；清 `~/.ros/log`。`mapping.launch.py` 已加水位保护（<2G 拒启动、<5G 警告并给出可建图分钟数） |
| **`rm -rf build/` 之后 `ros2 pkg prefix` 找不到包 / launch 报 local_setup.bash not found** | 本工作空间是 **symlink install**，`install/` 里有文件是指向 `build/` 的软链，删掉 build 就全变悬空（实测 `wheeltec_nav2` 有 11 个）。**"build 可安全删、install 独立可用"这句话对 symlink 安装不成立** | `find install -xtype l` 查悬空，把受影响的包 `colcon build --packages-select <pkg>` 重编一遍即可，不必全量重编 |
| `apt install` 报 `无法连接 mirrors.aliyun.com` 但 ping 得通 | 当前 WiFi 网络到 aliyun 源不通（域名解析正常、TCP 连不上） | 临时换源: 写一个只含 USTC 源的 list 文件, `apt-get -o Dir::Etc::sourcelist=<file> -o Dir::Etc::sourceparts=/dev/null update/install`。**必须用 https**——http 会被中间设备改写, apt 报 `明文签署文件不可用 NOSPLIT` |
| 装 `ros-humble-image-geometry`/`image-publisher`/任何拉 `libopencv-dev` 的包 → `libavutil-dev 将不会被安装` | RKMPP 魔改 ffmpeg (`+rkmpp20230327`) 与官方 ffmpeg 版本锁死冲突（老坑，PCL/OpenCV 也是它） | `apt-get download` 下 deb 后 `sudo dpkg-deb -x xxx.deb /` 手工解包，绕过依赖检查 |
| `apt install libglog-dev` 报找不到包 | jammy 里这个包叫 `libgoogle-glog-dev` | 装 `libgoogle-glog-dev` |
| 相机点云一个点都没有 (`相机障碍点云: 0 点/帧`) | ① 相机正对 <0.35m 的近物(Astra S 近距离盲区), 全被 min_range 滤掉 ② `camera_z`/`camera_pitch` 填错, 整片点被高度带滤掉 | 先把 `min_z:=-5.0 max_z:=5.0` 跑一遍看还有没有点; 有点就是安装参数不对, 没点就是场景/量程问题 |
| 相机点云有, 但 costmap 上多出一圈车前的假障碍 | 相机俯角/高度标定误差, 地面被抬进高度带（和雷达 `min_height` 是同一类坑） | 调大 `depth_obstacle_filter` 的 `min_z`(0.06→0.10), 或重新量 `camera_pitch` |
| `ros2 topic hz/echo` 明明有数据却什么都不打印 | 输出被管道块缓冲, `timeout` 用 SIGTERM 杀掉时没 flush | `stdbuf -oL` 或重定向到文件再看; 这不是 ROS 的问题 |
| 网页"实时画面"卡片一直转/无画面 | 相机只在**导航**时启动(`use_camera:=true`); 建图/空闲时本来就没画面 | 起导航后再看; 卡片收起=前端主动停推流, 不是坏了 |
| 地图上大片本该能走的地方是未知(灰)、自由区只有个位数百分比 | 地面是镜面/抛光的, 激光掠射被反射走收不到回波, 而 pcd2pgm 的"可通行"完全靠地面点 | 见 5.6。`save_map` 日志看"自由区:"那行; 必要时加大 `free_close_radius:=3` |
| 下了目标点车纹丝不动, 也不报错 | ① NavFn `allow_unknown:false` + 地图大片未知(见5.6) ② 车站在 cost>=99 的格子里 | 查 `/local_costmap/costmap` 里机器人所在格的 cost; 救急: `ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}"` |
| 重定位/绑架恢复之后车突然"站在障碍里"起不来 | `local_costmap.global_frame` 原厂写的是 `map`, AMCL 一修正位姿(实测跳0.38m), 旧标记留在原 map 坐标 -> 落到车身底下; 而车身周围没人能清(相机0.85m内盲区、`/scan`是base_footprint系z=0射线只清最底层体素) | 已改 `global_frame: odom_combined`(局部图只跟连续的里程计) |
| 想在**板载卡 wlan0** 上起热点 → 整机卡死/彻底失联 | RTL8852BE + Realtek树外驱动 `8852be.ko`（**不是 `rtw89`**），不走 mac80211，`iw list` 上报零个 interface combinations，社区已知起热点会整机挂死 | **热点只做在USB卡 wlan1 上**（`wheeltec-ap-usb`），板载卡永远只做站点模式。详见文档最前面的"📡 网络架构"。查资料认准 `rtl8852be`/`8852be.ko`，搜 `rtw89` 是白费功夫 |
| 换了别的USB无线网卡后热点起不来/卡死 | 不是所有卡都能做AP，Realtek 树外驱动尤其容易上报零个接口组合 | 先查 `iw phy <phy> info \| grep -A2 "interface combinations"`，必须能看到含 `AP` 的合法组合。`wifi_ap_setup.sh` 的 `0)` 段会自动拦截 |
| 热点起来后本机上不了外网 | 热点抢了默认路由 | `ipv4.never-default yes`（脚本已设）。查证 `ip route \| grep default` 应指向 eth0/wlan0 |
| 热点只能用2.4G，设5G不生效 | USB卡跟随全局 `country 00`，该域下 5GHz 全是 `PASSIVE-SCAN`(禁止主动发射) | 只能用 2.4G ch1-11（已固定 ch6）。想开5G需要正确设置国家码，但树外驱动的 regdomain 支持不可靠 |
| `sudo iw reg set CN` 设了没反应 | 板载卡 `phy#2` 是 **self-managed** regdomain(`iw reg get` 显示 `country 99`)，只接受驱动自己下发的 regd，`iw reg set` 对它是空操作 | 改国家码只能走模块参数 `rtw_country_code`(当前是 `(null)`)。USB卡 `phy#1` 不是 self-managed，跟随全局设置 |
| 网页上局部代价地图位置/角度对不上，车一转弯整片错开 | `local_costmap.global_frame` 是 `odom_combined`（为避免 AMCL 修正位姿后旧标记落到车底），而网页把 costmap 的 origin 当 map 系坐标直接画 —— 两系之间既有平移也有偏航 | 已修：后端查 `map <- odom_combined` 变换原点并把偏航角放进 `meta.yaw`，前端旋转着画。改 `global_frame` 时记得同步 |
| AMCL 位姿慢慢偏、然后跳一下回来 | `update_min_d` 太大：两次滤波更新之间 AMCL 不动 `map->odom`，显示位姿完全由里程计外推，最多能攒满一个 `update_min_d` 的偏差再被一次修正拉回 | `update_min_d` 0.25→0.1、`update_min_a` 0.2→0.1、`transform_tolerance` 1.0→0.3（1秒外推本身就是可见的滞后/超前）。代价是滤波更新频率 2.5 倍，出现 `Control loop missed` 就把 `max_beams` 退回 60 |
| `pkill` 杀掉 webapp 后它没自己起来 | webapp 由 systemd 托管(`wheeltec-webapp.service`)，`pkill` 绕过了 systemd，`Restart=on-failure` 对被信号杀死不生效 | `sudo systemctl restart wheeltec-webapp`；查状态 `systemctl status wheeltec-webapp` |
| 改了 webapp 代码但网页没变化 | webapp 跑的是 `install/` 里的副本，不是 `src/` | `colcon build --packages-select wheeltec_webapp` 后重启服务 |
| `save_map` 报"地面拟合失败(低处点不足)"、或校平后满图黑 | 雷达倾装后 `camera_init` 跟着歪（它不是重力对齐的），而 `pcd2pgm` 的地面搜索轴按安装角算 | 确认 `save_map.launch.py` 的 `lidar_pitch`/`lidar_roll` 与 `mapping.launch.py` **完全一致**。看日志"与期望法向偏差 X°"，>3° 就是安装角标定不准，重标（见 5.7） |
| 定位飘/绑架恢复很慢，`/scan` 看着"少半圈" | 雷达倾装后后方视野翻到天花板，碰撞高度带在正后方收不到点 | `/scan` 必须用 0.15~2.0m 高带且**地图 `max_z` 同为 2.0**；避障另走 `/scan_obstacle`。见 5.7 |
| 改了 `/scan` 高度带但没重建地图 → 定位分一直上不去 | scan 里有而图上没有的点被似然场当失配惩罚 | scan 带和 `save_map` 的 `max_z` 必须一起改、一起重建图 |
| 倒车时撞到后方障碍 | `/scan_obstacle`（0.05~0.45m 碰撞带）在正后方实测 0 束，雷达倾装后的固有盲区 | 已加装超声波做固件急停兜底；ROS 侧接入见 5.7 末尾（尚未做） |
| 用纸箱标相机俯角，"法向竖直"和"面底贴地"两条判据对不上 | 纸箱立面不保证垂直（后仰几度判据就偏几度）；且相机平视时 0.5m 处视野下沿离地只有 1.2cm，"面最低点"根本不是箱底 | 换一面真墙重标，**不要取两条的平均**。判据不交汇=参照物不合格。纸箱只能用来求 `lidar_x`（那个不要求垂直） |
| 导航到一半失败，`GridBased: failed to create plan`，而那块地方**明明是空的** | 建图时录进去的障碍已经搬走，但 nav2 的 `static_layer` 清不掉；车按局部图开进去后，全局图认为车站在障碍里 → NavFn 起点非法。日志特征见 5.7（恢复行为反复调 `clear_entirely_global_costmap` 却没用，就是信号） | `ros2 run wheeltec_fastlio clean_map.py --duration 180 --write`，开着车过一遍那些区域。或重新建图 |
| 下了目标点走不动，看着像"把地面当成了障碍" | 相机 `depth_obstacle_filter` 的 `min_z=0.05` 前提已失效：相机改平视后镜面地面会反投影出 −0.17~−0.36m 的鬼点，部分落进 0.05~0.15 带；相机 FOV 只有 58°，误标的格子没有射线去清，持续累积把车围死 | `min_z` 提到 **0.15**、`max_range` 降到 **2.5**，param 里 camera 源 `min_obstacle_height` 同步改。判定：车停着清空 local costmap 盯 lethal 格数——真障碍会稳住，误标记一直涨；再停 `depth_obstacle_filter` 做 A/B。见 5.7 |
| `save_map` 出来的图巨大(如 960x813)、未知率 95%+、自由区个位数 | 几十个离群点(镜面/玻璃多路径反射)把包围盒撑大数倍；连带 `VoxelGrid` 因 int32 索引溢出而**静默不降采样**(日志里"降采样后 N 点"与输入相同就是信号，PCL 只在 stderr 打一行 `Leaf size is too small`) | 已加 `outlier_percentile`(默认0.1)按分位数裁剪。见 5.7.0 |
| `save_map` 报的"雷达离地高度"与卷尺差十几厘米 | 镜面地面回波太少，RANSAC 拟合的**高度**不可靠(法向还行) | `save_map.launch.py` 传 `lidar_z:=<实测值>`，pcd2pgm 会直接用它定地面高度，只拿 RANSAC 拟合法向 |
| 镜面地面场地里，水平雷达用点云算出的 `lidar_z` 偏大十几~二十厘米 | 掠射角下地面几乎无回波，镜面产生的"地下鬼点"把低处点 RANSAC 带偏。**残差小、内点多并不能说明拟合对**（错的那次残差只有 13.6mm） | 用卷尺（底座+8cm）。验证用**视场几何**而不是地面：每个距离环的最低 1% 分位应满足 `lidar_z − d·tan(7°)`，填错 19cm 这四个数就整体偏 19cm。见 5.7.0 |
| **换到地毯/哑光地面后，地图上整条走廊的地面变成密密麻麻的黑麻点，车过不去**（同一套参数在抛光地面上从没出过这问题） | `lidar_z` 覆盖开关是无条件生效的，而它只该在"镜面地面、拟合不可信"时用。地毯上地面回波正常、拟合值可信（内点占比 51%），但 `lidar_z=0.28` 比拟合值 0.18 高 0.10m —— 校平后整片地面正好被顶到 `z=+0.10 = min_z`，**骑在障碍带下沿上**，噪声决定每个格子黑不黑，于是成麻点。抛光地面根本收不到地面点，所以同样的错误当时完全不显形 | 已加守卫（2026-08-08）：拟合可信（内点占比 ≥ `ground_fit_trust_ratio`，默认 0.25）且 与 `lidar_z` 差 ≥ `min_z/2` 时**自动改用拟合值并打 WARN**。手动办法：`save_map.launch.py lidar_z:=0`。判据看日志"内点 N(占低处点 X%)"：X 大 ⇒ 信拟合，X 小 ⇒ 信卷尺 |
| 拿点云做标定/拟合，结果离谱（如算出雷达在车头前 0.5m） | **Livox 对无回波点填 (0,0,0)**，原点处几十万个假点把平面拟合带偏 | 拟合前先 `norm > 0.15` 滤掉。这不是 ROS 的问题，是 Livox 的数据约定 |
| 前向超声波读数一动不动，以为探头坏了 | 车静止正对平墙时超声波读数本来就是死稳的 | 挪一下车或在探头前晃手再看。六路布局见 5.7（D前左/E前右/C左/F右/B后左/A后右） |
| 整机卡死后 `/sys/fs/pstore/` 是空的，以为"没有内核panic" | pstore **根本没配后端**(`/proc/cmdline` 里无 `ramoops=`)，硬件看门狗也没开(`RuntimeWatchdogUSec=0`，但 `/dev/watchdog0` 是存在的) | 空的 pstore 不能证明没 panic。要抓卡死原因：配 ramoops，或接 USB-TTL 到 debug UART(`console=ttyFIQ0`)看串口输出 |
| 网页3D点云"校平"之后地面反而更斜、网格怎么调都贴不上 | 地面法向的符号搞反了(`+sinθ` 写成了 `-sinθ` 或反之)，校平不是消掉倾角而是**加倍** | 正确的是 `up=(-sinθ, sinφcosθ, cosφcosθ)`，x 分量为负；IMU 重力法可实测验证。见 5.10 |
| 3D 点云拖动方向不跟着视角走 / 上下拖总是沿同一个方向 | 平移只按 yaw 算了横向那一半，纵向那半写死在世界 Y 上 | 沿相机 X/Y 基向量平移，步长用 `2·dist·tan(fov/2)/画布高`。见 5.10 |
| 3D 视图轻轻一拖就飞出去几公里 | 比例尺分母取 `clientHeight||1`，而 canvas 此刻不可见/已脱离文档 → 1 像素当成 19.66m | 分母兜底用 `cv3.height/DPR` 再兜 800，**绝不能兜成 1**；另外别用 `outerHTML` 换掉 canvas |
| 双指操作时松开一根手指，剩下那根就完全失灵 | `touchend` 里一律 `mode=0`，把整串手势丢掉了 | 手指数变化时重新取基准(`baseline`)而不是清状态。见 5.10 |
| 建图时跟车的人被建进地图 | 正后方扇区没屏蔽 | `blind_back_deg:=90`(已是默认)。见 5.8 |
| 网页"地面"芯片一直显示**标称** | 收不到 `/livox/imu`(建图没起)，或 `/Odometry` 停了超过 2 秒 | 起建图后再看；标称状态下网格只是**可能**对不上地面，不影响建图本身 |
| 拿 `/cloud_registered` 做平面拟合怎么都不收敛 | 那个话题是 `filter_size_surf=0.5` 体素降采样过的，每帧只有 ~600 点；配上镜面地面，用错的种子法向会让地面在直方图里整个糊开 | 法向改用 IMU 重力(见 5.10)，高度用直方图最低强峰。别再往 RANSAC 上使劲 |
| 改了 webapp 的 `app.py` 但行为没变 | ① 没 `colcon build` ② 建完没重启服务(**静态页 `v2.html` 会按 mtime 热重载，`app.py` 不会**) ③ **`colcon build` 是在 `~` 跑的**——colcon 会往下递归找到 `~/wheeltec_ros2/src` 里的包，照样编译成功，但产物落在 `~/install`，而服务读的是 `~/wheeltec_ros2/install`，等于白编 | 必须 `cd ~/wheeltec_ros2` 再 build；然后 `sudo systemctl restart wheeltec-webapp` |
| `sudo systemctl restart wheeltec-webapp` 之后建图没了、PCD 也没保存 | 该 unit 没设 `KillMode`，默认 `control-group`——**从网页启动的建图是 webapp 的子进程，在同一个 cgroup 里，重启服务会把它一起杀掉**，而且发的是 SIGTERM 不是 SIGINT，FAST-LIO 不会存 PCD | 建图期间不要重启 webapp；要重启先在网页上正常停止建图 |
| 整机卡死，事后完全查不出原因 | journald 带缓冲，硬断电丢最后几分钟；`/proc/cmdline` 无 `ramoops=`，pstore 没后端；看门狗也关着 | 装黑匣子：`ros2 run wheeltec_fastlio blackbox.py --install`(按提示 sudo 三条命令)。之后 `--report` 能看到断电前最后 30 秒的内存/温度/负载/吃内存前三名 |
| **改了 MPPI 的 `vx_max`/`wz_max`，服务返回成功，车速纹丝不动** | humble 版 `nav2_mppi_controller` 的动态参数只写进 `settings_.base_constraints`，而真正裁剪输出的是 `settings_.constraints`（`optimizer.cpp` 的 `xt::clip(..., s.constraints.vx_min, s.constraints.vx_max)`）。两者只在 `getParams()` 里同步过一次，参数变更挂的 post callback 是 `reset()`，**它不做这个拷贝**。而且 `ParametersHandler::dynamicParamsCallback` 无论如何都 `return successful=true`，连报错都没有 | 改完参数**再往 `/speed_limit` 发一条 `nav2_msgs/SpeedLimit{speed_limit:0.0}`**（0.0=NO_SPEED_LIMIT）——`Optimizer::setSpeedLimit` 会把 `base_constraints` 抄进 `constraints`。webapp 的 `nav_speed` 已内置。不必改 nav2 源码 |
| 膨胀半径/衰减系数改了倒是生效 | `InflationLayer::dynamicParametersCallback` 和 MPPI 的 `ObstaclesCritic` 都是正常的动态参数（会 `need_cache_recompute` + 重新膨胀） | 无需处理。**只有速度那三个是坏的** |
| 网页上选了地图但画布没反应 | 选中只写了 `localStorage`，没人去加载那张图 | 已加 `preview_map`：选中即把该图推给前端当底图。导航运行中会拒绝覆盖（那时画布上是正在用的实时 `/map`）并回一条说明 |
| 删了地图后首页/地图库还显示那个名字，启动导航还报找不到 | `UI.selMap` 存在 localStorage 里，图删了它不会自己消失 | 已改：每次收到 `maps` 列表都核对一遍 `selMap`，对不上就清空并提示 |
| 导航中首页"当前地图"显示的不是正在用的那张 | `active_map`(启动时锁定) 和 `selMap`(下次启动用哪张) 是两回事，之前 `selMap` 优先 | 已加 `mapNameNow()`：运行中 `active_map` 优先，空闲时才回落到 `selMap` |
| 刚设好目标点，图标立刻消失 / 面板"目标点"一直是"未设置" | 前端每收到一条 `nav` 就 `if(终态) goal=null`，而后端在新目标被 action server 接受**之前**会先补发一条带着上一次终态的 `nav` | 已改：目标点一直留着，只在它身上标 `done`（绿=已到达/红=失败），真正清除只发生在取消和设定下一个目标时 |
| 已经到了，"剩余距离"还挂着一个数 | `nav={...nav,...m}` 合并上来，终态那条消息不带 `dist`，上一条 running 的值就留着了 | 已改：进终态时显式 `nav.dist=null` |
| 导航面板里"当前位置/当前朝向"是死的 | 那个每秒刷新的定时器加了 `if(navT0)` 条件，没在导航时根本不跑；位姿回调也没调 `renderPanel` | 已改：位姿回调调用轻量的 `updatePanelLive()`（只改几个数字，不重建面板，否则航点列表会闪、按钮会掉焦点） |

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
ros2 launch wheeltec_fastlio mapping.launch.py     # 建图(完后 Ctrl+C 存PCD; 默认屏蔽正后方90°)
ros2 launch wheeltec_fastlio mapping.launch.py blind_back_deg:=0    # 建图(不屏蔽后方)
ros2 launch wheeltec_fastlio save_map.launch.py    # PCD → 2D导航地图
ros2 launch wheeltec_fastlio navigation.launch.py  # 导航(自动重定位; 相机不参与避障)
ros2 launch wheeltec_fastlio navigation.launch.py use_camera:=false      # 导航(纯雷达, 相机故障时用)
ros2 launch wheeltec_fastlio navigation.launch.py camera_z:=0.22 camera_pitch:=0.09  # 覆盖相机安装参数
ros2 launch wheeltec_fastlio navigation.launch.py camera_avoid:=true    # 重新启用相机避障(还需改param的observation_sources)
ros2 launch wheeltec_fastlio lidar_test.launch.py  # 雷达单独测试
ros2 service call /relocalize std_srvs/srv/Trigger # 手动重定位
~/stop_nav.sh                                      # 关不掉时强停+清理
free -h && df -h /                                 # 内存/磁盘巡检
cat /sys/class/thermal/thermal_zone0/temp          # 检查 CPU 温度（>85000 则降频）
```

更多细节见 `wheeltec_fastlio/README.md`（随包拷贝）。
