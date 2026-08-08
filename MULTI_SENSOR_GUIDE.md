# 多传感器建图导航指南（MID360s / N10Plus / Odin1 三选一）

> ## ⚠️ 2026-08-05 起本文档已归档，不再维护
>
> 本车现在只用 **MID360s + Astra 深度相机** 这一套：车上只有 MID360s 一颗雷达，
> 不再做三选一切换。`wheeltec_n10plus` / `wheeltec_odin1` / `odin_ros_driver` /
> `wheeltec_lidar_ros2` 已全部打上 `COLCON_IGNORE`（代码留着，不参与编译），
> 网页控制台的三段式传感器选择器也只剩 MID360s 一项。
>
> 想恢复：删掉这些包里的 `COLCON_IGNORE`，把 `static/index.html` 里
> `sensorSel` 的 option 加回来、`curSensor` 恢复成读 localStorage 即可
> —— 后端 `app.py` 的 sensor 分支逻辑一行都没删。
>
> 相机接入导航的部分见 `wheeltec_fastlio/README.md` 与根目录 `CLAUDE.md`。
>
> 以下为归档内容 ↓

> 本文档写于 2026-07-22，N10Plus 和 Odin1 硬件均未到货，以下 N10Plus/Odin1
> 部分是**按官方文档/源码推断的最佳准备**，编译已验证通过，但从未接实际
> 硬件跑过。硬件到货后请按文中"待验证"清单逐项确认，出问题的概率不小，
> 属于正常情况，不代表配置整体思路错了。

## 三种模式如何选择

三套是完全独立的 launch 包，**互不冲突但也不共享定位**——同一时间只能跑一种。
选择方式就是"跑哪个包的 launch"，不是配置文件里的一个开关：

| 模式 | 包 | 建图 | 导航 |
|------|-----|------|------|
| MID360s + FAST-LIO（原有，已跑通） | `wheeltec_fastlio` | `mapping.launch.py` | `navigation.launch.py` |
| N10Plus 2D激光 + slam_toolbox（新） | `wheeltec_n10plus` | `mapping.launch.py` | `navigation.launch.py` |
| Odin1 空间记忆模组（新，自带SLAM+重定位） | `wheeltec_odin1` | `mapping.launch.py` | `navigation.launch.py` |

三者共用同一套底盘串口/球形防重复启动机制——`_abort_if_already_running()`
的进程特征列表已经把三家的关键进程名都加了进去，同时启动两套会被拒绝。

**网页控制台（`wheeltec_webapp`）已支持三选一**：任务卡片顶部有 MID360s/
N10Plus/Odin1 三段式选择器，忙碌时锁定并跟随实际运行的传感器。N10Plus/Odin1
没有独立的"保存地图"步骤，必须在**建图运行中**点保存（N10Plus走
`map_saver_cli`，Odin1写命令文件触发SDK），跟MID360"先停止建图再单独转换"
的流程不一样，网页按钮/提示已按此适配。改了 `wheeltec_webapp` 的代码后要
`sudo systemctl restart wheeltec-webapp` 才生效。

---

## N10Plus（镭神智能 2D激光雷达）—— 硬件已到货，基础验证已通过 ✅

### 已完成
- 驱动包 `lslidar_driver`/`lslidar_msgs`（随 `wheeltec_lidar_ros2` 一起拷贝，
  官方型号列表明确支持 N10Plus）已编译通过
- `wheeltec_n10plus` 包：`config/n10plus.yaml`（雷达参数）、
  `config/mapper_params_online_async.yaml`（slam_toolbox建图参数）、
  `launch/mapping.launch.py`、`launch/navigation.launch.py`
- 导航阶段复用现有 `wheeltec_nav2` 框架 + `wheeltec_fastlio` 的
  `auto_relocalize` 扫描匹配重定位节点（它只依赖通用的 `/scan`+`/map`，
  跟地图是怎么建出来的无关，两种模式可以共用）
- **2026-07-22 实机验证通过**：串口版插上后**不需要自己写udev规则**——
  系统原厂自带的 `wheeltec_lidar3.rules`（CH9102, `idVendor=1a86 idProduct=55d4
  serial=0001`）会自动生成 `/dev/wheeltec_lidar`（底盘控制器是同一芯片但
  `serial=0002`，靠serial区分不冲突）。`n10plus.yaml` 已把 `serial_port` 改成
  `/dev/wheeltec_lidar`，`wheeltec_n10plus/udev/n10plus.rules` 已不需要
  （留着仅作参考）。实测 `/scan` 稳定 10Hz、5400点/圈、有效测距点数据正常
  （示例读数 4.4~5.3m，符合真实房间尺度），驱动+配置本身没有问题。
- **踩过一个硬件坑（不是软件问题）**：第一次插上时 dmesg 显示反复
  连接→断开循环（14次/几十秒），是 USB 供电不稳/接触不良，换个口/重新插紧
  后正常了。以后如果雷达又开始"时有时无"，先怀疑这个，不是配置文件的问题。

### 还需要做的事
1. ~~udev 规则~~ 已确认不需要，见上。
2. **测距范围**：`config/n10plus.yaml` 里 `max_range: 10.0`，实测中在10m内的
   数据看起来正常（读到过7.58m的有效点），暂不需要改，后续如果发现近距离
   噪点多再收紧。
3. ~~安装偏移~~ 已实测回填：轴心距车头10cm，雷达在车头内侧缩进6cm→轴心前方
   4cm(`lidar_x=0.04`)；离地20cm(`lidar_z=0.2`)。`lidar_y`/`lidar_yaw` 假设居中
   正向未测(=0)，如果雷达没有严格装在车体中轴线上或有转向角度需要另外核实。
3.5. **屏蔽车正后方90°**（人跟车后面推着建图，不想把自己扫进地图）：
   `config/n10plus.yaml` 里 `angle_disable_min: [13500]` / `angle_disable_max: [22500]`
   （135°~225°，单位0.01°）。读 `lslidar_x10_driver.cpp` 源码确认角度约定:
   0°=正前方, 90°=左, 180°=正后方, 逆时针为正; **前提是雷达实际朝向和
   `lidar_yaw=0`(正前方)一致**——如果雷达装车时有偏转角度, 这两个数字要跟着
   同样偏转量调整, 否则屏蔽的不是真后方。已用真机验证: 屏蔽区间内有效点从
   基线9.8%降到0.1%, 其余方向不受影响。不想屏蔽时改回 `[0]`/`[0]`。
4. ~~验证雷达能出数据~~ 已验证通过。**注意**：`lslidar_driver` 自带的
   `lslidar_x10_launch.py` 里 `driver_config` 路径是硬编码的，不接受
   `params_file:=` 这种launch参数覆盖，会一直加载它自己包内默认的
   `lslidar_x10.yaml`（型号是M10P，不是N10Plus）。实测验证用的是绕开这个坑的
   命令，直接指定我们自己的配置：
   ```bash
   ros2 run lslidar_driver lslidar_driver_node --ros-args -r __ns:=/x10 \
     --params-file ~/wheeltec_ros2/install/wheeltec_n10plus/share/wheeltec_n10plus/config/n10plus.yaml
   ros2 topic hz /scan   # 实测: 稳定10Hz
   ```

### ✅ 2026-07-22 建图+导航全流程实测通过
- 建图：走一圈后地图 211x315px@0.05m(~10.5x15.8m)，自由/障碍/未知比例
  29.6%/3.6%/66.8%，合理
- 导航：重定位成功 `score=0.95`（超过0.9验收线），定位健康度0.82，
  `map->base_footprint` TF正确解析
- **踩坑&已修复**：`auto_relocalize.cpp` 里重定位/健康检查固定只抽样
  60/120/180个激光点打分，这个数字是按 MID360 转换出的稠密`/scan`
  (有效率接近100%)调的。N10Plus实测有效点率只有~9.6%(517/5400)，抽60个点
  只有约6个有效，报"有效激光束过少无法重定位"。已把三处抽样数改成
  400/600/900（对MID360几乎无额外开销）并重新编译 `wheeltec_fastlio`，
  之后N10Plus重定位/健康检查都正常了。**这个修复是通用的，MID360也受益**。
- 待办：还没实际发送导航目标点测试小车真的动起来走到目标点（重定位/TF/
  costmap都验证过了，但controller/planner的真实寻路行为还没测）

### 使用
```bash
ros2 launch wheeltec_n10plus mapping.launch.py      # 建图, Ctrl+C退出
ros2 run nav2_map_server map_saver_cli -f ~/wheeltec_ros2/src/wheeltec_robot_nav2/map/N10PLUS_MAP \
  --ros-args -p save_map_timeout:=10.0
# 注意: 默认save_map_timeout太短(~2s), 跟slam_toolbox的map_update_interval(2.0s)
# 节奏对不上容易"Failed to spin map subscription"存图失败, 必须显式加长超时。
# 存到 install/wheeltec_nav2/map 和 src/wheeltec_robot_nav2/map 需手动同步一份(参照MID360的save_map习惯)
ros2 launch wheeltec_n10plus navigation.launch.py   # 导航
```

---

## Odin1（留形科技 空间记忆模组）

这是三者里**不确定性最大**的一个，因为它是全新的外部黑盒硬件，不是简单换个雷达。

### 已完成
- 从 `github.com/manifoldsdk/odin_ros_driver` 克隆并编译成功
  （`host_sdk_sample`/`pcd2depth_ros2_node`/`cloud_reprojection_ros2_node`/
  `image_overlay_node` 四个可执行文件都装好了）
- **⚠️ 排雷记录**：该仓库自带的 `script/build_ros2.sh` 默认动作会
  `rm -rf` 整个工作空间的 `build/install/log`（跟 `livox_ros_driver2/build.sh`
  是同一类陷阱，CLAUDE.md 已经警告过livox那个，这是第二个类似的坑）——
  **千万不要直接跑这个脚本**，改用普通的
  `colcon build --packages-select odin_ros_driver` 就行，本项目就是这么编译的。
- `wheeltec_odin1` 包：udev规则、`control_command_mapping.yaml`
  (`custom_map_mode: 1` SLAM建图)、`control_command_navigation.yaml`
  (`custom_map_mode: 2` 重定位)、`param_odin1.yaml`（独立的Nav2参数）、
  `mapping.launch.py`、`navigation.launch.py`
- **架构要点**：Odin1 自己出 `map->odom->odin1_base_link` 定位（读了它的
  `include/host_sdk_sample.h` 源码确认），所以导航模式**不跑 AMCL、不跑
  nav2_map_server、不跑 auto_relocalize**——那些都是"外部提供地图+扫描匹配"
  的定位方式，跟 Odin1 自带定位同时跑会在 map/odom 帧上打架（TF 单亲冲突）。
  底盘的轮式 EKF 同理也不跑（它默认会广播 odom_combined→base_footprint，
  一样会冲突），只保留 `base_serial`(电机驱动) + `joint_state_publisher`。

### 已知限制（当前设计的妥协，不是bug）
1. **没有真正的全局静态地图**：Odin1 不像 FAST-LIO/N10Plus 那样能直接产出
   2D occupancy grid，`param_odin1.yaml` 的 global_costmap 去掉了
   `static_layer`，改用 20x20m 的 `rolling_window`（类似放大版的局部代价地图）。
   意味着全局路径规划范围有限，不是真正"提前知道整栋楼布局"的规划。
   如果要做真正的静态地图，思路是：建图时把 `/odin1/cloud_slam` 落盘成 pcd，
   再复用 `wheeltec_fastlio` 现成的 `pcd2pgm` 工具转成 2D 地图——这部分
   **还没做**，是后续工作。
2. **TF树方向的不确定性**：读源码发现 Odin1 内部广播 TF 时，`odom` 同时是
   `odin1_base_link` 和 `map` 两者的父节点（不是标准 REP105 的
   `map→odom→base_link` 链式顺序，而是 `odom` 一个节点带两个子节点）。
   tf2 查询任意两帧间的相对变换本身不要求 map 一定是树根，理论上
   costmap/controller 依然能正常算出 `map→base_footprint`，但如果它的
   重定位/闭环修正是打在 `odom→map` 这条支路上，可能表现为"map这个全局
   参考系本身发生跳变"而不是"机器人在map里的位置被修正"——**这一点必须
   等硬件到手后实测确认**，命令：
   ```bash
   ros2 launch wheeltec_odin1 navigation.launch.py
   # 另开终端:
   ros2 run tf2_tools view_frames        # 生成TF树图, 看实际父子关系
   ros2 topic echo /tf --once             # 看实际帧名
   ```
   如果确认真的会导致 map 帧跳变，需要另写一个小的 TF 桥接节点纠正
   （监听 odom↔map 的实际变换，重新发布成规范的 map→odom_bridge 形式）。

### 硬件到货后必须做的事
1. **udev 规则**（这个来自官方文档，置信度较高，不是瞎猜）：
   ```bash
   sudo cp ~/wheeltec_ros2/src/wheeltec_odin1/udev/odin1.rules /etc/udev/rules.d/99-odin-usb.rules
   sudo udevadm control --reload && sudo udevadm trigger
   lsusb   # 应能看到 ID 2207:0019 Fuzhou Rockchip Electronics Company hawk
   ```
2. **USB 3.0**：官方文档强调 SLAM 建图功能需要 USB 3.0 口，确认鲁班猫4接的
   是 USB3 口（一般蓝色触点那个）。
3. **安装偏移**：`navigation.launch.py` 的 `lidar_x/y/z/yaw`（这里其实是
   odin1_base_link 相对 base_footprint 的偏移）目前占位 0，需要实测回填，
   且必须与 mapping 阶段的假设一致（mapping模式不涉及Nav2, 不需要这个值,
   但建图轨迹的物理尺度依赖传感器实际安装位置的一致性）。
4. **先独立验证驱动能跑通**，参考它自己的 `RELOCALIZATION_GUIDE.md`：
   ```bash
   source ~/wheeltec_ros2/install/setup.bash
   ros2 run odin_ros_driver host_sdk_sample --ros-args -p config_file:=~/wheeltec_ros2/src/wheeltec_odin1/config/control_command_mapping.yaml
   ros2 topic list   # 应能看到 /odin1/odometry /odin1/cloud_raw 等
   ```
5. **建图后存图的具体交互**：官方给的是一个 `set_param.sh` 脚本
   （`odin_ros_driver/set_param.sh save_map 1`），具体怎么触发建议到货后
   先读一遍该包的 `RELOCALIZATION_GUIDE.md` 原文再操作，本文档没有把这部分
   写死进 launch 文件（因为存图是一次性动作，不适合塞进常驻 launch）。

### 使用（预期流程，未实测）
```bash
ros2 launch wheeltec_odin1 mapping.launch.py
# 推小车绕场地走一圈, 尽量形成闭环
cd ~/wheeltec_ros2/src/odin_ros_driver && ./set_param.sh save_map 1
# 确认 wheeltec_odin1/map/odin1_map.bin 生成
ros2 launch wheeltec_odin1 navigation.launch.py
```

---

## 依赖安装踩坑记录

- **`ros-humble-*` 系列 apt 包在本网络下载不了**：`packages.ros.org` 被本地
  网络策略（"浩鸿科技上网基线策略"）拦截替换成了假响应（文件大小对不上），
  Aliyun/USTC 的 ROS2 镜像是通的。已经把 `/etc/apt/sources.list.d/ros2.list`
  改成了 `https://mirrors.aliyun.com/ros2/ubuntu`。以后如果新装
  `ros-humble-*` 包又下载失败，先看这个文件有没有被什么操作改回官方源。
- N10Plus 需要 `ros-humble-slam-toolbox`；Odin1 需要 `libusb-1.0-0-dev`
  （其余依赖 OpenCV/Eigen3/yaml-cpp/OpenSSL/PCL 本机已装好）。
