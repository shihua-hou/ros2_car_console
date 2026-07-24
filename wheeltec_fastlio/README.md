# wheeltec_fastlio — MID360 + FAST-LIO2 建图导航

WHEELTEC 小车 (RDK X5 主控, ROS2 Humble) 使用 Livox MID360 3D 激光雷达的建图与导航功能包。

## 系统架构

```
建图:  MID360 --CustomMsg--> FAST-LIO2 --> 3D点云地图(PCD)
                                 |
                          Ctrl+C退出后
                                 v
       pcd2pgm: PCD --高度带投影--> 2D栅格地图 WHEELTEC3D.pgm/yaml

导航:  MID360 --PointCloud2--> pointcloud_to_laserscan --> /scan
       TF: map --(AMCL)--> odom_combined --(EKF轮式+IMU)--> base_footprint
       Nav2 复用 wheeltec_nav2 现有参数(按车型自动选择)
```

相关组件：
- `Livox-SDK2`：已编译安装到 /usr/local（源码在 ~/Livox-SDK2）
- `livox_ros_driver2`：MID360 ROS2 驱动（src/livox_ros_driver2）
- `FAST_LIO`：hku-mars 官方 ROS2 分支（src/FAST_LIO）
- 本包：集成 launch、pcd2pgm 转图工具

## 一、硬件连接与网络配置

1. MID360 通过网线接 RDK X5 的 **eth0** 网口，使用 12V 独立供电。
2. 主机 eth0 已配置静态 IP **192.168.1.5**（NetworkManager 连接名 `livox-mid360`，
   开机自动生效，不影响 wlan0 上网）。
3. MID360s 雷达出厂 IP 为 `192.168.1.1XX`，**XX = 雷达 SN 码最后两位**。
   修改 [config/MID360s_config.json](config/MID360s_config.json) 中 `lidar_configs.ip` 为实际雷达 IP，然后重新编译：
   ```bash
   cd ~/wheeltec_ros2 && colcon build --packages-select wheeltec_fastlio
   ```
4. 验证连通：`ping 192.168.1.1XX`；雷达数据测试：
   ```bash
   ros2 launch wheeltec_fastlio lidar_test.launch.py
   ros2 topic hz /livox/lidar        # 应约10Hz
   ```

## 二、建图

```bash
# 终端1: 启动建图(底盘+MID360+FAST-LIO2)
ros2 launch wheeltec_fastlio mapping.launch.py

# 终端2: 键盘遥控小车走一圈
ros2 run wheeltec_robot_keyboard wheeltec_keyboard
```

- 远程 PC 上开 rviz 观察：Fixed Frame 设为 `camera_init`，添加 `/cloud_registered` 点云。
- 建图过程中 3D 点云每 100 帧（约10秒）自动分段保存到
  `~/wheeltec_ros2/src/FAST_LIO/PCD/scans_N.pcd`，**Ctrl+C 退出**时保存剩余部分。
- **注意**：每次启动 mapping.launch.py 会自动清空 PCD 目录中的旧点云，
  如需保留请提前备份。

生成 2D 导航地图：

```bash
ros2 launch wheeltec_fastlio save_map.launch.py
```

地图 `WHEELTEC3D.pgm/yaml` 保存到 wheeltec_nav2 的 map 目录（install 和 src 各一份）。

### 高度带参数

pcd2pgm 转换时自动 RANSAC 拟合地面并校平（消除雷达安装倾斜影响，日志会打印
拟合倾角和雷达离地高度，后者可填入 launch 的 `lidar_z`）。校平后高度带以**地面为 z=0**：
- `min_z:=0.10`（障碍物带下限，地面上方 0.10m）
- `max_z:=0.45`（障碍物带上限 = 小车可通行高度，与导航 /scan 的 max_height 保持一致）

```bash
ros2 launch wheeltec_fastlio save_map.launch.py min_z:=0.15 max_z:=0.8
```

如需关闭校平（如多层斜坡场景）加 `align_ground:=false`，此时 z 以雷达为原点。

## 三、导航

```bash
ros2 launch wheeltec_fastlio navigation.launch.py
```

- **启动后自动重定位**：`auto_relocalize` 节点用激光扫描与地图做全局匹配，
  自动算出初始位姿发给 AMCL（日志出现 `重定位成功 ... score=0.9x` 即完成），
  无需手动 2D Pose Estimate。
- **绑架检测（默认开启）**：小车被人为搬动/定位丢失后，看门狗用窄高斯严格分
  （σ=0.08，正确位姿 0.6~0.8、被搬动 0.1~0.2）检测当前位姿，连续 2 次 <0.4
  （每 2 秒检查）即自动触发全局重定位，约 5~10 秒恢复。参数在
  navigation.launch.py 的 auto_relocalize 节点里调。
- 也可随时手动触发重定位：
  ```bash
  ros2 service call /relocalize std_srvs/srv/Trigger
  ```
- 自动匹配失败（环境特征太少/大改动，score<0.55 不发布）时仍可在 rviz 用
  `2D Pose Estimate` 手动定位。
- rviz 中用 `Nav2 Goal` 下发导航目标（目标点要落在白色可通行区域）。
- 换地图：`navigation.launch.py map:=/path/to/xx.yaml`
- 环境高度对称（长直走廊）时自动重定位可能匹配到错误的对称位置，
  发现定位错了就手动 2D Pose Estimate 纠正。

## 四、雷达安装位置标定

`mapping.launch.py` 和 `navigation.launch.py` 都有雷达安装位置参数
（雷达相对 base_footprint，单位 m/rad，两处必须一致）：

```bash
ros2 launch wheeltec_fastlio navigation.launch.py lidar_x:=0.1 lidar_z:=0.25 lidar_yaw:=0.0
```

长期使用建议直接修改两个 launch 文件中的默认值。

## 五、Ctrl+C 关不掉怎么办

humble 版 nav2 容器关闭时偶发卡死（已知问题）。Ctrl+C 后 10 秒还没退出就开新终端执行：

```bash
~/stop_nav.sh
```

脚本会强停所有建图/导航进程并清理共享内存残留，之后可直接重新启动。
（平时正常退出仍用 Ctrl+C，等它自己退完最干净。）

## 六、常见问题

| 现象 | 排查 |
|------|------|
| /livox/lidar 无数据 | ping 雷达 IP；确认 json 里雷达 IP 与实际一致；确认 eth0 为 192.168.1.5 |
| FAST-LIO 报 "No point, skip this scan" | 建图 launch 用的是 CustomMsg (xfer_format=1)，确认没有同时跑其它驱动实例 |
| 建图漂移/发散 | 雷达安装要牢固；开机静止几秒再动；避免长走廊快速旋转 |
| 导航时 /scan 为空 | TF base_footprint→livox_frame 是否发布；min_height/max_height 高度带内是否有点 |
| 地图全灰/墙很少 | save_map 的 min_z/max_z 与雷达装高不匹配，见上文高度带说明 |
| save_map 报找不到 scans*.pcd | 建图后必须 Ctrl+C 正常退出才会保存；确认建图确实运行过且时间超过10秒 |
| 导航启动后一直没有 /map（nav2 容器 100% CPU 无日志） | 被强杀进程在 /dev/shm 残留已锁定的 `sem.fastrtps_*` 信号量，新节点死锁。建图/导航 launch 已内置自动清理（仅当无其它 ROS 进程时执行）；手动修复：停止所有 ROS 进程后 `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*` 再重启 |
| PC 的 rviz 里 /scan 不显示（QoS 警告） | LaserScan 显示项 → Topic → Reliability Policy 改为 **Best Effort** |

## 注意事项

- FAST-LIO 的 TF（camera_init→body）与底盘 TF（odom_combined→base_footprint）是两棵独立的树，互不冲突；建图时 rviz Fixed Frame 用 `camera_init`。
- 建图与导航**不能同时**启动（驱动的数据格式不同）。
- `livox_ros_driver2` 不要用其自带 `build.sh` 编译——它会清空整个工作空间的 build/install，用 `colcon build --packages-select livox_ros_driver2 --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble`。
- 驱动已打补丁（`src/comm/pub_handler.cpp`，搜 "WHEELTEC patch"）：强制点云和 IMU 都用主机到达时间打戳。原版驱动中 MID360s 点云包与 IMU 包的 time_type 不一致，点云会用雷达内部时钟导致两路时间戳漂移，FAST-LIO 报 `No Effective Points!` / `IMU and LiDAR not Synced`。若今后部署 PTP 时间同步需恢复原逻辑。升级驱动版本后需重新打此补丁。
