#!/bin/bash
# 户外建图和导航可视化完整指南

cat << 'EOF'
╔══════════════════════════════════════════════════════════════════╗
║                                                                  ║
║          户外建图和导航 - 可视化完整指南                         ║
║                                                                  ║
╚══════════════════════════════════════════════════════════════════╝

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

【场景 1】户外建图 + 可视化

步骤 1：启动建图系统
-----------------------------------------------
终端 1（主系统）：
  cd ~/wheeltec_ros2
  source install/setup.bash
  ros2 launch wheeltec_outdoor_nav outdoor_mapping.launch.py

  等待看到：
    ✓ [livox_ros_driver2]: Livox-SDK2 init success!
    ✓ [轨迹记录节点已启动]
    ✓ [记录统计 - 雷达: XXX, GPS: XXX]

步骤 2：启动 RViz 可视化
-----------------------------------------------
终端 2（可视化）：
  cd ~/wheeltec_ros2
  source install/setup.bash
  rviz2

  在 RViz 中配置：
  1. Fixed Frame 设置为 "camera_init"

  2. 添加显示项（左下角 Add 按钮）：
     - PointCloud2
       * Topic: /cloud_registered
       * Size: 0.05
       * Color: Intensity 或 RGB8
       说明：实时点云地图

     - Path
       * Topic: /Odometry（需要转换）
       说明：雷达里程计轨迹

     - TF
       * 显示坐标系关系

     - Marker (可选)
       * Topic: /gps_marker (如果有)
       说明：GPS 轨迹标记

  3. 保存配置：
     File -> Save Config As
     保存到：~/wheeltec_ros2/outdoor_mapping.rviz

步骤 3：遥控车建图
-----------------------------------------------
终端 3（控制）：
  cd ~/wheeltec_ros2
  source install/setup.bash
  ros2 run wheeltec_robot_keyboard wheeltec_keyboard

  控制说明：
    W - 前进
    S - 后退
    A - 左转
    D - 右转
    空格 - 停止

  建图技巧：
    ✓ 慢速移动（0.3-0.5 m/s）
    ✓ 避免快速转弯
    ✓ 尽量形成闭环
    ✓ 持续 5-10 分钟
    ✓ 覆盖目标区域

步骤 4：监控建图状态
-----------------------------------------------
终端 4（监控，可选）：
  # 查看雷达数据
  ros2 topic hz /livox/lidar

  # 查看 GPS 数据
  ros2 topic echo /gps/fix --once

  # 查看轨迹记录统计
  # （在终端1可以看到每5秒的统计输出）

步骤 5：停止建图
-----------------------------------------------
  在终端 1 按 Ctrl+C 停止

  自动保存位置：
    - 点云地图：~/wheeltec_ros2/src/FAST_LIO/PCD/scans_*.pcd
    - GPS轨迹：~/wheeltec_ros2/outdoor_maps/trajectory_XXXXXX.csv

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

【场景 2】转换为 2D 地图 + 可视化

步骤 1：转换点云为 2D 地图
-----------------------------------------------
终端 1：
  cd ~/wheeltec_ros2
  source install/setup.bash
  ros2 launch wheeltec_fastlio save_map.launch.py

  等待转换完成（约 1-2 分钟）

  输出文件：
    ~/wheeltec_ros2/src/wheeltec_robot_nav2/map/WHEELTEC3D.pgm
    ~/wheeltec_ros2/src/wheeltec_robot_nav2/map/WHEELTEC3D.yaml

步骤 2：查看生成的地图
-----------------------------------------------
方式 1 - RViz 查看：
  rviz2
  添加 Map 显示：
    * Topic: /map
    * Color Scheme: map

方式 2 - 图片查看器：
  eog ~/wheeltec_ros2/src/wheeltec_robot_nav2/map/WHEELTEC3D.pgm

步骤 3：计算 GPS-地图对齐
-----------------------------------------------
终端 1：
  cd ~/wheeltec_ros2/outdoor_maps/
  ros2 run wheeltec_outdoor_nav align_gps_to_map.py

  查看输出：
    ✓ 对齐完成，RMS误差: 0.XX m（应 < 1m）
    ✓ 已保存: gps_map_calibration.yaml

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

【场景 3】户外导航 + 完整可视化

步骤 1：启动导航系统
-----------------------------------------------
终端 1（主系统）：
  cd ~/wheeltec_ros2
  source install/setup.bash
  ros2 launch wheeltec_outdoor_nav outdoor_navigation.launch.py

  等待看到：
    ✓ [amcl]: Initializing
    ✓ [gps_localization_guard]: GPS定位守卫已启动
    ✓ [controller_server]: Created controller

步骤 2：启动 RViz 导航可视化
-----------------------------------------------
终端 2（可视化）：
  cd ~/wheeltec_ros2
  source install/setup.bash

  # 使用 Nav2 默认配置
  ros2 launch nav2_bringup rviz_launch.py

  或手动配置 RViz：
  rviz2

  配置项：
  1. Fixed Frame: "map"

  2. 必须添加的显示：
     ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     【地图层】
     - Map (Topic: /map)
       * 显示静态地图

     - Map (Topic: /global_costmap/costmap)
       * 全局代价地图，Color Scheme: costmap

     - Map (Topic: /local_costmap/costmap)
       * 局部代价地图，Color Scheme: costmap

     ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     【定位层】
     - PoseArray (Topic: /particle_cloud)
       * AMCL 粒子云，显示定位置信度
       * Shape: Arrow, Color: 绿色

     - PoseWithCovariance (Topic: /amcl_pose)
       * AMCL 估计位姿，显示协方差椭圆
       * Color: 蓝色

     - PoseStamped (Topic: /gps/pose) [可选]
       * GPS 位置对比
       * Color: 红色

     ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     【传感器层】
     - LaserScan (Topic: /scan)
       * 2D 激光扫描，Size: 0.05
       * Color: 白色或按强度

     - PointCloud2 (Topic: /livox/lidar) [可选]
       * 3D 点云，Size: 0.03
       * Color: RGB8

     ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     【路径规划层】
     - Path (Topic: /plan)
       * 全局路径规划，Color: 黄色

     - Path (Topic: /local_plan)
       * 局部路径规划，Color: 绿色

     - Path (Topic: /received_global_plan)
       * 接收到的全局路径，Color: 橙色

     ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     【机器人模型】
     - RobotModel (TF: base_link)
       * 显示机器人模型

     - TF
       * 显示所有坐标系
       * Frames: 只显示重要的（map, odom, base_link）

     ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     【导航目标】
     - PoseStamped (Topic: /goal_pose)
       * 导航目标点，Color: 红色

  3. 工具栏：
     - 2D Pose Estimate：设置初始位姿（如果定位不准）
     - 2D Goal Pose：发送导航目标点

  4. 保存配置：
     File -> Save Config As
     保存到：~/wheeltec_ros2/outdoor_navigation.rviz

步骤 3：设置初始位姿（如果需要）
-----------------------------------------------
  在 RViz 中：
  1. 点击工具栏的 "2D Pose Estimate"
  2. 在地图上点击机器人当前位置
  3. 拖动设置朝向

  观察粒子云是否收敛到机器人周围

步骤 4：下发导航目标
-----------------------------------------------
  在 RViz 中：
  1. 点击工具栏的 "2D Goal Pose"
  2. 在地图上点击目标位置
  3. 拖动设置目标朝向

  观察：
    ✓ 黄色线：全局路径规划
    ✓ 绿色线：局部路径规划
    ✓ 机器人开始移动

步骤 5：监控导航状态
-----------------------------------------------
终端 3（监控）：
  # 查看定位质量
  ros2 topic echo /amcl_pose | grep covariance -A 6

  # 查看 GPS 守卫状态（日志会自动输出）
  # 正常：GPS-AMCL偏差: X.XX m
  # 异常：定位丢失！使用RTK重置AMCL

  # 查看导航状态
  ros2 topic echo /navigate_to_pose/_action/status

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

【常见可视化问题】

问题 1：RViz 中看不到点云
-----------------------------------------------
解决：
  1. 检查 Fixed Frame 是否正确（建图时用 camera_init）
  2. 检查 Topic 名称是否正确
  3. 运行：ros2 topic list | grep cloud

问题 2：AMCL 粒子云分散，定位不准
-----------------------------------------------
解决：
  1. 使用 "2D Pose Estimate" 手动设置初始位姿
  2. 让车原地旋转一圈，帮助 AMCL 收敛
  3. 检查激光扫描是否有数据：ros2 topic echo /scan

问题 3：导航路径规划失败
-----------------------------------------------
解决：
  1. 检查目标点是否在地图范围内
  2. 检查目标点是否在障碍物上
  3. 查看 costmap 是否正常（终端1的日志）

问题 4：RViz 卡顿
-----------------------------------------------
优化：
  1. 降低点云显示数量（Size 改大，或不显示）
  2. 关闭不必要的 TF 显示
  3. 降低 LaserScan 的显示密度

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

【高级可视化】可选功能

1. 记录和回放
-----------------------------------------------
  # 记录建图过程
  ros2 bag record -a -o outdoor_mapping

  # 回放
  ros2 bag play outdoor_mapping

2. PlotJuggler 数据可视化
-----------------------------------------------
  # 安装
  sudo apt install ros-humble-plotjuggler-ros

  # 启动
  ros2 run plotjuggler plotjuggler

  # 可视化数据：
  - /amcl_pose 的协方差变化
  - /gps/fix 的精度变化
  - 机器人速度曲线

3. Web 可视化（Foxglove）
-----------------------------------------------
  # 安装
  sudo apt install ros-humble-foxglove-bridge

  # 启动
  ros2 launch foxglove_bridge foxglove_bridge_launch.xml

  # 浏览器访问
  https://studio.foxglove.dev

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

【推荐工作流程】

完整的户外建图导航流程：
1. 建图（30分钟）
   - 启动系统 + RViz
   - 遥控车走一圈
   - 停止保存

2. 地图处理（10分钟）
   - 转换为 2D 地图
   - 计算 GPS 对齐
   - 检查地图质量

3. 导航测试（1小时）
   - 启动导航 + RViz
   - 设置初始位姿
   - 下发目标点测试
   - 观察定位和路径规划

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

现在就可以开始户外建图了！祝测试顺利！🚀

EOF
