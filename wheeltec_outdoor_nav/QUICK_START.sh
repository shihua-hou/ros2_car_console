#!/bin/bash
# RTK 户外定位系统 - 快速启动指南

cat << 'EOF'
╔════════════════════════════════════════════════════════════════╗
║   RTK 辅助的户外定位系统 - 已就绪！                            ║
╚════════════════════════════════════════════════════════════════╝

✅ 已完成项：
  - 功能包已编译
  - Python 依赖已安装 (pyproj, tf_transformations)
  - 输出目录已创建

⚠️  还需要你做的（首次使用）：

1. 配置 RTK 设备的 udev 规则
   cd ~/wheeltec_ros2/src/wheeltec_gps
   sudo bash wheeltec_gnss.sh
   然后重新插拔 RTK 的 USB 线
   验证: ls -l /dev/wheeltec_gnss

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

📖 使用流程：

【测试 RTK 数据】(可选，验证硬件)
  ros2 launch wheeltec_gps_driver wheeltec_dual_rtk_driver_nmea.launch.py
  # 另开终端
  ros2 topic echo /gps/fix

【户外建图】
  ros2 launch wheeltec_outdoor_nav outdoor_mapping.launch.py
  # 遥控车走一圈（5-10分钟），Ctrl+C 停止

【转换地图】
  ros2 launch wheeltec_fastlio save_map.launch.py

【计算 GPS-地图对齐】(如果建图时有GPS数据)
  cd ~/wheeltec_ros2/outdoor_maps/
  ros2 run wheeltec_outdoor_nav align_gps_to_map.py

【户外导航】
  ros2 launch wheeltec_outdoor_nav outdoor_navigation.launch.py

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

🔧 特殊场景：

【纯雷达建图（无GPS）】
  ros2 launch wheeltec_outdoor_nav outdoor_mapping.launch.py enable_gps:=false

【禁用 GPS 守卫（纯AMCL导航）】
  ros2 launch wheeltec_outdoor_nav outdoor_navigation.launch.py enable_gps_guard:=false

【室内导航（完全不变）】
  ros2 launch wheeltec_fastlio navigation.launch.py

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

📚 文档位置：
  - 使用手册: ~/wheeltec_ros2/src/wheeltec_outdoor_nav/README.md
  - 实施报告: ~/wheeltec_ros2/src/wheeltec_outdoor_nav/IMPLEMENTATION_REPORT.md

🔍 诊断工具：
  bash ~/wheeltec_ros2/src/wheeltec_outdoor_nav/scripts/test_rtk_system.sh

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

🎯 核心特性：
  ✓ RTK 辅助定位（GPS 不可用时自动降级）
  ✓ 定位丢失救援（3秒内恢复）
  ✓ 完全解耦设计（拔掉 RTK 不影响系统）
  ✓ 优雅降级机制（多层后备方案）

准备好开始测试了！🚀

EOF
