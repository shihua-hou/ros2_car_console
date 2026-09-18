#!/bin/bash
# RTK 功能测试脚本
# 用于验证 RTK 模块是否正常工作

echo "=========================================="
echo "RTK 功能诊断工具"
echo "=========================================="
echo ""

# 检查 1: udev 设备
echo "[1/6] 检查 RTK 设备..."
if [ -e "/dev/wheeltec_gnss" ]; then
    echo "  ✓ /dev/wheeltec_gnss 存在"
    ls -l /dev/wheeltec_gnss
else
    echo "  ✗ /dev/wheeltec_gnss 不存在"
    echo "    可用的 USB 串口："
    ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "    无"
    echo ""
    echo "  解决方法："
    echo "    cd ~/wheeltec_ros2/src/wheeltec_gps"
    echo "    sudo bash wheeltec_gnss.sh"
    echo "    重新插拔 USB"
fi
echo ""

# 检查 2: Python 依赖
echo "[2/6] 检查 Python 依赖..."
python3 -c "import pyproj; print('  ✓ pyproj 已安装')" 2>/dev/null || \
    echo "  ✗ pyproj 未安装，运行: pip3 install pyproj"

python3 -c "import yaml; print('  ✓ yaml 已安装')" 2>/dev/null || \
    echo "  ✗ yaml 未安装，运行: pip3 install pyyaml"

python3 -c "import numpy; print('  ✓ numpy 已安装')" 2>/dev/null || \
    echo "  ✗ numpy 未安装，运行: pip3 install numpy"

python3 -c "from tf_transformations import quaternion_from_euler; print('  ✓ tf_transformations 已安装')" 2>/dev/null || \
    echo "  ✗ tf_transformations 未安装，运行: pip3 install transforms3d"
echo ""

# 检查 3: 功能包编译
echo "[3/6] 检查功能包..."
if [ -d "/home/cat/wheeltec_ros2/install/wheeltec_outdoor_nav" ]; then
    echo "  ✓ wheeltec_outdoor_nav 已编译"
else
    echo "  ✗ wheeltec_outdoor_nav 未编译"
    echo "    运行: cd ~/wheeltec_ros2 && colcon build --packages-select wheeltec_outdoor_nav"
fi
echo ""

# 检查 4: GPS 驱动包
echo "[4/6] 检查 GPS 驱动..."
if [ -d "/home/cat/wheeltec_ros2/src/wheeltec_gps/wheeltec_dual_rtk_driver" ]; then
    echo "  ✓ wheeltec_dual_rtk_driver 存在"
else
    echo "  ✗ GPS 驱动包不存在"
fi
echo ""

# 检查 5: 输出目录
echo "[5/6] 检查输出目录..."
OUTDOOR_DIR="$HOME/wheeltec_ros2/outdoor_maps"
if [ -d "$OUTDOOR_DIR" ]; then
    echo "  ✓ $OUTDOOR_DIR 存在"

    # 列出轨迹文件
    TRAJ_COUNT=$(ls "$OUTDOOR_DIR"/trajectory_*.csv 2>/dev/null | wc -l)
    echo "    - 轨迹文件: $TRAJ_COUNT 个"

    # 列出对齐文件
    if [ -f "$OUTDOOR_DIR/gps_map_calibration.yaml" ]; then
        echo "    - GPS对齐参数: 已生成 ✓"
    else
        echo "    - GPS对齐参数: 未生成（建图后运行 align_gps_to_map.py）"
    fi
else
    echo "  - 创建输出目录..."
    mkdir -p "$OUTDOOR_DIR"
    echo "  ✓ 已创建 $OUTDOOR_DIR"
fi
echo ""

# 检查 6: ROS 环境
echo "[6/6] 检查 ROS 环境..."
if [ -f "/home/cat/wheeltec_ros2/install/setup.bash" ]; then
    source /home/cat/wheeltec_ros2/install/setup.bash 2>/dev/null
    if ros2 pkg list | grep -q wheeltec_outdoor_nav; then
        echo "  ✓ ROS 环境正常"
    else
        echo "  ✗ ROS 包未找到，请 source install/setup.bash"
    fi
else
    echo "  ✗ install/setup.bash 不存在"
fi
echo ""

# 总结
echo "=========================================="
echo "诊断完成"
echo "=========================================="
echo ""
echo "下一步："
echo "  1. 如果设备正常，测试 GPS 数据："
echo "     ros2 launch wheeltec_gps_driver wheeltec_dual_rtk_driver_nmea.launch.py"
echo "     ros2 topic echo /gps/fix"
echo ""
echo "  2. 户外建图："
echo "     ros2 launch wheeltec_outdoor_nav outdoor_mapping.launch.py"
echo ""
echo "  3. 计算对齐参数："
echo "     cd ~/wheeltec_ros2/outdoor_maps/"
echo "     ros2 run wheeltec_outdoor_nav align_gps_to_map.py"
echo ""
echo "  4. 户外导航："
echo "     ros2 launch wheeltec_outdoor_nav outdoor_navigation.launch.py"
echo ""
