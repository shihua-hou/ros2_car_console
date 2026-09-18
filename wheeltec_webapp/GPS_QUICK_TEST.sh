#!/bin/bash
# GPS 可视化功能快速测试脚本

echo "=========================================="
echo "  Web App v2 - GPS 可视化功能测试"
echo "=========================================="
echo ""

# 1. 检查 GPS 驱动
echo "[1/4] 检查 GPS 数据..."
if ros2 topic list 2>/dev/null | grep -q "/gps/fix"; then
    echo "  ✓ GPS 话题存在"
    timeout 2 ros2 topic echo /gps/fix --once 2>/dev/null | grep -q "latitude" && echo "  ✓ GPS 有数据" || echo "  ✗ GPS 无数据"
else
    echo "  ✗ GPS 话题不存在，请先启动 GPS 驱动："
    echo "    ros2 launch wheeltec_gps_driver wheeltec_dual_rtk_driver_nmea.launch.py"
fi

echo ""

# 2. 检查 Web App
echo "[2/4] 检查 Web App..."
if pgrep -f "wheeltec_webapp" > /dev/null; then
    echo "  ✓ Web App 已运行"
    PORT=$(netstat -tlnp 2>/dev/null | grep python3 | grep -oP ':\K[0-9]+' | head -1)
    echo "  ✓ 端口: ${PORT:-8080}"
else
    echo "  ✗ Web App 未运行，启动命令："
    echo "    ros2 run wheeltec_webapp web"
fi

echo ""

# 3. 获取 IP 地址
echo "[3/4] 获取访问地址..."
IP=$(hostname -I | awk '{print $1}')
echo "  ✓ 机器人 IP: $IP"
echo "  ✓ Web 地址: http://$IP:8080/v2"

echo ""

# 4. 功能说明
echo "[4/4] 功能说明"
echo "----------------------------------------"
echo "首页（仪表盘）："
echo "  - 查看底部状态芯片区域"
echo "  - GPS 状态显示（绿色=RTK固定解）"
echo ""
echo "导航页面："
echo "  - 右上角显示高德地图窗口"
echo "  - 蓝色圆点为机器人位置"
echo "  - 显示 GPS 状态和坐标"
echo ""
echo "建图页面："
echo "  - 右上角显示高德地图窗口"
echo "  - 实时显示建图时的 GPS 轨迹"
echo "----------------------------------------"

echo ""
echo "=========================================="
echo "测试完成！请在浏览器打开上述地址"
echo "=========================================="
