# RTK 模块配置完成

## ✅ GPS/RTK 已成功配置！

### 设备信息
- **模块型号**：和芯星通 UM982（双天线 RTK）
- **串口设备**：`/dev/wheeltec_gnss` → `/dev/ttyACM4`
- **输出格式**：NMEA 标准格式
- **定位状态**：RTK 固定解（status=2，精度 ±2cm）

### 当前位置
- 纬度：31.977167° N
- 经度：118.749413° E  
- 高度：82.18 m
- 卫星数：19-22 颗

### ROS 话题
```bash
/gps/fix         # GPS 位置（NavSatFix）
/gps/pose        # 位姿信息
/gps/vel         # 速度信息
/gps/nmea/vel    # NMEA 速度
```

---

## 🎯 现在可以进行的测试

### 1. 启动 GPS 驱动测试
```bash
ros2 launch wheeltec_gps_driver wheeltec_dual_rtk_driver_nmea.launch.py

# 另开终端查看数据
ros2 topic echo /gps/fix
```

### 2. 户外建图（带 GPS 记录）
```bash
ros2 launch wheeltec_outdoor_nav outdoor_mapping.launch.py
# 遥控车走一圈
```

### 3. 转换地图
```bash
ros2 launch wheeltec_fastlio save_map.launch.py
```

### 4. 计算 GPS-地图对齐
```bash
cd ~/wheeltec_ros2/outdoor_maps/
ros2 run wheeltec_outdoor_nav align_gps_to_map.py
```

### 5. 户外导航（带 GPS 守卫）
```bash
ros2 launch wheeltec_outdoor_nav outdoor_navigation.launch.py
```

---

## ⚠️ 注意事项

### 串口说明
AirM2M RTK 模块有 4 个 USB 串口：
- `/dev/ttyACM1` - AT 命令口
- `/dev/ttyACM2` - 调试口
- `/dev/ttyACM3` - **二进制数据口**（Unicore 协议）
- `/dev/ttyACM4` - **NMEA 数据口**（✓ 使用这个）

我们使用 **ttyACM4**，它输出标准 NMEA 格式，兼容所有驱动。

### 双天线航向角
当前驱动使用的是 NMEA 标准驱动，可以获取：
- ✅ 位置（纬度、经度、高度）
- ✅ 速度
- ⚠️ 航向角（从 $GNHPR 消息解析，但当前驱动未发布专门话题）

如果需要高精度航向角，可以：
1. 使用 wheeltec 的双 RTK 专用驱动
2. 或修改当前驱动解析 $GNHPR 消息

---

## 📚 相关文档
- 完整使用手册：`~/wheeltec_ros2/src/wheeltec_outdoor_nav/README.md`
- 部署报告：`~/wheeltec_ros2/src/wheeltec_outdoor_nav/DEPLOYMENT_COMPLETE.md`
- 系统诊断：`bash ~/wheeltec_ros2/src/wheeltec_outdoor_nav/scripts/test_rtk_system.sh`

---

**GPS 功能已完全就绪！可以开始户外建图和导航测试了。** 🚀
