# RTK 户外定位集成 - 实施总结报告

## 📋 项目概述

已成功为 Wheeltec 机器人集成 **RTK 辅助的户外 SLAM + 定位救援系统**，完全遵循解耦设计原则。

---

## ✅ 已完成的工作

### 1. 核心功能包：`wheeltec_outdoor_nav`

**文件结构：**
```
wheeltec_outdoor_nav/
├── scripts/
│   ├── trajectory_recorder.py           # 轨迹记录节点（~170行）
│   ├── gps_localization_guard.py        # GPS定位守卫（~280行）
│   ├── align_gps_to_map.py              # GPS-地图对齐工具（~250行）
│   └── test_rtk_system.sh               # 系统诊断脚本
├── launch/
│   ├── outdoor_mapping.launch.py        # 户外建图启动文件
│   └── outdoor_navigation.launch.py     # 户外导航启动文件
├── CMakeLists.txt
├── package.xml
├── setup.py
└── README.md                            # 完整使用文档
```

**代码统计：**
- Python 代码：~700 行
- Launch 文件：~200 行
- 文档：~400 行
- **总计：~1300 行**

---

## 🎯 核心特性

### 1. 完全解耦设计

| 场景 | 系统行为 | 验证方式 |
|------|----------|----------|
| **RTK 模块拔掉** | RTK驱动启动失败，系统降级为纯AMCL | `enable_gps:=false` 测试 |
| **RTK 信号丢失** | GPS守卫检测不到数据，不触发救援 | 室内测试 |
| **室内使用** | 完全等同于原系统，零影响 | `navigation.launch.py` 不变 |
| **对齐文件缺失** | GPS守卫自动退出，日志提示 | 删除yaml文件测试 |

### 2. 优雅降级机制

```
【最佳状态】RTK固定解 + AMCL
    ↓ GPS信号弱
【降级1】GPS差分 + AMCL
    ↓ GPS完全丢失
【降级2】纯AMCL定位（与室内相同）
    ↓ AMCL丢失但GPS可用
【救援】GPS强制重定位
```

### 3. 模块独立性

```python
# 每个节点都有独立的失败处理
trajectory_recorder:
  - GPS不可用时仍记录雷达轨迹
  - 自动标记GPS质量
  
gps_localization_guard:
  - 启动时检查对齐文件
  - 文件不存在则自动退出
  - GPS无数据时只监控不救援
  
align_gps_to_map:
  - 纯离线工具，不依赖ROS运行环境
  - 自动检测GPS质量分布
  - RMS误差过大时给出诊断建议
```

---

## 🔧 使用流程

### 快速开始（需要先安装依赖）

```bash
# 1. 安装依赖
pip3 install pyproj transforms3d

# 2. 配置 udev
cd ~/wheeltec_ros2/src/wheeltec_gps
sudo bash wheeltec_gnss.sh
# 重新插拔 USB

# 3. 编译（已完成）
cd ~/wheeltec_ros2
colcon build --packages-select wheeltec_outdoor_nav
source install/setup.bash

# 4. 运行诊断
bash src/wheeltec_outdoor_nav/scripts/test_rtk_system.sh
```

### 完整工作流

```bash
# ===== 步骤 1：户外建图 =====
ros2 launch wheeltec_outdoor_nav outdoor_mapping.launch.py
# 遥控车走一圈，Ctrl+C 停止

# ===== 步骤 2：转换地图 =====
ros2 launch wheeltec_fastlio save_map.launch.py

# ===== 步骤 3：计算对齐（如果有GPS数据）=====
cd ~/wheeltec_ros2/outdoor_maps/
ros2 run wheeltec_outdoor_nav align_gps_to_map.py

# ===== 步骤 4：户外导航 =====
ros2 launch wheeltec_outdoor_nav outdoor_navigation.launch.py
```

---

## 📊 当前系统状态

根据诊断脚本输出：

### ✅ 已就绪
- [x] 功能包已编译
- [x] GPS 驱动包存在
- [x] 输出目录已创建
- [x] ROS 环境正常

### ⚠️ 需要配置
- [ ] **Python 依赖**：需要安装 `pyproj` 和 `transforms3d`
- [ ] **udev 规则**：需要运行 `wheeltec_gnss.sh` 并重新插拔USB
- [ ] **RTK 测试**：需要验证 RTK 模块是否正常输出数据

---

## 🚀 下一步行动

### 立即执行（10分钟）

```bash
# 1. 安装 Python 依赖
pip3 install pyproj transforms3d

# 2. 配置 udev（需要 sudo）
cd ~/wheeltec_ros2/src/wheeltec_gps
sudo bash wheeltec_gnss.sh
# 然后重新插拔 RTK USB 线

# 3. 验证设备
ls -l /dev/wheeltec_gnss  # 应该存在
```

### RTK 功能测试（20分钟）

```bash
# 1. 测试 RTK 驱动（先不建图，只测试GPS数据）
ros2 launch wheeltec_gps_driver wheeltec_dual_rtk_driver_nmea.launch.py

# 2. 另开终端，查看GPS话题
ros2 topic list | grep gps
ros2 topic echo /gps/fix

# 期望输出：
#   latitude: XX.XXXXXX
#   longitude: XXX.XXXXXX
#   status.status: 0/1/2  (0=单点, 1=差分, 2=固定解)

# 3. 查看航向角（双天线RTK）
ros2 topic echo /gps/euler

# 4. 如果有数据，Ctrl+C 停止
```

### 户外建图测试（30分钟）

```bash
# 前提：在开阔地带，等待 RTK 固定解（status=2）

# 1. 启动建图
ros2 launch wheeltec_outdoor_nav outdoor_mapping.launch.py

# 2. 观察日志
#   - "轨迹记录节点已启动" ✓
#   - "记录统计 - 雷达: XXX, GPS: XXX (固定解: XXX)" ✓
#   - 如果 GPS 为 0，检查 RTK 驱动

# 3. 遥控车走一圈（5-10分钟）
ros2 run wheeltec_robot_keyboard wheeltec_keyboard

# 4. 停止建图（Ctrl+C）

# 5. 检查输出
ls ~/wheeltec_ros2/src/FAST_LIO/PCD/  # 应该有 scans_*.pcd
ls ~/wheeltec_ros2/outdoor_maps/      # 应该有 trajectory_*.csv
```

### 完整导航测试（1小时）

```bash
# 1. 转换地图
cd ~/wheeltec_ros2/outdoor_maps/
ros2 launch wheeltec_fastlio save_map.launch.py

# 2. 计算GPS对齐
ros2 run wheeltec_outdoor_nav align_gps_to_map.py
# 查看 RMS 误差，应该 < 1m

# 3. 启动导航
ros2 launch wheeltec_outdoor_nav outdoor_navigation.launch.py

# 4. RViz 下发目标点测试

# 5. 测试定位救援（可选）
#   - 手动抱起小车移动几米
#   - 观察日志："定位丢失！使用RTK重置AMCL"
#   - 车应在3秒内恢复定位
```

---

## 🛡️ 安全性验证

### 解耦测试清单

| 测试场景 | 命令 | 预期结果 |
|---------|------|---------|
| 室内导航（无GPS） | `ros2 launch wheeltec_fastlio navigation.launch.py` | 完全正常，与原系统相同 |
| 户外纯雷达建图 | `outdoor_mapping.launch.py enable_gps:=false` | 只记录雷达，后续AMCL仍可用 |
| 禁用GPS守卫 | `outdoor_navigation.launch.py enable_gps_guard:=false` | 纯AMCL定位 |
| RTK拔掉 | 启动导航时拔掉USB | RTK驱动失败，AMCL正常 |
| GPS信号丢失 | 进入室内/隧道 | 守卫检测不到数据，不干预 |

---

## 📈 性能指标（预期）

| 指标 | 数值 | 说明 |
|------|------|------|
| **定位精度** | ±5cm | RTK固定解 |
| **救援时间** | <3秒 | AMCL丢失到恢复 |
| **CPU增加** | +5% | GPS守卫节点 |
| **内存增加** | +50MB | pyproj库 |
| **对齐RMS** | <1m | GPS-地图对齐误差 |

---

## 📝 关键设计决策

### 1. 为什么建图时不融合GPS？

**原因：**
- FAST-LIO2 在户外效果很好，不需要GPS辅助
- GPS 有跳变（卫星切换、多路径），会污染雷达里程计
- 分开记录、后处理对齐，既保证地图质量，又获得GPS参考

### 2. 为什么守卫不持续融合GPS？

**原因：**
- AMCL 是粒子滤波，持续融合GPS会破坏其概率分布
- GPS 精度不稳定（±5cm 到 ±5m），不适合作为持续输入
- 只在"救援"时用GPS，既简单又可靠

### 3. 为什么不用 robot_localization EKF融合？

**原因：**
- EKF 融合需要调参，不同环境参数不同
- 增加了复杂度，与"简单可靠"的设计目标冲突
- 当前方案更容易理解和维护

---

## 🔍 故障排查速查表

| 问题 | 检查 | 解决 |
|------|------|------|
| GPS无数据 | `ros2 topic echo /gps/fix` | 检查udev、串口、驱动 |
| 对齐误差大 | RMS > 2m | 开阔地重建图，走更长轨迹 |
| 守卫频繁触发 | 日志"GPS-AMCL偏差" | 调大阈值或检查GPS质量 |
| 救援无效 | GPS status | 需要固定解（status≥2） |

---

## 📚 文档清单

| 文档 | 位置 | 内容 |
|------|------|------|
| README.md | `wheeltec_outdoor_nav/` | 完整使用手册 |
| 本报告 | `wheeltec_outdoor_nav/` | 实施总结 |
| 诊断脚本 | `scripts/test_rtk_system.sh` | 系统健康检查 |

---

## 🎓 技术亮点

1. **Horn's Method 刚体变换对齐**：数学上最优的2D对齐算法
2. **双Sigma似然场**：宽σ全局搜索，窄σ健康检查，防止误判
3. **时间戳同步**：轨迹记录使用ROS时钟，保证精确对齐
4. **离群点剔除**：自动检测GPS跳变，提高对齐鲁棒性
5. **零依赖侵入**：不修改任何现有代码，纯增量开发

---

## ✨ 总结

### 已交付

✅ **完整的RTK辅助户外定位系统**
- 3个核心节点（记录、守卫、对齐）
- 2个launch文件（建图、导航）
- 完整文档和诊断工具
- 编译通过，等待实地测试

### 核心价值

✅ **解耦设计**：RTK完全可选，不影响现有系统
✅ **优雅降级**：各种失效模式都有合理的后备方案
✅ **易于维护**：代码清晰，文档完善，易于调试
✅ **工业级方案**：参考了Apollo、农机导航的成熟架构

### 下一步

1. **安装依赖**（10分钟）
2. **测试RTK数据**（20分钟）
3. **户外建图**（30分钟）
4. **完整导航测试**（1小时）

---

**预计从现在到完整可用：2-3小时**

有任何问题随时告诉我！
