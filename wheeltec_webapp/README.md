# wheeltec_webapp — 小车 Web 控制台（轻量级 web 版 rviz）

在小车上跑一个单端口 Web 服务，电脑 / iPad 浏览器打开 **`http://小车IP:8080`** 即可：

| 功能 | 说明 |
|------|------|
| 虚拟摇杆遥控 | 发布 `/cmd_vel`，0.6s 断流看门狗自动刹车（WiFi 掉线不跑飞），速度可调且服务端硬限幅 |
| 一键建图 / 保存地图 / 导航 / 雷达测试 | 托管 `wheeltec_fastlio` 四个 launch；停止一律 SIGINT 优雅退出（建图靠它保存 PCD），22s 超时才 SIGKILL 并清理 `/dev/shm` |
| 实时画布 | `/map`(自动转PNG)、`/scan`、`/plan`、机器人位姿(TF map→base_footprint，建图时用 `/Odometry`)、建图点云(`/cloud_registered` 按高度着色累积)与轨迹 |
| 点图操作 | 设 Nav2 目标点(`/goal_pose`)、设初始位姿(`/initialpose`)、手动重定位(`/relocalize`)、取消目标、急停 |
| 巡检 | 电压 / 可用内存 / 磁盘 / CPU温度 顶栏常显（4GB 内存机器必看） |
| 外部实例感知 | SSH 手动启动的建图/导航会被识别为"外部实例"，网页可接管停止 |

## 手动运行

```bash
source ~/wheeltec_ros2/install/setup.bash
ros2 run wheeltec_webapp web            # 默认 0.0.0.0:8080
ros2 run wheeltec_webapp web -p 9090    # 换端口
```

## 开机自启（推荐，免 SSH）

```bash
sudo cp ~/wheeltec_ros2/src/wheeltec_webapp/systemd/wheeltec-webapp.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now wheeltec-webapp
systemctl status wheeltec-webapp        # 查看状态
journalctl -u wheeltec-webapp -f        # 查看日志
```

## 使用流程

1. 浏览器打开 `http://小车IP:8080`（IP 用 `hostname -I` 查；iPad 可"添加到主屏幕"全屏用）
2. **建图**：点"🗺️ 建图" → 摇杆慢速遥控走一圈（画布实时显示彩色点云+紫色轨迹）→ "⏹ 停止"（自动保存 PCD 分段）
3. **保存地图**：点"💾 保存地图"，日志出现"地面校平完成"后自动加载 2D 地图预览
4. **导航**：点"🧭 导航" → 等日志出现"重定位成功" → "🎯 设目标点"在地图上按下-拖方向-松开
5. 小车被搬动后不恢复 → "🔄 自动重定位"

## 技术备注

- 仅依赖系统已有的 `python3-websockets`(9.1) + `numpy` + `PIL`，无需联网加载任何前端库
- 单端口：HTTP 静态页与 WebSocket 复用同一端口（websockets 的 `process_request`）
- 地图以 PNG 传输（服务端 numpy 转位图），点云/扫描/路径为二进制 Float32 帧，1 字节类型前缀
- 建图与导航互斥由 launch 自带的防重复实例机制 + 本服务的模式管理双重保证
- 内存占用约 60-80MB，对 4GB 整机无压力；页面端点云累积在浏览器侧，不占小车内存
