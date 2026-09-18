# Web App v2 - GPS 可视化功能集成完成

## 已完成的功能

### 1. 后端集成（app.py）

#### 添加的导入
- `from sensor_msgs.msg import NavSatFix` - GPS 消息类型
- `B_GPS = 10` - GPS 二进制消息类型标识

#### GPS 数据订阅
```python
self.create_subscription(NavSatFix, '/gps/fix', self._cb('gps'), 10)
```

#### GPS 数据推送（task_fast 函数）
- 约 2Hz 频率推送 GPS 数据
- 包含：纬度、经度、高度、状态、服务类型、协方差

```python
if gps is not None and tick % 3 == 0:
    self.send_json(
        t='gps',
        lat=gps.latitude,
        lon=gps.longitude,
        alt=gps.altitude,
        status=gps.status.status,
        service=gps.status.service,
        cov=list(gps.position_covariance[:3])
    )
```

---

### 2. 前端集成（v2.html）

#### 高德地图 API 集成
```html
<script src="https://webapi.amap.com/maps?v=2.0&key=c9515d8a263546e0a94f01137e8e44d1&plugin=AMap.Scale,AMap.ToolBar"></script>
<script>
  window._AMapSecurityConfig = { securityJsCode: '9e5bf6810330a0b354c2e7c9e002b5ce' };
</script>
```

#### CSS 样式
- GPS 地图窗口样式（浮动窗口，320x240px）
- GPS 信息面板样式
- GPS 状态芯片样式（不同状态不同颜色）

#### HTML 结构
1. **GPS 地图窗口**（#gpsMap）
   - 高德地图容器
   - GPS 信息面板（状态、坐标）

2. **首页 GPS 状态芯片**
   - RTK 固定解：绿色
   - DGPS：橙色
   - GPS：蓝色
   - 无信号：红色

#### JavaScript 功能

##### 全局变量
```javascript
let gps = {ok:false, lat:0, lon:0, alt:0, status:-1, heading:0};
let gpsMap = null, gpsMarker = null;
```

##### GPS 数据处理（onJson 函数）
```javascript
if(m.t==='gps'){
  gps.ok = (m.status >= 0);
  gps.lat = m.lat;
  gps.lon = m.lon;
  gps.alt = m.alt;
  gps.status = m.status;
  updateGpsDisplay();
  // 更新地图标记位置
}
```

##### 核心函数

1. **initGpsMap()** - 初始化高德地图
   - 创建地图实例
   - 添加比例尺和工具条
   - 创建机器人位置标记

2. **updateGpsDisplay()** - 更新 GPS 显示
   - 更新 GPS 信息面板
   - 更新状态文本和颜色
   - 显示坐标信息

3. **toggleGpsMap(show)** - 控制地图显示/隐藏
   - 在导航和建图页面显示
   - 在其他页面隐藏
   - 自动居中到当前位置

---

## GPS 状态映射

| status 值 | 含义 | 显示文本 | 颜色 |
|-----------|------|----------|------|
| 2 | RTK 固定解 | RTK固定解 | 绿色 |
| 1 | DGPS | DGPS | 橙色 |
| 0 | GPS | GPS | 蓝色 |
| -1 | 无定位 | 无信号 | 红色 |

---

## 使用场景

### 1. 首页（仪表盘）
- **位置**：状态胶囊区域（底部）
- **显示内容**：GPS 状态芯片
- **示例**：`[·] GPS RTK固定解`（绿色）

### 2. 导航页面（/nav）
- **位置**：
  - 右上角：GPS 地图窗口（320x240px）
  - 顶栏状态区：GPS 状态文本
- **功能**：
  - 实时显示机器人在高德地图上的位置
  - 显示 GPS 状态和坐标
  - 地图自动跟随机器人移动

### 3. 建图页面（/map）
- **位置**：
  - 右上角：GPS 地图窗口
  - 顶栏状态区：GPS 状态文本
- **功能**：
  - 在建图时实时显示 GPS 位置
  - 可以对比雷达地图和 GPS 轨迹

---

## 启动和测试

### 1. 启动 GPS 驱动
```bash
cd ~/wheeltec_ros2
source install/setup.bash
ros2 launch wheeltec_gps_driver wheeltec_dual_rtk_driver_nmea.launch.py
```

### 2. 启动 Web App
```bash
ros2 run wheeltec_webapp web
```

### 3. 打开浏览器
```
http://192.168.0.100:8080/v2
```

### 4. 验证功能

#### 首页验证
- [ ] 状态胶囊区显示 GPS 状态芯片
- [ ] GPS 有信号时显示绿色/橙色/蓝色
- [ ] GPS 无信号时显示红色

#### 导航页面验证
- [ ] 右上角显示 GPS 地图窗口
- [ ] 地图上显示机器人当前位置（蓝色圆点）
- [ ] GPS 信息面板显示状态和坐标
- [ ] 机器人移动时，地图标记跟随移动
- [ ] 地图可以缩放、平移

#### 建图页面验证
- [ ] 右上角显示 GPS 地图窗口
- [ ] 建图时 GPS 位置实时更新
- [ ] 可以对比点云轨迹和 GPS 轨迹

---

## 配置说明

### 高德地图配置
当前使用的是你提供的高德地图配置：

```javascript
{
  "amapKey": "b6e147797f54ba4ac2056ea24ef99b4c",
  "amapSecurityCode": "9e5bf6810330a0b354c2e7c9e002b5ce",
  "amapWebKey": "c9515d8a263546e0a94f01137e8e44d1",
  "defaultCenter": [116.397428, 39.90923],
  "defaultZoom": 18
}
```

**注意**：
- Web API 使用 `amapWebKey`
- 安全密钥使用 `amapSecurityCode`
- 默认中心点为北京天安门（首次加载使用）
- 有 GPS 数据后会自动切换到实际位置

---

## 性能优化

1. **GPS 数据推送频率**：约 2Hz（每 0.6 秒），避免过载
2. **地图懒加载**：只在导航/建图页面初始化地图
3. **地图重用**：切换页面时不重建地图实例
4. **条件渲染**：无 GPS 信号时不显示地图窗口

---

## 故障排查

### GPS 状态显示"无信号"
1. 检查 GPS 驱动是否启动
   ```bash
   ros2 topic list | grep gps
   ros2 topic echo /gps/fix --once
   ```

2. 检查串口连接
   ```bash
   ls -l /dev/wheeltec_gnss
   ```

### 地图不显示
1. 检查网络连接（需要访问高德地图 API）
2. 打开浏览器开发者工具查看控制台错误
3. 验证高德地图 key 是否有效

### 地图位置不准确
1. 等待 GPS 获取 RTK 固定解（status=2）
2. 检查 GPS 天线安装位置
3. 确保在室外空旷环境

---

## 未来扩展

### 可选增强功能

1. **航向角显示**
   - 解析 $GNHPR 消息获取双天线航向角
   - 在地图上显示机器人朝向箭头

2. **轨迹回放**
   - 保存 GPS 轨迹历史
   - 在地图上绘制行驶路径

3. **地图切换**
   - 支持卫星图/路网图切换
   - 支持 3D 地图视图

4. **GPS-地图对齐可视化**
   - 在高德地图上叠加显示雷达地图
   - 显示 GPS 和 AMCL 定位的偏差

5. **GPS 守卫状态显示**
   - 显示 GPS-AMCL 偏差值
   - 显示重定位事件

---

## 总结

✅ **后端集成完成**：订阅 `/gps/fix` 话题，推送数据到前端  
✅ **前端集成完成**：高德地图显示、GPS 状态显示、实时位置更新  
✅ **UI 集成完成**：首页状态芯片、导航/建图页面地图窗口  
✅ **功能验证**：等待实际测试  

**现在可以启动系统并在 Web 界面查看 GPS 数据了！** 🚀
