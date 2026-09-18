#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 /gps/fix 换算成地图系的里程计, 喂给全局 EKF。

为什么不用 robot_localization 自带的 navsat_transform_node
--------------------------------------------------------
那个节点要靠 IMU 的**绝对航向**(或手工填 datum)去对齐 UTM 和地图系。但本车
IMU 驱动自报 orientation_covariance = 1e6(自己都说姿态不可信), 现有 EKF 也是
imu0_relative: true 当相对量用 —— 磁力计这条路在金属车体上是堵死的。

而存图时已经把地图地理配准过了(georef_map.py 生成的 <地图名>.gps.yaml 里就是
精确的 map<->UTM 刚体变换), 直接拿来换算比配 navsat_transform 简单得多, 也少
一个容易配错的环节。

航向从哪来
----------
不用双天线, 用**位移方向**(course over ground): RTK 固定解 2cm 精度, 取 2m
基线 -> 航向噪声约 0.6°, 人群和磁场都干扰不到。
代价是只在车**前进**时有效 —— 停车和原地转向时不发布航向(把 yaw 协方差置成
天文数字, EKF 自然会忽略它, 靠 IMU+轮速撑住)。
倒车时位移方向和车头差 180°, 所以用 /odom 的 linear.x 符号做门控, 倒车直接
不给航向, 不去猜。

协方差按解算等级给
------------------
固定解和单点解的误差差两个数量级, 一视同仁的话, 要么浪费固定解的精度, 要么
让单点解把滤波器带偏。无解直接不发。
"""
import math
import os
from collections import deque

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Quaternion, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import UInt8

try:
    from pyproj import Proj
except ImportError:
    Proj = None

# GGA 质量码 -> 位置标准差(米)。无解/不认识的码不发布。
QUALITY_STD = {
    4: 0.02,    # RTK 固定解
    5: 0.50,    # RTK 浮点解
    2: 1.50,    # DGPS 差分
    1: 5.00,    # 单点定位
}
# 没有 /gps/quality 时退回 NavSatFix.status
STATUS_STD = {2: 0.02, 1: 0.50, 0: 5.00}


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class GpsMapOdom(Node):

    def __init__(self):
        super().__init__('gps_map_odom')

        self.declare_parameter('map_file', '')
        self.declare_parameter('fix_topic', '/gps/fix')
        self.declare_parameter('quality_topic', '/gps/quality')
        self.declare_parameter('output_topic', '/odometry/gps')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        # 双天线航向偏置 = 实测的 (euler - 车头罗盘航向); 车头 = euler - 这个值。
        # /gps/euler 报的是基线方向, 天线横向安装所以接近 -90°。
        # 2026-09-16 实测 -85.1°, 两个相差 181.8° 的朝向上极差 0.0° —— 是常数。
        # 换车或重装天线必须重测(这是机械安装角, 不是标定参数)。
        # 三次独立测量(都是 euler - 车头罗盘航向):
        #   9/16  GPS 位移方向              -85.1°  (当时没录轮速, 无法排除倒车)
        #   9/18  GPS 位移 + 轮速确认前进   -86.9° ± 3.7°  (12 个 3m 窗口, GPS/车轮路程 1.00)
        #   9/18  激光-地图匹配(固定解)     -91.8°  (匹配分 0.81, 两次一致)
        # 取 -86.9°: 唯一一个方向被轮速确认、GPS 被路程比校验过的测量。
        # 负号的物理含义: 主天线在右、基线从右指向左, 读数比车头小约 90°。
        # 注意 9/18 曾一度把符号改反又撤回 —— 撤回依据的是一次匹配分只有 0.53 的
        # 激光测量, 不可靠; 以后再怀疑这个值, 先用 laser_offset.py 在固定解 +
        # 地图结构丰富的点上测, 看它报不报"可信"。
        self.declare_parameter('heading_offset_deg', -86.9)
        # 实测离散 4.55°/8.51°(两趟), 取 8° 偏保守 —— 宁可让 EKF 少信一点。
        self.declare_parameter('heading_std_deg', 8.0)
        self.declare_parameter('euler_topic', '/gps/euler')
        # /gps/euler 真实更新率只有 1Hz(驱动以 20Hz 重复推同一个值), 超过这个
        # 时间没更新就当航向失效, 别拿旧值当新观测喂给 EKF。
        self.declare_parameter('heading_timeout', 3.0)
        # 转弯时不发 GPS 航向: GPS 航向比陀螺晚约 0.3s, 以 27°/s 转弯时到达即落后
        # 约 8°, 会把 EKF 拽出 5~8.5° 的跳变(2026-09-18 实测)。转弯中交给陀螺
        # (比例误差 1.3%), 直行/停车时再用 GPS 航向消除累积漂移。
        self.declare_parameter('heading_max_turn_rate_deg', 5.0)
        self.declare_parameter('imu_topic', '/imu/data_filtered')
        # 主天线相对 base_footprint 的杆臂, 车体系(+x 车头, +y 左)。
        # 2026-09-16 实测确认: 两天线在轴心左右各 15cm, 主天线在右 -> (0, -0.15)。
        # 262 个干净样本实测车体系横向偏置 -0.089 ± 0.090 m, 方向吻合。
        # 暂不补偿: 把它旋转到地图系需要可信的偏航角, 而当前唯一的偏航源
        # (AMCL)正是要被替换掉的那个。第 2 步 EKF 起来后用 EKF 的偏航来转。
        self.declare_parameter('antenna_offset_x', 0.0)
        self.declare_parameter('antenna_offset_y', -0.15)

        self.map_file = os.path.expanduser(
            self.get_parameter('map_file').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.head_off = math.radians(
            self.get_parameter('heading_offset_deg').value)
        self.head_var = math.radians(
            self.get_parameter('heading_std_deg').value) ** 2
        self.head_timeout = self.get_parameter('heading_timeout').value
        self.max_turn = math.radians(
            self.get_parameter('heading_max_turn_rate_deg').value)
        self.lever = np.array([
            self.get_parameter('antenna_offset_x').value,
            self.get_parameter('antenna_offset_y').value], dtype=float)

        if Proj is None:
            self.get_logger().error('缺少 pyproj (pip3 install pyproj), 节点退出')
            raise SystemExit(1)
        if not self._load_calibration():
            raise SystemExit(1)

        self.quality = None
        self.head = None             # (接收时刻, 地图系 yaw)
        self.n_pub = 0
        # 去重用: 驱动 20Hz 重复推送, 真实更新只有 1Hz(见 on_fix / on_euler)
        self.last_fix_key = None
        self.last_eul_z = None
        self.last_pos = None         # 最近一次发布的位置, 航向消息里当占位
        self.n_dup = 0
        self.n_bad_head = 0
        self.n_head = 0
        self.n_turn_skip = 0
        self.gyro = deque(maxlen=100)    # (时刻, 陀螺 z), 判断最近是否在转弯
        self.last_warn = 0.0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(
            Odometry, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            NavSatFix, self.get_parameter('fix_topic').value, self.on_fix, qos)
        self.create_subscription(
            UInt8, self.get_parameter('quality_topic').value,
            lambda m: setattr(self, 'quality', int(m.data)), qos)
        self.create_subscription(
            Vector3Stamped, self.get_parameter('euler_topic').value,
            self.on_euler, qos)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value,
            lambda m: self.gyro.append(
                (self.get_clock().now().nanoseconds * 1e-9, m.angular_velocity.z)),
            qos)

        self.get_logger().info(
            f'gps_map_odom 已启动 -> {self.get_parameter("output_topic").value}')
        self.get_logger().info(
            f'  航向: 双天线, 偏置 '
            f'{self.get_parameter("heading_offset_deg").value:+.1f}°, '
            f'std {self.get_parameter("heading_std_deg").value:.0f}°, '
            f'超时 {self.head_timeout:.0f}s')
        self.get_logger().info(
            f'  天线杆臂(车体系): ({self.lever[0]:+.2f}, {self.lever[1]:+.2f}) m')

    # ---------- 标定 ----------
    def _load_calibration(self):
        if not self.map_file:
            self.get_logger().error('必须传 map_file(导航正在加载的地图 .yaml)')
            return False
        name = os.path.splitext(os.path.basename(self.map_file))[0]
        cal = os.path.join(os.path.dirname(self.map_file), name + '.gps.yaml')
        if not os.path.exists(cal):
            self.get_logger().error(f'找不到地图的 GPS 标定: {cal}')
            self.get_logger().error('  存图时会自动生成; 没有说明当次建图 RTK 没到'
                                    '固定解, 或固定解期间轨迹近似共线')
            return False
        with open(cal, encoding='utf-8') as f:
            d = yaml.safe_load(f)

        # 标定必须属于这张图 —— 2026-09-16 就是因为没这道检查, 用了另一次会话的
        # 标定, 位置算出来偏了中位 31m
        owner = d.get('map_name', '')
        if owner and owner != name:
            self.get_logger().error(
                f'标定属于 "{owner}", 当前地图是 "{name}" —— 拒绝加载')
            return False

        tf = d['gps_to_map_transform']
        self.R = np.array(tf['rotation'], dtype=float)      # utm = R@map + t
        self.t = np.array(tf['translation'], dtype=float)
        zone = tf.get('utm_zone', 50)
        band = tf.get('utm_band', 'N')
        self.proj = Proj(proj='utm', zone=zone, ellps='WGS84', datum='WGS84',
                         units='m', south=(band < 'N'))
        rms = d.get('rms_error')
        ratio = d.get('axis_ratio')
        self.get_logger().info(f'已加载 {os.path.basename(cal)}: UTM {zone}{band}')
        self.get_logger().info(
            f'  归属地图 {owner or "(未记录)"}, RMS {rms}, 轴比 {ratio}')
        # 标定本身的误差要计入输出协方差, 否则会高估 GPS 的可信度
        self.cal_var = float(rms) ** 2 if rms else 0.25
        return True

    # ---------- 主回调 ----------
    def on_fix(self, msg):
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return
        # 驱动以 20Hz 推送, 但接收机真实定位只有 1Hz, 同一组经纬度会被重复推 20 次。
        # 原样转发会被 EKF 当成 20 次独立测量: 严重过度自信, 而且车在动时旧位置
        # 最多落后 0.5m —— 被一秒 20 次往回拽、新值一到又往前跳, 位姿锯齿状。
        # 2026-09-18 导航时车左右摆、点云乱转, 根子在这儿。
        key = (msg.latitude, msg.longitude, msg.altitude)
        if key == self.last_fix_key:
            self.n_dup += 1
            return
        self.last_fix_key = key
        std = (QUALITY_STD.get(self.quality) if self.quality is not None
               else STATUS_STD.get(msg.status.status))
        if std is None:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self.last_warn > 10.0:
                self.last_warn = now
                self.get_logger().warn(
                    f'解算等级不可用(quality={self.quality}, '
                    f'status={msg.status.status}), 不发布')
            return

        ux, uy = self.proj(msg.longitude, msg.latitude)
        mx, my = self.R.T @ (np.array([ux, uy]) - self.t)

        yaw = self._heading()
        # 杆臂补偿: 接收机报的是右侧主天线的相位中心, 不是 base_footprint。
        # 把车体系的杆臂转到地图系减掉。没有航向时转不了, 只能带着这 15cm 误差
        # (仍远小于 0.38m 的标定 RMS, 可以接受)。
        if yaw is not None and (self.lever != 0).any():
            c, s = math.cos(yaw), math.sin(yaw)
            mx -= c * self.lever[0] - s * self.lever[1]
            my -= s * self.lever[0] + c * self.lever[1]

        o = Odometry()
        o.header.stamp = msg.header.stamp
        o.header.frame_id = self.map_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x = float(mx)
        o.pose.pose.position.y = float(my)

        # GPS 自身误差 + 标定误差。两者独立, 方差相加。
        var = std ** 2 + self.cal_var
        cov = [0.0] * 36
        cov[0] = cov[7] = var
        cov[14] = 1e6                      # z 不给
        cov[21] = cov[28] = 1e6            # roll/pitch 不给

        # 位置消息只带位置: 航向由 on_euler 在新航向到达时单独发(见 _publish_heading)。
        # 绑在一起发的话, 新航向要等下一个位置才能送出去, 最多滞后 1 秒。
        cov[35] = 1e6
        o.pose.pose.orientation.w = 1.0
        o.pose.covariance = cov

        self.pub.publish(o)
        self.last_pos = (float(mx), float(my))
        self.n_pub += 1
        if self.n_pub % 30 == 0:           # 真实 1Hz, 约半分钟一条
            self.get_logger().info(
                f'位置观测 {self.n_pub} 次, 航向观测 {self.n_head} 次; '
                f'丢弃重复定位 {self.n_dup} 帧, 无效航向 {self.n_bad_head} 帧, '
                f'转弯中略过航向 {self.n_turn_skip} 次; '
                f'最新 map({mx:.2f}, {my:.2f}) std {math.sqrt(var):.2f}m')

    def on_euler(self, m):
        """双天线航向 -> 地图系 yaw。

        m.vector.z 是基线方向的罗盘航向(北起顺时针, 度)。三步换算:
          车头罗盘航向 = z - heading_offset      (offset 定义为 euler-车头, 实测 -86.9°)
          ENU 航向     = 90° - 罗盘航向           (罗盘 -> 东起逆时针)
          地图系 yaw   = ENU 航向 - theta         (theta = 标定里 map->UTM 的旋转)
        最后一步不能省: 地图系是 FAST-LIO 的 camera_init, 和正北毫无关系
        (本场地实测差 93.88°)。
        """
        z = m.vector.z
        # 驱动在航向无效时给 0.0(2026-09-18 实测连续 61 帧)。真实航向恰好 0.000°
        # 的概率可以忽略, 当无效处理 —— 否则会把车头猛拽到一个垃圾方向。
        if z == 0.0:
            self.n_bad_head += 1
            return
        # 同样是 1Hz 真实更新、20Hz 重复推送: 只在值变化时当作新测量
        if z == self.last_eul_z:
            return
        self.last_eul_z = z
        # 减, 不是加: heading_offset 是实测的 (euler - 车头), 所以 车头 = euler - offset。
        # 2026-09-17 发现这里曾写成加号, 航向错 170.2°(位置对、车头几乎反了)。
        compass = math.radians(z) - self.head_off
        yaw_enu = math.pi / 2.0 - compass
        theta = math.atan2(self.R[1, 0], self.R[0, 0])
        yaw = math.atan2(math.sin(yaw_enu - theta), math.cos(yaw_enu - theta))
        now = self.get_clock().now().nanoseconds * 1e-9
        self.head = (now, yaw)           # 杆臂补偿照常用最新航向
        # 最近 0.6s(覆盖 GPS 航向约 0.3s 的延迟, 再留余量)内在转弯, 就不发。
        # 没有陀螺数据时不门控, 按原样发 —— 宁可有点跳, 不能没有航向。
        recent = [abs(w) for t, w in self.gyro if now - t < 0.6]
        if recent and max(recent) > self.max_turn:
            self.n_turn_skip += 1
            return
        self._publish_heading(yaw, m.header.stamp)

    def _publish_heading(self, yaw, stamp):
        """只带航向的观测: x/y 方差 1e6, EKF 只会用其中的 yaw。

        还没有过任何位置时不发: 占位用的 x/y 没有意义, 而下一秒就会有新航向。
        """
        if self.last_pos is None:
            return
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.map_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x, o.pose.pose.position.y = self.last_pos
        o.pose.pose.orientation = yaw_to_quat(yaw)
        cov = [0.0] * 36
        cov[0] = cov[7] = cov[14] = cov[21] = cov[28] = 1e6
        cov[35] = self.head_var
        o.pose.covariance = cov
        self.pub.publish(o)
        self.n_head += 1

    def _heading(self):
        """当前可用的地图系航向; 超时就返回 None(别拿旧值当新观测)。"""
        if self.head is None:
            return None
        age = self.get_clock().now().nanoseconds * 1e-9 - self.head[0]
        return None if age > self.head_timeout else self.head[1]


def main():
    rclpy.init()
    try:
        node = GpsMapOdom()
    except SystemExit:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
