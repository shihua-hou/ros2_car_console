#!/usr/bin/env python3
# coding=utf-8
"""
GPS定位守卫节点：监控AMCL健康度，必要时用RTK救援重定位

功能：
  1. 监控AMCL定位质量（协方差、粒子分布）
  2. 对比GPS位置与AMCL估计的偏差
  3. 检测到定位丢失时，用GPS位置发送 /initialpose 重置AMCL

设计原则（解耦 + 优雅降级）：
  - GPS不可用时自动禁用守卫功能，不影响正常AMCL定位
  - 需要GPS-地图对齐文件，文件不存在时自动禁用
  - AMCL正常时不干预，只在异常时救援
  - 所有参数可配置，适应不同环境
"""
import os
import math
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped
from sensor_msgs.msg import NavSatFix
from tf_transformations import quaternion_from_euler, euler_from_quaternion

try:
    from pyproj import Proj
    PYPROJ_AVAILABLE = True
except ImportError:
    PYPROJ_AVAILABLE = False


class GPSLocalizationGuard(Node):
    def __init__(self):
        super().__init__('gps_localization_guard')

        # ============ 参数配置 ============
        self.declare_parameter('enable_guard', True)  # 总开关
        self.declare_parameter('calibration_file',
            os.path.expanduser('~/wheeltec_ros2/outdoor_maps/gps_map_calibration.yaml'))
        self.declare_parameter('amcl_topic', '/amcl_pose')
        self.declare_parameter('gps_fix_topic', '/gps/fix')
        self.declare_parameter('gps_heading_topic', '/gps/euler')  # 双RTK航向角（可选）
        self.declare_parameter('initialpose_topic', '/initialpose')

        # 阈值参数
        self.declare_parameter('max_covariance', 0.5)  # AMCL协方差阈值(m^2)
        self.declare_parameter('max_gps_amcl_distance', 5.0)  # GPS-AMCL最大偏差(m)
        self.declare_parameter('bad_count_threshold', 3)  # 连续异常次数触发救援
        self.declare_parameter('min_gps_quality', 0)  # 最低GPS质量（-1=无效, 0=单点, 1=差分, 2=固定）
        self.declare_parameter('check_rate', 1.0)  # 检查频率(Hz)

        # GPS位置协方差（用于initialpose）
        self.declare_parameter('gps_xy_std', 0.05)  # RTK固定解标准差(m)
        self.declare_parameter('gps_heading_std', 0.5)  # 航向角标准差(rad)，单天线时不确定性大

        # 导航正在加载的地图(.yaml 路径)。给了它就用和地图同名的
        # <地图名>.gps.yaml 做标定, 并校验里面的 map_name —— 标定和地图必须同源。
        self.declare_parameter('map_file', '')

        # 读取参数
        self.enable_guard = self.get_parameter('enable_guard').value
        self.map_file = os.path.expanduser(self.get_parameter('map_file').value)
        self.map_name = (os.path.splitext(os.path.basename(self.map_file))[0]
                         if self.map_file else '')
        if self.map_name:
            # 和地图放在一起、存图时自动生成的那份
            self.calib_file = os.path.join(
                os.path.dirname(self.map_file), self.map_name + '.gps.yaml')
        else:
            # 兼容手工流程: 没传地图路径就退回全局路径, 但这份标定与地图无绑定
            self.calib_file = os.path.expanduser(
                self.get_parameter('calibration_file').value)
        self.max_cov = self.get_parameter('max_covariance').value
        self.max_distance = self.get_parameter('max_gps_amcl_distance').value
        self.bad_threshold = self.get_parameter('bad_count_threshold').value
        self.min_gps_quality = self.get_parameter('min_gps_quality').value
        self.check_rate = self.get_parameter('check_rate').value
        self.gps_xy_std = self.get_parameter('gps_xy_std').value
        self.gps_heading_std = self.get_parameter('gps_heading_std').value

        # ============ 状态变量 ============
        self.guard_active = False  # 守卫是否激活
        self.calibration = None
        self.proj = None
        self.R = None  # 旋转矩阵
        self.t = None  # 平移向量

        self.last_amcl_pose = None
        self.last_gps_fix = None
        self.last_gps_heading = None
        self.amcl_bad_count = 0
        self.total_rescues = 0

        # ============ 初始化 ============
        if not self.enable_guard:
            self.get_logger().info('GPS定位守卫已禁用（参数 enable_guard=false）')
            return

        # 检查依赖
        if not PYPROJ_AVAILABLE:
            self.get_logger().error('pyproj库未安装，GPS守卫无法启动！')
            self.get_logger().error('请运行: pip3 install pyproj')
            self.enable_guard = False
            return

        # 加载对齐参数
        if not self._load_calibration():
            self.get_logger().warn('GPS-地图标定不可用，GPS守卫不会激活（导航正常进行）')
            self.get_logger().warn(f'  期望路径: {self.calib_file}')
            if self.map_name:
                self.get_logger().warn(
                    '  这份标定应在存图时自动生成。没有说明当次建图 RTK 没到固定解,'
                    ' 或固定解期间轨迹近似共线 —— 重新建图时保持固定解并绕环。')
            else:
                self.get_logger().warn('  未传入 map_file，退回了全局标定路径')
            self.enable_guard = False
            return

        # ============ 创建订阅和发布 ============
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('amcl_topic').value,
            self.amcl_callback,
            10
        )

        self.gps_sub = self.create_subscription(
            NavSatFix,
            self.get_parameter('gps_fix_topic').value,
            self.gps_callback,
            10
        )

        self.heading_sub = self.create_subscription(
            Vector3Stamped,
            self.get_parameter('gps_heading_topic').value,
            self.heading_callback,
            10
        )

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter('initialpose_topic').value,
            10
        )

        # 定时检查
        self.check_timer = self.create_timer(1.0 / self.check_rate, self.check_health)

        self.get_logger().info('GPS定位守卫已启动')
        self.get_logger().info(f'  对齐参数: {self.calib_file}')
        self.get_logger().info(f'  协方差阈值: {self.max_cov} m^2')
        self.get_logger().info(f'  距离阈值: {self.max_distance} m')
        self.get_logger().info(f'  触发阈值: 连续{self.bad_threshold}次异常')
        self.guard_active = True

    def _load_calibration(self):
        """加载GPS-地图对齐参数"""
        if not os.path.exists(self.calib_file):
            return False

        try:
            with open(self.calib_file, 'r') as f:
                self.calibration = yaml.safe_load(f)

            # 标定必须属于当前这张图。不校验的话, 换一次建图旧标定就会继续
            # 被加载, 而两次会话的 camera_init 差着任意旋转 —— 2026-09-16 实测
            # 这种错配让每次救援把车瞬移中位 31m。
            calib_map = self.calibration.get('map_name', '')
            if self.map_name and calib_map and calib_map != self.map_name:
                self.get_logger().error(
                    f'标定与地图不匹配: 标定属于 "{calib_map}", 当前地图是 '
                    f'"{self.map_name}" —— 拒绝加载(用它会把车拉到错误位置)')
                return False
            if self.map_name and not calib_map:
                self.get_logger().warn(
                    '标定文件里没有 map_name(旧格式), 无法确认是否属于当前地图。'
                    ' 建议重新存一次图让它自动重新生成。')

            transform = self.calibration['gps_to_map_transform']
            self.R = np.array(transform['rotation'])
            self.t = np.array(transform['translation'])

            utm_zone = transform.get('utm_zone', 50)
            utm_band = transform.get('utm_band', 'N')
            hemisphere = 'north' if utm_band >= 'N' else 'south'

            self.proj = Proj(proj='utm', zone=utm_zone, ellps='WGS84',
                           datum='WGS84', units='m', south=(hemisphere == 'south'))

            self.get_logger().info(f'已加载GPS对齐参数: UTM Zone {utm_zone}{utm_band}')
            self.get_logger().info(f'  归属地图: {calib_map or "(未记录)"}')
            self.get_logger().info(f'  RMS误差: {self.calibration.get("rms_error", "N/A")} m')
            self.get_logger().info(f'  对齐点数: {self.calibration.get("num_points", "N/A")}')
            # 轴比大说明拟合样本近似共线, 旋转角没被约束住 —— RMS 再小也不能全信
            ratio = self.calibration.get('axis_ratio')
            if ratio is not None:
                self.get_logger().info(f'  样本轴比: {ratio:.1f}'
                                       + ('' if ratio <= 5 else '  ⚠ 近似共线, 旋转角不可靠'))
            return True

        except Exception as e:
            self.get_logger().error(f'加载对齐参数失败: {e}')
            return False

    def amcl_callback(self, msg):
        """接收AMCL位姿"""
        self.last_amcl_pose = msg

    def gps_callback(self, msg):
        """接收GPS数据"""
        # 只接受质量足够的GPS数据
        if msg.status.status >= self.min_gps_quality:
            self.last_gps_fix = msg

    def heading_callback(self, msg):
        """接收双RTK航向角（可选）"""
        # msg.vector.z 是航向角（度）
        self.last_gps_heading = math.radians(msg.vector.z)

    def check_health(self):
        """定时检查定位健康度"""
        if not self.guard_active:
            return

        if self.last_amcl_pose is None:
            return  # 等待AMCL数据

        if self.last_gps_fix is None:
            # GPS不可用，只监控不救援
            return

        # ============ 检查1：AMCL协方差 ============
        cov = self.last_amcl_pose.pose.covariance
        cov_x = cov[0]   # x方差
        cov_y = cov[7]   # y方差
        cov_yaw = cov[35]  # yaw方差

        amcl_bad = False

        if cov_x > self.max_cov or cov_y > self.max_cov:
            amcl_bad = True
            self.get_logger().warn(
                f'AMCL协方差过大: x={cov_x:.3f}, y={cov_y:.3f} (阈值={self.max_cov})',
                throttle_duration_sec=2.0
            )

        # ============ 检查2：GPS与AMCL位置偏差 ============
        try:
            gps_map_pos = self._gps_to_map(self.last_gps_fix)
            amcl_pos = self.last_amcl_pose.pose.pose.position

            distance = math.sqrt(
                (gps_map_pos[0] - amcl_pos.x) ** 2 +
                (gps_map_pos[1] - amcl_pos.y) ** 2
            )

            if distance > self.max_distance:
                amcl_bad = True
                self.get_logger().warn(
                    f'GPS-AMCL位置偏差: {distance:.2f}m (阈值={self.max_distance}m)',
                    throttle_duration_sec=2.0
                )
        except Exception as e:
            self.get_logger().error(f'GPS坐标转换失败: {e}', throttle_duration_sec=5.0)
            return

        # ============ 判断是否需要救援 ============
        if amcl_bad:
            self.amcl_bad_count += 1

            if self.amcl_bad_count >= self.bad_threshold:
                self.get_logger().error(
                    f'定位丢失（连续{self.amcl_bad_count}次异常）！使用RTK重置AMCL'
                )
                self._rescue_with_gps()
                self.amcl_bad_count = 0  # 重置计数
        else:
            # 定位正常，清零计数
            if cov_x < self.max_cov * 0.5 and distance < self.max_distance * 0.5:
                if self.amcl_bad_count > 0:
                    self.get_logger().info('定位已恢复正常')
                self.amcl_bad_count = 0

    def _gps_to_map(self, gps_msg):
        """GPS经纬度 → 地图坐标"""
        # 1. 经纬度 → UTM
        utm_x, utm_y = self.proj(gps_msg.longitude, gps_msg.latitude)

        # 2. UTM → 地图坐标（应用对齐变换的逆）
        utm_vec = np.array([utm_x, utm_y])
        map_vec = self.R.T @ (utm_vec - self.t)

        return map_vec

    def _rescue_with_gps(self):
        """用GPS位置重置AMCL"""
        try:
            map_pos = self._gps_to_map(self.last_gps_fix)

            # 构造 initialpose 消息
            init_pose = PoseWithCovarianceStamped()
            init_pose.header.stamp = self.get_clock().now().to_msg()
            init_pose.header.frame_id = 'map'

            init_pose.pose.pose.position.x = map_pos[0]
            init_pose.pose.pose.position.y = map_pos[1]
            init_pose.pose.pose.position.z = 0.0

            # 朝向：优先使用双RTK航向，否则保留AMCL上一次的朝向
            if self.last_gps_heading is not None:
                # 双天线航向是相对正北的, 地图系不是 —— 必须用标定里的旋转换算。
                # 直接拿来当 yaw 会差出一个"地图朝向与正北的夹角"
                # (2026-09-16 本场地实测约 101°, 位置对朝向错, AMCL 反而更乱)。
                theta = math.atan2(self.R[1, 0], self.R[0, 0])   # map -> UTM
                yaw_map = (math.pi / 2.0 - self.last_gps_heading) - theta
                yaw_map = math.atan2(math.sin(yaw_map), math.cos(yaw_map))
                q = quaternion_from_euler(0, 0, yaw_map)
                self.get_logger().info(
                    f'  救援朝向: 地理航向 '
                    f'{math.degrees(self.last_gps_heading):.1f}° -> 地图系 '
                    f'{math.degrees(yaw_map):.1f}°')
                init_pose.pose.pose.orientation.x = q[0]
                init_pose.pose.pose.orientation.y = q[1]
                init_pose.pose.pose.orientation.z = q[2]
                init_pose.pose.pose.orientation.w = q[3]
            else:
                # 使用AMCL当前朝向
                init_pose.pose.pose.orientation = self.last_amcl_pose.pose.pose.orientation

            # 协方差：反映GPS和航向的不确定性
            xy_var = self.gps_xy_std ** 2
            yaw_var = self.gps_heading_std ** 2

            init_pose.pose.covariance = [
                xy_var, 0, 0, 0, 0, 0,
                0, xy_var, 0, 0, 0, 0,
                0, 0, 0.01, 0, 0, 0,  # z方差（平面运动，不关心）
                0, 0, 0, 0.01, 0, 0,  # roll方差
                0, 0, 0, 0, 0.01, 0,  # pitch方差
                0, 0, 0, 0, 0, yaw_var  # yaw方差
            ]

            # 发布
            self.initialpose_pub.publish(init_pose)
            self.total_rescues += 1

            self.get_logger().info(
                f'已发送GPS重定位 (第{self.total_rescues}次): '
                f'pos=({map_pos[0]:.2f}, {map_pos[1]:.2f}), '
                f'heading={math.degrees(self.last_gps_heading) if self.last_gps_heading else "保持":.1f}°'
            )

        except Exception as e:
            self.get_logger().error(f'GPS救援失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = GPSLocalizationGuard()

    if node.enable_guard:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        node.get_logger().info('守卫节点未激活，退出')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
