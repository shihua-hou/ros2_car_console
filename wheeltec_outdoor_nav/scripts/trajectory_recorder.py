#!/usr/bin/env python3
# coding=utf-8
"""
轨迹记录节点：同时记录雷达里程计和GPS轨迹，用于后处理对齐

订阅：
  /Odometry (nav_msgs/Odometry) - FAST-LIO2雷达里程计
  /gps/fix (sensor_msgs/NavSatFix) - RTK全局位置

输出：
  trajectory.csv - 时间戳对齐的双轨迹数据

设计原则：
  - GPS不可用时仍然可以记录纯雷达轨迹
  - 建图时与FAST-LIO2完全解耦，不影响建图质量
  - 自动处理时间戳同步
"""
import os
import csv
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')

        # 参数
        self.declare_parameter('lidar_odom_topic', '/Odometry')
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('output_dir', os.path.expanduser('~/wheeltec_ros2/outdoor_maps/'))
        self.declare_parameter('record_rate', 5.0)  # Hz，降采样记录

        self.lidar_topic = self.get_parameter('lidar_odom_topic').value
        self.gps_topic = self.get_parameter('gps_topic').value
        self.output_dir = os.path.expanduser(self.get_parameter('output_dir').value)
        self.record_rate = self.get_parameter('record_rate').value

        # 确保输出目录存在
        os.makedirs(self.output_dir, exist_ok=True)

        # 数据缓存
        self.lidar_data = None
        self.gps_data = None
        self.data_lock = threading.Lock()

        # 统计
        self.lidar_count = 0
        self.gps_count = 0
        self.gps_fixed_count = 0

        # 订阅
        self.lidar_sub = self.create_subscription(
            Odometry, self.lidar_topic, self.lidar_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, self.gps_topic, self.gps_callback, 10)

        # CSV输出文件
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_file = os.path.join(self.output_dir, f'trajectory_{timestamp}.csv')
        self.csv_writer = None
        self.csv_fd = open(self.csv_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_fd)
        # 写入表头
        self.csv_writer.writerow([
            'timestamp_sec', 'timestamp_nsec',
            'lidar_x', 'lidar_y', 'lidar_z',
            'lidar_qx', 'lidar_qy', 'lidar_qz', 'lidar_qw',
            'gps_lat', 'gps_lon', 'gps_alt',
            'gps_status', 'gps_cov_x', 'gps_cov_y', 'gps_cov_z'
        ])

        # 定时记录
        self.record_timer = self.create_timer(1.0 / self.record_rate, self.record_callback)

        # 定时统计
        self.stats_timer = self.create_timer(5.0, self.print_statistics)

        self.get_logger().info(f'轨迹记录节点已启动')
        self.get_logger().info(f'  雷达里程计: {self.lidar_topic}')
        self.get_logger().info(f'  GPS话题: {self.gps_topic}')
        self.get_logger().info(f'  输出文件: {self.csv_file}')
        self.get_logger().info(f'  记录频率: {self.record_rate} Hz')

    def lidar_callback(self, msg):
        """接收雷达里程计"""
        with self.data_lock:
            self.lidar_data = msg
            self.lidar_count += 1

    def gps_callback(self, msg):
        """接收GPS数据"""
        with self.data_lock:
            self.gps_data = msg
            self.gps_count += 1
            # 统计固定解数量（假设status.status >= 0表示有效）
            if msg.status.status >= 0:
                self.gps_fixed_count += 1

    def record_callback(self):
        """定时记录数据"""
        with self.data_lock:
            if self.lidar_data is None:
                return  # 等待雷达数据

            # 获取当前时间戳
            now = self.get_clock().now()
            ts_sec = now.seconds_nanoseconds()[0]
            ts_nsec = now.seconds_nanoseconds()[1]

            # 雷达数据（始终记录）
            lidar_pos = self.lidar_data.pose.pose.position
            lidar_ori = self.lidar_data.pose.pose.orientation

            # GPS数据（可能不可用）
            if self.gps_data is not None:
                gps_lat = self.gps_data.latitude
                gps_lon = self.gps_data.longitude
                gps_alt = self.gps_data.altitude
                gps_status = self.gps_data.status.status
                gps_cov = self.gps_data.position_covariance
                gps_cov_x = gps_cov[0]
                gps_cov_y = gps_cov[4]
                gps_cov_z = gps_cov[8]
            else:
                # GPS不可用时填充空值
                gps_lat = gps_lon = gps_alt = 0.0
                gps_status = -1
                gps_cov_x = gps_cov_y = gps_cov_z = 999.0

            # 写入CSV
            self.csv_writer.writerow([
                ts_sec, ts_nsec,
                lidar_pos.x, lidar_pos.y, lidar_pos.z,
                lidar_ori.x, lidar_ori.y, lidar_ori.z, lidar_ori.w,
                gps_lat, gps_lon, gps_alt,
                gps_status, gps_cov_x, gps_cov_y, gps_cov_z
            ])
            self.csv_fd.flush()  # 确保数据写入磁盘

    def print_statistics(self):
        """打印统计信息"""
        self.get_logger().info(
            f'记录统计 - 雷达: {self.lidar_count}, GPS: {self.gps_count} '
            f'(固定解: {self.gps_fixed_count})'
        )

        if self.gps_count == 0:
            self.get_logger().warn('未收到GPS数据，将只记录雷达轨迹（可在室内/无GPS环境使用）')

    def __del__(self):
        """析构时关闭文件"""
        if hasattr(self, 'csv_fd') and self.csv_fd:
            self.csv_fd.close()
            self.get_logger().info(f'轨迹已保存至: {self.csv_file}')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到停止信号，保存轨迹文件...')
    finally:
        if hasattr(node, 'csv_fd') and node.csv_fd:
            node.csv_fd.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
