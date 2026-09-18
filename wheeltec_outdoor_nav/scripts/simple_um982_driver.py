#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简单的 UM982 RTK 驱动测试节点
直接读取串口并尝试解析 Unicore 二进制协议
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped
import serial
import struct
import time

class SimpleUM982Driver(Node):
    def __init__(self):
        super().__init__('simple_um982_driver')

        # 参数
        self.declare_parameter('port', '/dev/ttyACM3')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        # 发布器
        self.fix_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.heading_pub = self.create_publisher(Vector3Stamped, '/gps/euler', 10)

        # 打开串口
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'✓ 成功打开串口: {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'✗ 无法打开串口: {e}')
            raise

        # 统计
        self.total_bytes = 0
        self.valid_messages = 0

        # 定时器
        self.timer = self.create_timer(0.1, self.read_callback)
        self.stats_timer = self.create_timer(5.0, self.print_stats)

        self.get_logger().info('RTK 驱动已启动，等待数据...')

    def read_callback(self):
        """读取串口数据"""
        try:
            data = self.ser.read(100)
            if len(data) > 0:
                self.total_bytes += len(data)
                # 这里应该解析 Unicore 二进制协议
                # 由于协议比较复杂，我们先简单输出原始数据
                pass
        except Exception as e:
            self.get_logger().error(f'读取串口错误: {e}')

    def print_stats(self):
        """打印统计信息"""
        self.get_logger().info(
            f'统计: 接收 {self.total_bytes} 字节, '
            f'有效消息 {self.valid_messages} 条'
        )

        if self.total_bytes > 0:
            self.get_logger().info('✓ RTK 模块有数据输出')
        else:
            self.get_logger().warn('✗ 未收到任何数据')

    def __del__(self):
        if hasattr(self, 'ser'):
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)

    try:
        node = SimpleUM982Driver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'错误: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
