# coding=utf-8
import sys
import math
import traceback

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import UInt8
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler, euler_from_quaternion


from wheeltec_dual_rtk_driver.um982_serial import UM982Serial, gga_quality_to_guard_status
from geometry_msgs.msg import Vector3Stamped


class wheeltec_dual_rtk_driver(Node):
    def _ros_log_debug(self, log_data):
        self.get_logger().debug(str(log_data))

    def _ros_log_info(self, log_data):
        self.get_logger().info(str(log_data))

    def _ros_log_warn(self, log_data):
        self.get_logger().warn(str(log_data))

    def _ros_log_error(self, log_data):
        self.get_logger().error(str(log_data))


    def __init__(self) -> None:
        super().__init__('um982_serial_driver')
        global gps_frame_id

        # Step1：从参数服务器获取port和baud
        self.declare_parameter('port', '/dev/wheeltec_gnss')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('gps_frame_id', 'navsat_link')
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        gps_frame_id = self.get_parameter('gps_frame_id').get_parameter_value().string_value
        # Step2：打开串口
        try:
            self.um982serial = UM982Serial(port, baud)
            self._ros_log_info(f'serial {port} open successfully!')
        except Exception:
            self._ros_log_error(f'serial {port} do not open! (port={port}, baud={baud})')
            self._ros_log_error(traceback.format_exc())
            sys.exit(0)
        # Step3：新建一个线程用于处理串口数据
        self.um982serial.start()
        # Step4：ROS相关
        self.fix_pub        = self.create_publisher(NavSatFix, '/gps/fix',     10)
        self.utm_pub        = self.create_publisher(Odometry,  '/gps/utm_pose',  10)
        self.euler_pub      = self.create_publisher(Vector3Stamped, '/gps/euler', 10)  
        # 原始 GGA 质量位(0无效/1单点/2差分/4RTK固定/5RTK浮点/6推算)。
        # NavSatFix.status 只有4档, 塞不下, 界面要准确显示就得靠这个。
        self.quality_pub    = self.create_publisher(UInt8, '/gps/quality', 10)

        self.pub_timer      = self.create_timer(1/20, self.pub_task)

    def pub_task(self):
        # fix / orientation / vel 分别来自 GNGGA / GNHPR / #BESTNAVA 三种不同报文，
        # 到达时机互相独立（#BESTNAVA 在当前模块配置下可能一直不会出现），
        # 因此三个话题的发布各自独立判空，不能整体等齐再发，否则 fix 会被 vel 卡住永远发不出去。
        this_time = self.get_clock().now().to_msg()

        if self.um982serial.fix is not None:
            bestpos_hgt, bestpos_lat, bestpos_lon, bestpos_hgtstd, bestpos_latstd, bestpos_lonstd = self.um982serial.fix

            # Step 1: Publish GPS Fix Data
            fix_msg = NavSatFix()
            fix_msg.header.stamp = this_time
            fix_msg.header.frame_id = gps_frame_id
            fix_msg.latitude = bestpos_lat
            fix_msg.longitude = bestpos_lon
            fix_msg.altitude = bestpos_hgt
            fix_msg.status.status = gga_quality_to_guard_status(self.um982serial.fix_quality)
            fix_msg.status.service = NavSatStatus.SERVICE_GPS
            fix_msg.position_covariance[0] = float(bestpos_latstd)**2
            fix_msg.position_covariance[4] = float(bestpos_lonstd)**2
            fix_msg.position_covariance[8] = float(bestpos_hgtstd)**2
            fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self.fix_pub.publish(fix_msg)
            self.quality_pub.publish(UInt8(data=int(self.um982serial.fix_quality or 0)))

        if self.um982serial.orientation is not None:
            heading, pitch, roll = self.um982serial.orientation
            euler_msg = Vector3Stamped()
            euler_msg.header.stamp = this_time
            euler_msg.header.frame_id = 'euler_link'
            euler_msg.vector.x = roll     # x轴表示roll（横滚）
            euler_msg.vector.y = pitch    # y轴表示pitch（俯仰）
            euler_msg.vector.z = heading  # z轴表示heading（偏航）
            self.euler_pub.publish(euler_msg)

        # Step 2: Publish UTM Position Data（需要 fix + orientation + vel 三者齐全）
        # utmpos 由读串口线程在第一次定位后算出, 和 fix 之间有极短的空窗
        if self.um982serial.fix is not None and self.um982serial.orientation is not None \
                and self.um982serial.vel is not None and self.um982serial.utmpos is not None:
            bestpos_hgt, bestpos_lat, bestpos_lon, bestpos_hgtstd, bestpos_latstd, bestpos_lonstd = self.um982serial.fix
            utm_x, utm_y = self.um982serial.utmpos
            vel_east, vel_north, vel_ver, vel_east_std, vel_north_std, vel_ver_std = self.um982serial.vel
            heading, pitch, roll = self.um982serial.orientation

            odom_msg = Odometry()
            odom_msg.header.stamp = this_time
            odom_msg.header.frame_id = 'earth'
            odom_msg.child_frame_id  = 'base_link'
            odom_msg.pose.pose.position.x = utm_x
            odom_msg.pose.pose.position.y = utm_y
            odom_msg.pose.pose.position.z = bestpos_hgt
            quaternion = quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(heading))
            odom_msg.pose.pose.orientation.x = quaternion[0]
            odom_msg.pose.pose.orientation.y = quaternion[1]
            odom_msg.pose.pose.orientation.z = quaternion[2]
            odom_msg.pose.pose.orientation.w = quaternion[3]
            odom_msg.pose.covariance         = [0.0] * 36
            odom_msg.pose.covariance[0]      = float(bestpos_latstd)**2
            odom_msg.pose.covariance[7]      = float(bestpos_lonstd)**2
            odom_msg.pose.covariance[14]     = float(bestpos_hgtstd)**2
            odom_msg.pose.covariance[21]     = 0.1
            odom_msg.pose.covariance[28]     = 0.1
            odom_msg.pose.covariance[35]     = 0.1
            odom_msg.twist.twist.linear.x    = vel_east
            odom_msg.twist.twist.linear.y    = vel_north
            odom_msg.twist.twist.linear.z    = vel_ver
            odom_msg.twist.covariance        = [0.0] * 36
            odom_msg.twist.covariance[0]     = float(vel_east_std)**2
            odom_msg.twist.covariance[7]     = float(vel_north_std)**2
            odom_msg.twist.covariance[14]    = float(vel_ver_std)**2
            self.utm_pub.publish(odom_msg)


    def run(self):
        if rclpy.ok():
            rclpy.spin(self)

    def stop(self):
        self.um982serial.stop()
        self.pub_timer.cancel()



import time
import signal

def signal_handler(sig, frame):
    # 构造函数里要等串口数据(最多 15 秒), 这期间收到 SIGINT 时 dual_rtk_driver
    # 还没赋值 —— 2026-09-22 日志里的 NameError 就是这个, 那就直接退出
    drv = globals().get('dual_rtk_driver')
    if drv is not None:
        drv.stop()
    time.sleep(0.1)
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)
rclpy.init()
dual_rtk_driver = wheeltec_dual_rtk_driver()


def main():
    dual_rtk_driver.run()

if __name__ == "__main__":
    main()
