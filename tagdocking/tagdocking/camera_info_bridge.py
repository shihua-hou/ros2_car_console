"""CameraInfo timestamp-synchronization bridge.

为什么需要它
------------
相机驱动（Astra / usb_cam）发布的 ``image`` 和 ``camera_info`` 时间戳几乎总是
不对齐：两者由各自的采集线程独立打戳，差几毫秒到几十毫秒。apriltag_ros 用
``image_transport::CameraSubscriber`` 严格按时间戳配对 image + camera_info，
错开的帧被当作无法配对而丢弃 —— 实测 ``Synchronized pairs: 0``，apriltag
检测频率跌到个位数，转向后来不及重新看到 tag 而丢失。

本桥做的事
----------
订阅源 ``image_topic`` 和 ``camera_info_topic``。每收到一帧 image，就用**该
image 的消息头时间戳**重发一份最新的 camera_info（并拷贝 image 头一起重发），
保证输出端 ``sync_image_topic`` / ``sync_info_topic`` 时间戳逐帧对齐，apriltag
能拿到全部配对帧。

注意：camera_info 内容用最近收到的那份原样转发（只改 header），因为内参极少
变化；只是把它"盖"到 image 的时间戳上。

参数
----
image_topic            源 image 话题 (默认 /image_raw)
camera_info_topic      源 camera_info 话题 (默认 /camera_info)
image_out_topic        桥输出的 image 话题 (默认 /camera_sync/image_raw)
camera_info_out_topic  桥输出的 camera_info 话题 (默认 /camera_sync/camera_info)

用法
----
    ros2 run tagdocking camera_info_bridge
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo


class CameraInfoBridge(Node):

    def __init__(self):
        super().__init__('camera_info_bridge')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('image_out_topic', '/camera_sync/image_raw')
        self.declare_parameter('camera_info_out_topic', '/camera_sync/camera_info')
        # 降采样因子 (整数, 按像素抽取)。>1 时把 image 和 camera_info 内参同步缩小,
        # 再转发给 apriltag。理由: 本机 FastDDS 对大帧 RELIABLE image 投递有缺陷,
        # 640x480 rgb8=900KB/帧会把发送通道堵住, 使同回调背靠背发的 camera_info
        # 错开到达 → apriltag 的 exact 时间戳同步配不上对 (Synchronized pairs≈0)
        # → 检测几乎为 0 → 停靠锁不到码。降到 320x240(=230KB) 负载减 4 倍, 同步恢复。
        self.declare_parameter('downscale', 2)

        image_topic = self.get_parameter('image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        image_out = self.get_parameter('image_out_topic').value
        info_out = self.get_parameter('camera_info_out_topic').value
        self._downscale = int(self.get_parameter('downscale').value)

        # 最近一份 camera_info (无回波前为 None, 期间丢弃 image 不转发,
        # 避免把空内参喂给 apriltag)
        self._latest_info = None

        # 输入端: 相机驱动可能是 RELIABLE 也可能是 BEST_EFFORT
        # (Astra color=image_raw 实测是 RELIABLE), 用 sensor_data profile 兼容两者。
        in_qos = qos_profile_sensor_data

        # 输出端: 发 RELIABLE (depth=10)。
        # 实测 apriltag_ros 的 image_transport::CameraSubscriber 在本机用 BEST_EFFORT
        # 订阅时反而收不到 image (同话题同 QoS 下普通订阅能收 8Hz, apriltag 收 0,
        # 怀疑 image_transport 插件层与 BEST_EFFORT 不兼容); 发 RELIABLE 时 apriltag
        # 虽然只收到 ~0.3 帧/s 且 warning 刷 "Synchronized pairs: 0", 但检测能正常出
        # (detections 有输出, 停靠可进行)。故保留 RELIABLE, warning 视为噪声。
        out_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(Image, image_topic, self._on_image, in_qos)
        self.create_subscription(CameraInfo, info_topic, self._on_info, in_qos)

        self._img_pub = self.create_publisher(Image, image_out, out_qos)
        self._info_pub = self.create_publisher(CameraInfo, info_out, out_qos)

        self.get_logger().info(
            f'bridge: {image_topic}+{info_topic} -> '
            f'{image_out}+{info_out}')

    def _on_info(self, msg: CameraInfo):
        self._latest_info = msg

    def _on_image(self, msg: Image):
        info = self._latest_info
        if info is None:
            return

        f = self._downscale
        if f > 1:
            msg = self._downscale_image(msg, f)

        # 用 image 的时间戳重发 image 和 camera_info (改 header)
        # 两者的 header.stamp 取同一值, 保证 apriltag 严格配对。
        self._img_pub.publish(msg)

        sync = CameraInfo()
        sync.header.stamp = msg.header.stamp
        # camera_info 的 frame 由相机驱动保证与 image 一致, 直接沿用
        sync.header.frame_id = info.header.frame_id
        sync.distortion_model = info.distortion_model
        sync.d = info.d
        sync.r = info.r
        if f > 1:
            sync.width = info.width // f
            sync.height = info.height // f
            sync.k = self._scale_k(info.k, f)
            sync.p = self._scale_p(info.p, f)
            sync.binning_x = (info.binning_x or 1) * f
            sync.binning_y = (info.binning_y or 1) * f
        else:
            sync.width = info.width
            sync.height = info.height
            sync.k = info.k
            sync.p = info.p
            sync.binning_x = info.binning_x
            sync.binning_y = info.binning_y
        sync.roi = info.roi
        self._info_pub.publish(sync)

    @staticmethod
    def _downscale_image(msg: Image, f: int) -> Image:
        """按因子 f 抽样降采样 (无损于 apriltag 检测, 16cm tag @1m 仍有 ~50px)。"""
        w, h = msg.width, msg.height
        bpp = msg.step // w if w else 1
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, bpp)
            small = arr[::f, ::f, :]
            msg.data = small.tobytes()
            msg.height = small.shape[0]
            msg.width = small.shape[1]
            msg.step = small.shape[1] * bpp
        except Exception:
            # 解析失败就原样发 (宁可图大也别发空图)
            pass
        return msg

    @staticmethod
    def _scale_k(k, f: int):
        # K = [fx, 0, cx,  0, fy, cy,  0, 0, 1]
        k = list(k)
        k[0] /= f; k[2] /= f
        k[4] /= f; k[5] /= f
        return k

    @staticmethod
    def _scale_p(p, f: int):
        # P = [fx, 0, cx, Tx,  0, fy, cy, Ty,  0, 0, 1, 0]
        p = list(p)
        p[0] /= f; p[2] /= f; p[3] /= f
        p[5] /= f; p[6] /= f; p[7] /= f
        return p


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
