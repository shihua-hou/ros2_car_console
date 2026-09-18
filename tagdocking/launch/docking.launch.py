"""Launch file for tagdocking — AprilTag auto-docking framework.

Starts:
  1. april_tag node (tag detection + TF broadcast)
  2. docking_node (tagdocking controller)

Usage:
  # Minimal: camera already publishing, just start docking stack
  ros2 launch tagdocking docking.launch.py

  # With custom tag
  ros2 launch tagdocking docking.launch.py dock_tag_id:=5 tag_size:=0.21

  # Omni wheel mode
  ros2 launch tagdocking docking.launch.py base_type:=omni
"""

import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context):
    # ── Kill stray apriltag_node processes from previous crashed runs ──
    # apriltag_node is a separate process from docking_node; if docking_node
    # crashed (or was SIGKILLed), the apriltag node survived. Multiple stray
    # apriltag nodes each broadcast the SAME tag TF frame (tag<family>:<id>),
    # so the TF listener returns whichever competing transform arrived last —
    # producing wild, contradictory lat/dist jumps between measurements that
    # make docking impossible. Reap them before starting a fresh one.
    try:
        subprocess.run(['pkill', '-9', '-f', 'apriltag_node'],
                       timeout=5, check=False)
    except Exception:
        pass

    image_topic = LaunchConfiguration('image_topic').perform(context)
    camera_info_topic = LaunchConfiguration('camera_info_topic').perform(context)
    family = LaunchConfiguration('family').perform(context)
    tag_size = float(LaunchConfiguration('tag_size').perform(context))
    dock_tag_id = int(LaunchConfiguration('dock_tag_id').perform(context))
    camera_frame = LaunchConfiguration('camera_frame').perform(context)
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic').perform(context)
    base_type = LaunchConfiguration('base_type').perform(context)
    dock_distance = float(LaunchConfiguration('dock_distance').perform(context))
    final_straight_distance = float(LaunchConfiguration('final_straight_distance').perform(context))
    final_straight_yaw_deg = float(LaunchConfiguration('final_straight_yaw_deg').perform(context))

    pkg_share = get_package_share_directory('tagdocking')
    config_path = os.path.join(pkg_share, 'config', 'docking.yaml')

    # AprilTag frame name convention: tag<family>:<id>
    tag_frame = f'tag{family}:{dock_tag_id}'

    # 同步桥输出话题。apriltag_ros 的 image_transport::CameraSubscriber 从 image
    # 话题名同级派生 camera_info (image_raw → camera_info), 所以 apriltag 必须订阅
    # 桥的 image 输出, 才能拿到时间戳对齐的 camera_info。
    sync_image_topic = '/camera_sync/image_raw'
    sync_info_topic = '/camera_sync/camera_info'

    nodes = []

    # ── CameraInfo 同步桥 ────────────────────────────────────────
    # 相机 (usb_cam/astra) 的 image 与 camera_info 时间戳不对齐, apriltag 的严格
    # 时间同步会丢弃几乎所有帧 (Synchronized pairs: 0) → 检测频率极低 → 转向后
    # 来不及重新看到 tag 而丢失。桥每收到一帧 image 就用其时间戳重发 camera_info,
    # 保证每帧都能配对。
    #
    # camera_info_bridge 是 tagdocking 包内节点 (原依赖的独立 autodock 包已废弃,
    # 实现见 tagdocking/camera_info_bridge.py)。
    nodes.append(Node(
        package='tagdocking',
        executable='camera_info_bridge',
        name='camera_info_bridge',
        parameters=[{
            'image_topic': image_topic,
            'camera_info_topic': camera_info_topic,
            'image_out_topic': sync_image_topic,
            'camera_info_out_topic': sync_info_topic,
        }],
        output='screen',
    ))

    # ── AprilTag detection node ─────────────────────────────────
    nodes.append(Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        parameters=[{
            'family': family,
            'size': tag_size,
            'tag_ids': [dock_tag_id],
            'tag_frames': [tag_frame],
            'publish_tf': True,
            'z_up': False,
        }],
        remappings=[
            # 订阅桥的同步输出而非裸 image_raw; camera_info 自动派生到 sync_info_topic
            ('image_rect', sync_image_topic),
        ],
        output='screen',
    ))

    # ── Docking controller ──────────────────────────────────────
    nodes.append(Node(
        package='tagdocking',
        executable='docking_node',
        name='docking_node',
        parameters=[
            config_path,
            {
                'tag.frame': tag_frame,
                'tag.id': dock_tag_id,
                'tag.family': family,
                'tag.size': tag_size,
                'camera_frame': camera_frame,
                'base.cmd_vel_topic': cmd_vel_topic,
                'base.type': base_type,
                # 两阶段停泊参数 (覆盖 yaml)
                'dock_target.distance': dock_distance,
                'final_straight.start_distance': final_straight_distance,
                'final_straight.yaw_threshold_deg': final_straight_yaw_deg,
            },
        ],
        output='screen',
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/image_raw',
                             description='Camera image topic for apriltag_ros'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera_info',
                             description='Camera info topic (source for sync bridge)'),
        DeclareLaunchArgument('family', default_value='36h11',
                             description='AprilTag family (36h11, 25h9, etc.)'),
        DeclareLaunchArgument('tag_size', default_value='0.16',
                             description='Tag edge size in meters'),
        DeclareLaunchArgument('dock_tag_id', default_value='0',
                             description='Tag ID to dock to'),
        DeclareLaunchArgument('camera_frame',
                             default_value='camera_color_optical_frame',
                             description='Camera optical frame name'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel',
                             description='Velocity command topic'),
        DeclareLaunchArgument('base_type', default_value='diff_drive',
                             description='Chassis type: diff_drive, omni, quadruped'),
        DeclareLaunchArgument('dock_distance', default_value='0.55',
                             description='最终停泊距离 (m), 底盘距 tag'),
        DeclareLaunchArgument('final_straight_distance', default_value='0.85',
                             description='直行阶段起点距离 (m), 到此距离后纯直行不再调角 (须 > dock_distance)'),
        DeclareLaunchArgument('final_straight_yaw_deg', default_value='5.0',
                             description='进入直行阶段的航向门槛 (deg, 方阵误差)'),
        OpaqueFunction(function=launch_setup),
    ])
