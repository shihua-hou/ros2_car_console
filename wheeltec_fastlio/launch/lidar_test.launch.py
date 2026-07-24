"""
MID360 雷达单独测试: 只启动驱动, 发布 PointCloud2 到 /livox/lidar
用 ros2 topic hz /livox/lidar 检查数据是否正常
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('wheeltec_fastlio')
    return LaunchDescription([
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=[
                {'xfer_format': 0},
                {'multi_topic': 0},
                {'data_src': 0},
                {'publish_freq': 10.0},
                {'output_data_type': 0},
                {'frame_id': 'livox_frame'},
                {'user_config_path': os.path.join(pkg_dir, 'config', 'MID360s_config.json')},
            ],
        ),
    ])
