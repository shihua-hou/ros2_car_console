"""
将 FAST-LIO2 保存的 3D 点云 (FAST_LIO/PCD/scans.pcd) 转换为 2D 导航地图 WHEELTEC3D
地图保存到 wheeltec_nav2 的 map 目录(install和src各一份), 供导航使用。

转换时自动 RANSAC 拟合地面并校平, 高度带以地面为 z=0:
  min_z: 障碍物带下限(地面上方), 默认 0.10
  max_z: 障碍物带上限, 默认 0.45 (= 小车可通行高度: 高于此的桌沿/抽屉不算障碍,
         小车能从下面过; 若小车整高超过45cm请调大)

  2026-08-06 备注: 当天雷达曾下倾42°, 那时后方视野翻上天花板, /scan 被迫改用
  0.15~2.0m 高带补方位覆盖, 本文件的 max_z 也跟着提到 2.0/1.2(地图必须和 /scan
  同带, 否则似然场会把差集当失配)。**雷达改回水平后已全部退回 0.10/0.45**。
  另注: pcd2pgm 的图幅是按障碍带里的点算包围盒的, 改 max_z 会连带改变图幅大小。

用法:
  ros2 launch wheeltec_fastlio save_map.launch.py
  ros2 launch wheeltec_fastlio save_map.launch.py min_z:=0.15 max_z:=0.8
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav2_map_dir = os.path.join(get_package_share_directory('wheeltec_nav2'), 'map')
    src_map_dir = '/home/cat/wheeltec_ros2/src/wheeltec_robot_nav2/map'

    # pcd_file 留空 = 自动合并 PCD 目录下所有 scans*.pcd 分段
    pcd_file = LaunchConfiguration('pcd_file', default='')
    min_z = LaunchConfiguration('min_z', default='0.10')
    max_z = LaunchConfiguration('max_z', default='0.45')
    # 雷达安装角, 必须与 mapping.launch.py 一致(点云所在的 camera_init 系跟着雷达歪)
    # lidar_z: 传给 pcd2pgm 直接定地面高度(不信点云拟合值)。必须与 mapping.launch.py 一致。
    lidar_z = LaunchConfiguration('lidar_z', default='0.28')
    lidar_pitch = LaunchConfiguration('lidar_pitch', default='0.292')
    lidar_roll = LaunchConfiguration('lidar_roll', default='0.006')
    resolution = LaunchConfiguration('resolution', default='0.05')
    map_name = LaunchConfiguration('map_name', default='WHEELTEC3D')

    pcd2pgm_node = Node(
        package='wheeltec_fastlio',
        executable='pcd2pgm',
        output='screen',
        parameters=[{
            'pcd_file': pcd_file,
            'pcd_dir': '/home/cat/wheeltec_ros2/src/FAST_LIO/PCD',
            'map_dir': nav2_map_dir,
            'backup_dir': src_map_dir,
            'map_name': map_name,
            'resolution': resolution,
            'min_z': min_z,
            'max_z': max_z,
            'lidar_z': lidar_z,
            'lidar_pitch': lidar_pitch,
            'lidar_roll': lidar_roll,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('pcd_file', default_value='',
            description='留空则自动合并PCD目录下所有scans*.pcd'),
        DeclareLaunchArgument('min_z', default_value='0.10'),
        DeclareLaunchArgument('max_z', default_value='0.45'),
        DeclareLaunchArgument('lidar_z', default_value='0.28',
            description='雷达光心离地高度(米), 须与 mapping.launch.py 一致; 直接用于定地面高度'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.292',
            description='雷达俯仰角(弧度), 须与 mapping.launch.py 一致; 水平安装填0'),
        DeclareLaunchArgument('lidar_roll', default_value='0.006',
            description='雷达横滚角(弧度), 须与 mapping.launch.py 一致'),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('map_name', default_value='WHEELTEC3D',
            description='保存的地图文件名(不含扩展名)'),
        pcd2pgm_node,
    ])
