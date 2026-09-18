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
from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    # 局部地面网格: 坡地上必须开(默认true)。传 false 退回原来的全局单平面,
    # 只在'确认场地是严格平地、且怀疑网格插值有问题'时才需要。
    local_ground = LaunchConfiguration('local_ground', default='true')
    ground_cell = LaunchConfiguration('ground_cell', default='2.0')
    # 一格里至少几个点才算障碍。户外远处地面的稀疏掠射回波会撒成散点红斑,
    # 调高可滤掉(实测 5->障碍1.9%, 25->障碍0.3%)。调太高真实结构也会被抹掉。
    occupied_min_points = LaunchConfiguration('occupied_min_points', default='5')
    # 只把离建图轨迹这么近的点画进图; 0=不裁。户外远处掠射点又稀又不可靠,
    # 裁掉能让地图聚焦在真正走过的范围内(远点仍照常参与 FAST-LIO 里程计)。
    traj_crop_radius = LaunchConfiguration('traj_crop_radius', default='0.0')

    # 存完图自动做 GPS 地理配准, 结果写成 <地图名>.gps.yaml 放在地图旁边。
    # 默认关: 室内没有 GPS, 也就没有轨迹 CSV。webapp 户外存图时传 true。
    georef = LaunchConfiguration('georef', default='false')

    pcd2pgm_node = Node(
        package='wheeltec_fastlio',
        executable='pcd2pgm',
        output='screen',
        parameters=[{
            'pcd_file': pcd_file,
            'pcd_dir': '/home/cat/wheeltec_ros2/src/FAST_LIO/PCD',
            'map_dir': nav2_map_dir,
            'backup_dir': src_map_dir,
            # 必须强制成字符串: 地图名是纯数字时(如 2026091402), ros2 launch 会把它
            # 写成 YAML 整数, pcd2pgm 声明的是 string 参数 -> InvalidParameterType
            # 异常 -> SIGABRT, 表现为'存图一点就崩'(2026-09-14 实际踩到)。
            'map_name': ParameterValue(map_name, value_type=str),
            'local_ground': ParameterValue(local_ground, value_type=bool),
            'ground_cell': ParameterValue(ground_cell, value_type=float),
            'occupied_min_points': ParameterValue(occupied_min_points,
                                                 value_type=int),
            'traj_crop_radius': ParameterValue(traj_crop_radius,
                                               value_type=float),
            'resolution': resolution,
            'min_z': min_z,
            'max_z': max_z,
            'lidar_z': lidar_z,
            'lidar_pitch': lidar_pitch,
            'lidar_roll': lidar_roll,
        }],
    )

    # 依赖 wheeltec_outdoor_nav —— 但只在 georef:=true 时才会去启动它,
    # 所以室内单独用 wheeltec_fastlio 不受影响。
    # 用 ExecuteProcess 而不是 Node: georef_map.py 是普通脚本(不用 rclpy),
    # 而 launch_ros 的 Node 会无条件追加 --ros-args, argparse 见到就报错退出。
    georef_script = os.path.join(get_package_prefix('wheeltec_outdoor_nav'),
                                 'lib', 'wheeltec_outdoor_nav', 'georef_map.py')
    georef_node = ExecuteProcess(
        condition=IfCondition(georef),
        cmd=['python3', georef_script,
             '--map-dir', nav2_map_dir,
             '--also-dir', src_map_dir,
             '--map-name', map_name],
        output='screen',
    )

    # 必须等 pcd2pgm 退出: 地图和 .tf.yaml 都写完了, georef 才有东西可读
    georef_after_save = RegisterEventHandler(
        OnProcessExit(target_action=pcd2pgm_node, on_exit=[georef_node]))

    return LaunchDescription([
        DeclareLaunchArgument('pcd_file', default_value='',
            description='留空则自动合并PCD目录下所有scans*.pcd'),
        DeclareLaunchArgument('min_z', default_value='0.10'),
        DeclareLaunchArgument('max_z', default_value='0.45'),
        DeclareLaunchArgument('local_ground', default_value='true',
            description='true=按网格算局部地面高度(坡地必须开); false=用全局单平面'),
        DeclareLaunchArgument('traj_crop_radius', default_value='0.0',
            description='只保留离建图轨迹这么多米内的点; 0=不裁, 户外建议 20'),
        DeclareLaunchArgument('occupied_min_points', default_value='5',
            description='一格里至少几个点才算障碍; 户外散点多可调到 12~15'),
        DeclareLaunchArgument('ground_cell', default_value='2.0',
            description='局部地面网格边长(米)'),
        DeclareLaunchArgument('lidar_z', default_value='0.28',
            description='雷达光心离地高度(米), 须与 mapping.launch.py 一致; 直接用于定地面高度'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.292',
            description='雷达俯仰角(弧度), 须与 mapping.launch.py 一致; 水平安装填0'),
        DeclareLaunchArgument('lidar_roll', default_value='0.006',
            description='雷达横滚角(弧度), 须与 mapping.launch.py 一致'),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('map_name', default_value='WHEELTEC3D',
            description='保存的地图文件名(不含扩展名)'),
        DeclareLaunchArgument('georef', default_value='false',
            description='存图后自动做 GPS 地理配准(户外), 生成 <地图名>.gps.yaml'),
        pcd2pgm_node,
        georef_after_save,
    ])
