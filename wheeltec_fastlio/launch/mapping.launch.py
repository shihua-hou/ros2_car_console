"""
MID360 + FAST-LIO2 3D建图
用法:
  ros2 launch wheeltec_fastlio mapping.launch.py            # 无rviz(推荐在小车上)
  ros2 launch wheeltec_fastlio mapping.launch.py rviz:=true # 带rviz
建图完成后 Ctrl+C 退出, 点云自动保存到 FAST_LIO/PCD/scans.pcd,
然后运行: ros2 launch wheeltec_fastlio save_map.launch.py 生成2D导航地图
"""
import glob
import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

PCD_DIR = '/home/cat/wheeltec_ros2/src/FAST_LIO/PCD'


def _clean_stale_dds_shm():
    """清理被强杀进程残留的 FastDDS 共享内存/信号量文件(死锁毒源)。
    仅在当前没有其它ROS进程运行时清理。详见 navigation.launch.py 同名函数。"""
    probe = subprocess.run(
        ['pgrep', '-f',
         'component_container|livox_ros_driver2|wheeltec_robot_node|'
         'fastlio_mapping|ekf_node|rviz2|pointcloud_to_laserscan'],
        capture_output=True, text=True)
    if probe.stdout.strip():
        return
    for f in glob.glob('/dev/shm/fastrtps_*') + \
             glob.glob('/dev/shm/sem.fastrtps_*') + \
             glob.glob('/dev/shm/fast_datasharing*'):
        try:
            os.remove(f)
        except OSError:
            pass


def _abort_if_already_running():
    """检测到已有建图/导航实例时拒绝启动。
    双实例会把组件加载进同名 nav2 容器、串口/雷达冲突、CPU 过载, 系统必乱。"""
    probe = subprocess.run(
        ['pgrep', '-af',
         'component_container_isolated|livox_ros_driver2_node|'
         'wheeltec_robot_node|fastlio_mapping'],
        capture_output=True, text=True)
    lines = [l for l in probe.stdout.strip().splitlines() if l]
    if lines:
        raise RuntimeError(
            '\n检测到已有建图/导航实例正在运行, 拒绝重复启动!\n'
            '冲突进程:\n  ' + '\n  '.join(lines[:6]) +
            '\n请先在原终端 Ctrl+C 关闭它; 如果关不掉, 执行:\n'
            '  pkill -9 -f "component_container_isolated|livox_ros_driver2_node"\n'
            '然后重新启动本 launch。')


def generate_launch_description():
    _abort_if_already_running()
    _clean_stale_dds_shm()
    # 清理上次建图的分段点云, 防止 save_map 时新旧数据混在一起
    # (如需保留上次的3D点云, 启动建图前请自行备份 PCD 目录)
    for f in glob.glob(os.path.join(PCD_DIR, 'scans*.pcd')):
        os.remove(f)
    pkg_dir = get_package_share_directory('wheeltec_fastlio')
    wheeltec_launch_dir = os.path.join(
        get_package_share_directory('turn_on_wheeltec_robot'), 'launch')

    rviz_use = LaunchConfiguration('rviz', default='false')
    # 雷达在小车上的安装位置(相对base_footprint), 按实际安装修改默认值
    lidar_x = LaunchConfiguration('lidar_x', default='-0.20')
    lidar_y = LaunchConfiguration('lidar_y', default='0.0')
    lidar_z = LaunchConfiguration('lidar_z', default='0.34')
    lidar_yaw = LaunchConfiguration('lidar_yaw', default='0.0')

    # 小车底盘(串口+EKF: odom_combined->base_footprint)
    wheeltec_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wheeltec_launch_dir, 'turn_on_wheeltec_robot.launch.py')),
        launch_arguments={'carto_slam': 'false', 'robot_nav': 'false'}.items(),
    )

    # MID360 驱动, CustomMsg 格式(FAST-LIO 专用) /livox/lidar /livox/imu
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {'xfer_format': 1},      # 1-Livox CustomMsg (FAST-LIO 输入格式)
            {'multi_topic': 0},
            {'data_src': 0},
            {'publish_freq': 10.0},
            {'output_data_type': 0},
            {'frame_id': 'livox_frame'},
            {'user_config_path': os.path.join(pkg_dir, 'config', 'MID360s_config.json')},
        ],
    )

    # FAST-LIO2 建图, TF: camera_init -> body
    fast_lio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'wheeltec_mid360.yaml'),
            {'use_sim_time': False},
        ],
    )

    # 雷达安装位置静态TF
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_livox_tf',
        arguments=['--x', lidar_x, '--y', lidar_y, '--z', lidar_z,
                   '--yaw', lidar_yaw, '--pitch', '0', '--roll', '0',
                   '--frame-id', 'base_footprint', '--child-frame-id', 'livox_frame'],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'fastlio.rviz')],
        condition=IfCondition(rviz_use),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('lidar_x', default_value='-0.20'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.34'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),
        wheeltec_robot,
        livox_driver,
        fast_lio,
        lidar_tf,
        rviz_node,
    ])
