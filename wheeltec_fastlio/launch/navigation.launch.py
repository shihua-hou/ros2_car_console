"""
MID360 3D雷达导航 (基于 wheeltec_nav2 框架)
架构:
  MID360点云 -> pointcloud_to_laserscan -> /scan
  AMCL: map -> odom_combined | EKF(轮式+IMU): odom_combined -> base_footprint
  地图: 由 FAST-LIO2 建图 + save_map.launch.py 生成的 WHEELTEC3D
用法:
  ros2 launch wheeltec_fastlio navigation.launch.py
  ros2 launch wheeltec_fastlio navigation.launch.py map:=/path/to/other.yaml
"""
import glob
import os
import subprocess
import yaml
from ament_index_python.packages import get_package_share_directory


def _clean_stale_dds_shm():
    """清理被强杀进程残留的 FastDDS 共享内存/信号量文件。

    进程被 kill -9 或段错误退出后, /dev/shm 会残留已锁定的
    sem.fastrtps_portXXXX_mutex, 新启动的节点(尤其nav2容器)尝试加锁时
    会在 futex 上永久死锁(表现为100%CPU且无日志)。
    仅在当前没有其它ROS进程运行时清理, 避免误删活动会话的文件。
    """
    probe = subprocess.run(
        ['pgrep', '-f',
         'component_container|livox_ros_driver2|wheeltec_robot_node|'
         'fastlio_mapping|ekf_node|rviz2|pointcloud_to_laserscan'],
        capture_output=True, text=True)
    others = [p for p in probe.stdout.split() if p and int(p) != os.getpid()]
    if others:
        return
    for f in glob.glob('/dev/shm/fastrtps_*') + \
             glob.glob('/dev/shm/sem.fastrtps_*') + \
             glob.glob('/dev/shm/fast_datasharing*'):
        try:
            os.remove(f)
        except OSError:
            pass
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    pkg_dir = get_package_share_directory('wheeltec_fastlio')
    wheeltec_launch_dir = os.path.join(
        get_package_share_directory('turn_on_wheeltec_robot'), 'launch')
    nav_dir = get_package_share_directory('wheeltec_nav2')
    nav_launch_dir = os.path.join(nav_dir, 'launch')

    # 读取车型, 复用 wheeltec_nav2 对应车型的导航参数
    cfg_params = yaml.safe_load(open(os.path.join(
        get_package_share_directory('turn_on_wheeltec_robot'),
        'config', 'wheeltec_param.yaml')))
    car_mode = cfg_params['car_mode']
    print(f'car_mode: {car_mode}')

    map_file = LaunchConfiguration('map',
        default=os.path.join(nav_dir, 'map', 'WHEELTEC3D.yaml'))
    param_file = LaunchConfiguration('params',
        default=os.path.join(nav_dir, 'param', 'wheeltec_params',
                             f'param_{car_mode}.yaml'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 雷达安装位置(相对base_footprint), 需与建图时一致
    lidar_x = LaunchConfiguration('lidar_x', default='-0.20')
    lidar_y = LaunchConfiguration('lidar_y', default='0.0')
    lidar_z = LaunchConfiguration('lidar_z', default='0.34')
    lidar_yaw = LaunchConfiguration('lidar_yaw', default='0.0')

    imu_processor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wheeltec_launch_dir, 'imu_processor.launch.py')),
    )

    wheeltec_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wheeltec_launch_dir, 'turn_on_wheeltec_robot.launch.py')),
        launch_arguments={'carto_slam': 'false', 'robot_nav': 'false'}.items(),
    )

    # MID360 驱动, PointCloud2 格式
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {'xfer_format': 0},      # 0-PointCloud2
            {'multi_topic': 0},
            {'data_src': 0},
            {'publish_freq': 10.0},
            {'output_data_type': 0},
            {'frame_id': 'livox_frame'},
            {'user_config_path': os.path.join(pkg_dir, 'config', 'MID360s_config.json')},
        ],
    )

    # 3D点云 转 2D激光 /scan (取 base_footprint 坐标系下 0.05~0.6m 高度带,
    # 与 save_map 的障碍物带一致: 雷达装高0.2m时 min_z=-0.15 即地面上0.05m)
    cloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[('cloud_in', '/livox/lidar'),
                    ('scan', '/scan')],
        parameters=[{
            'target_frame': 'base_footprint',
            'transform_tolerance': 0.05,
            # min_height 不能太低: 车身加减速俯仰1~2°时, 远处地面点表观高度会
            # 抬升(5m外抬~0.1m), 低于0.15会把地面扫成环状幻影障碍, 导致
            # "Starting point in lethal space" 时好时坏
            'min_height': 0.15,
            'max_height': 0.45,   # 与save_map的max_z一致(小车可通行高度)
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0058,   # ~0.33度
            'scan_time': 0.1,
            'range_min': 0.3,
            'range_max': 30.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
    )

    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_livox_tf',
        arguments=['--x', lidar_x, '--y', lidar_y, '--z', lidar_z,
                   '--yaw', lidar_yaw, '--pitch', '0', '--roll', '0',
                   '--frame-id', 'base_footprint', '--child-frame-id', 'livox_frame'],
    )

    waypoint_cycle = Node(
        name='waypoint_cycle',
        package='nav2_waypoint_cycle',
        executable='nav2_waypoint_cycle',
    )

    # 扫描匹配自动重定位: 启动后自动定位(无需2D Pose Estimate),
    # 定位丢失可手动触发: ros2 service call /relocalize std_srvs/srv/Trigger
    auto_relocalize = Node(
        package='wheeltec_fastlio',
        executable='auto_relocalize',
        name='auto_relocalize',
        output='screen',
        parameters=[{
            'auto_on_startup': True,
            'accept_score': 0.55,
            'watchdog_en': True,     # 绑架检测: 位姿匹配分持续过低时自动全局重定位
            'watchdog_score': 0.4,   # 严格分(σ=0.08)阈值: 正确位姿0.6~0.8, 被搬动0.1~0.2
        }],
    )

    # Nav2 (AMCL+planner+controller等), 复用wheeltec现有配置
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': param_file}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map',
            default_value=os.path.join(nav_dir, 'map', 'WHEELTEC3D.yaml'),
            description='导航地图yaml路径'),
        DeclareLaunchArgument('params',
            default_value=os.path.join(nav_dir, 'param', 'wheeltec_params',
                                       f'param_{car_mode}.yaml'),
            description='nav2参数文件'),
        DeclareLaunchArgument('lidar_x', default_value='-0.20'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.34'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),
        imu_processor,
        wheeltec_robot,
        livox_driver,
        cloud_to_scan,
        lidar_tf,
        waypoint_cycle,
        auto_relocalize,
        nav2_bringup,
    ])
