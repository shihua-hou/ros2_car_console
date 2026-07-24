"""
N10Plus 2D激光雷达导航 (复用 wheeltec_nav2 框架 + wheeltec_fastlio 的
auto_relocalize扫描匹配重定位节点——该节点只依赖通用的 /scan 和 /map,
与地图是怎么建出来的无关, 因此MID360和N10Plus两种模式可以共用同一份重定位逻辑)

地图: 由 mapping.launch.py + slam_toolbox 保存服务生成的 N10PLUS_MAP
(与 MID360 的 WHEELTEC3D 是两份独立地图, 互不覆盖)

【硬件到货前状态说明】同 mapping.launch.py, 未接实际硬件测试过。

用法:
  ros2 launch wheeltec_n10plus navigation.launch.py
  ros2 launch wheeltec_n10plus navigation.launch.py map:=/path/to/other.yaml
"""
import glob
import os
import subprocess
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

EXT_PATTERN = ('component_container_isolated|livox_ros_driver2_node|'
               'wheeltec_robot_node|fastlio_mapping|lslidar_driver_node|'
               'async_slam_toolbox_node')


def _abort_if_already_running():
    probe = subprocess.run(['pgrep', '-af', EXT_PATTERN],
                           capture_output=True, text=True)
    lines = [l for l in probe.stdout.strip().splitlines() if l]
    if lines:
        raise RuntimeError(
            '\n检测到已有建图/导航实例正在运行(MID360或N10Plus), 拒绝重复启动!\n'
            '冲突进程:\n  ' + '\n  '.join(lines[:6]) +
            '\n请先在原终端 Ctrl+C 关闭它, 或执行 ~/stop_nav.sh。')


def _clean_stale_dds_shm():
    probe = subprocess.run(['pgrep', '-f', EXT_PATTERN],
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


def generate_launch_description():
    _abort_if_already_running()
    _clean_stale_dds_shm()
    pkg_dir = get_package_share_directory('wheeltec_n10plus')
    fastlio_pkg_dir = get_package_share_directory('wheeltec_fastlio')
    wheeltec_launch_dir = os.path.join(
        get_package_share_directory('turn_on_wheeltec_robot'), 'launch')
    nav_dir = get_package_share_directory('wheeltec_nav2')
    nav_launch_dir = os.path.join(nav_dir, 'launch')

    cfg_params = yaml.safe_load(open(os.path.join(
        get_package_share_directory('turn_on_wheeltec_robot'),
        'config', 'wheeltec_param.yaml')))
    car_mode = cfg_params['car_mode']
    print(f'car_mode: {car_mode}')

    map_file = LaunchConfiguration('map',
        default=os.path.join(nav_dir, 'map', 'N10PLUS_MAP.yaml'))
    param_file = LaunchConfiguration('params',
        default=os.path.join(nav_dir, 'param', 'wheeltec_params',
                             f'param_{car_mode}.yaml'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # N10Plus 安装位置, 必须与 mapping.launch.py 里的值完全一致
    # 2026-07-22 实测: 轴心前方4cm, 离地20cm
    lidar_x = LaunchConfiguration('lidar_x', default='0.04')
    lidar_y = LaunchConfiguration('lidar_y', default='0.0')
    lidar_z = LaunchConfiguration('lidar_z', default='0.2')
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

    # 导航需要完整360°扫描: n10plus.yaml 里屏蔽了正后方90°(135°~225°),
    # 那是给"建图时人跟在车正后方推车"用的; 导航时若屏蔽后方, 倒车看不见
    # 后面的障碍物容易撞。这里覆盖掉屏蔽区间(建图launch不受影响, 仍屏蔽后方)。
    n10plus_driver = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        namespace='x10',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'n10plus.yaml'),
                    {'angle_disable_min': [0], 'angle_disable_max': [0]}],
    )

    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_n10plus_tf',
        arguments=['--x', lidar_x, '--y', lidar_y, '--z', lidar_z,
                   '--yaw', lidar_yaw, '--pitch', '0', '--roll', '0',
                   '--frame-id', 'base_footprint', '--child-frame-id', 'n10plus_laser'],
    )

    waypoint_cycle = Node(
        name='waypoint_cycle',
        package='nav2_waypoint_cycle',
        executable='nav2_waypoint_cycle',
    )

    # 复用 wheeltec_fastlio 的通用扫描匹配重定位节点(与传感器来源无关)
    auto_relocalize = Node(
        package='wheeltec_fastlio',
        executable='auto_relocalize',
        name='auto_relocalize',
        output='screen',
        parameters=[{
            'auto_on_startup': True,
            'accept_score': 0.55,
            'watchdog_en': True,
            'watchdog_score': 0.4,
        }],
    )

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
            default_value=os.path.join(nav_dir, 'map', 'N10PLUS_MAP.yaml'),
            description='导航地图yaml路径'),
        DeclareLaunchArgument('params',
            default_value=os.path.join(nav_dir, 'param', 'wheeltec_params',
                                       f'param_{car_mode}.yaml'),
            description='nav2参数文件'),
        DeclareLaunchArgument('lidar_x', default_value='0.04'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.2'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),
        imu_processor,
        wheeltec_robot,
        n10plus_driver,
        lidar_tf,
        waypoint_cycle,
        auto_relocalize,
        nav2_bringup,
    ])
