"""
户外导航 Launch 文件

两种定位模式(localization 参数):

  gps (默认)  RTK 主定位。全局 EKF 融合 轮速 + IMU + GPS(位置+双天线航向),
              发布 map -> odom_combined。AMCL 仍在跑但不发 TF(只出 /amcl_pose
              供对照), 雷达只负责局部避障, 地图只负责全局规划。
  amcl        老方案。AMCL 点云配准定位 + GPS 守卫救援。

为什么默认 gps(2026-09-16 实测):
  AMCL 在人群中一帧跳 13.99m, 自报协方差却从 0.23m 变成 0.24m —— 它不知道自己
  错了, 协方差阈值的守卫也就永远不会触发。同期 GPS 换算到地图系的位置零跳变,
  与健康时的 AMCL 吻合在 0.22~0.41m。

gps 模式的前提是地图有配套的 <地图名>.gps.yaml(户外存图时自动生成)。
**没有就自动退回 amcl 模式**, 并打警告 —— 否则全局 EKF 没有绝对观测, 会从
原点开始按里程计推, 定位完全是错的。

使用方法:
  ros2 launch wheeltec_outdoor_nav outdoor_navigation.launch.py map:=/path/to/map.yaml
  ros2 launch wheeltec_outdoor_nav outdoor_navigation.launch.py localization:=amcl
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# gps 模式下给 nav2 用的参数文件: 本车型参数的副本, 只关掉 amcl 的 TF。
# 每次启动时重新生成, 这样室内那份改了这里自动跟上, 不会出现两份各改各的。
OUTDOOR_NAV2_PARAMS = '/tmp/wheeltec_outdoor_nav2_params.yaml'


def _car_nav2_params():
    """本车型的 nav2 参数文件(与 navigation.launch.py 的选取逻辑一致)。"""
    car = yaml.safe_load(open(os.path.join(
        get_package_share_directory('turn_on_wheeltec_robot'),
        'config', 'wheeltec_param.yaml')))['car_mode']
    return os.path.join(get_package_share_directory('wheeltec_nav2'),
                        'param', 'wheeltec_params', f'param_{car}.yaml')


def _outdoor_nav2_params():
    """复制本车型 nav2 参数, 只改一处: amcl.tf_broadcast = false。

    map -> odom_combined 这个变换只能有一个发布者。gps 模式下由全局 EKF 发,
    AMCL 必须闭嘴 —— 两个同时发会互相覆盖, TF 树上的位姿来回跳。
    AMCL 本身照常运行(仍出 /amcl_pose), 方便和 GPS 对照、将来也可以作为
    EKF 的一路辅助观测。
    """
    src = _car_nav2_params()
    with open(src, encoding='utf-8') as f:
        p = yaml.safe_load(f)
    p['amcl']['ros__parameters']['tf_broadcast'] = False
    with open(OUTDOOR_NAV2_PARAMS, 'w', encoding='utf-8') as f:
        yaml.safe_dump(p, f, allow_unicode=True, sort_keys=False)
    return src


def _gps_calibration(map_path):
    name = os.path.splitext(os.path.basename(map_path))[0]
    cal = os.path.join(os.path.dirname(map_path), name + '.gps.yaml')
    return cal if os.path.exists(cal) else None


def _setup(context):
    outdoor_pkg = get_package_share_directory('wheeltec_outdoor_nav')
    fastlio_dir = get_package_share_directory('wheeltec_fastlio')

    mode = LaunchConfiguration('localization').perform(context).strip().lower()
    map_path = os.path.expanduser(LaunchConfiguration('map').perform(context))
    guard_on = LaunchConfiguration('enable_gps_guard').perform(context).lower() == 'true'
    actions = []

    if mode not in ('gps', 'amcl'):
        actions.append(LogInfo(
            msg=f'[户外导航] 未知 localization:={mode}, 按 amcl 处理'))
        mode = 'amcl'

    cal = _gps_calibration(map_path)
    if mode == 'gps' and cal is None:
        actions.append(LogInfo(msg=(
            f'[户外导航] ⚠ 地图 {os.path.basename(map_path)} 没有 GPS 标定'
            f'(<地图名>.gps.yaml), 无法用 RTK 定位, 自动退回 AMCL。'
            ' 标定在户外存图时自动生成; 没有说明当次建图 RTK 没到固定解,'
            ' 或固定解期间轨迹近似共线。')))
        mode = 'amcl'

    nav_args = {
        'map': map_path,
        'use_camera': LaunchConfiguration('use_camera').perform(context),
        # 户外障碍带 2.0m, 必须和存图时的 max_z 一致(webapp 户外存图传 2.0)
        'scan_max_z': LaunchConfiguration('scan_max_z').perform(context),
    }
    if mode == 'gps':
        src = _outdoor_nav2_params()
        nav_args['params'] = OUTDOOR_NAV2_PARAMS
        # 点云配准重定位在 RTK 定位下没有意义, 只会反复给出错误坐标并播报
        # "重新定位成功"(2026-09-17 实测 40 秒内三次, 都离 GPS 位置 20m+)
        nav_args['use_auto_relocalize'] = 'false'
        actions.append(LogInfo(msg=(
            f'[户外导航] 定位模式: RTK + 全局 EKF (标定 {os.path.basename(cal)})。'
            f' AMCL 已关闭 TF 发布(参数取自 {os.path.basename(src)})')))
    else:
        actions.append(LogInfo(msg='[户外导航] 定位模式: AMCL'
                                   + (' + GPS 守卫' if guard_on else '')))

    # 1. 主导航系统(复用室内 navigation.launch.py: 底盘/雷达/局部EKF/nav2)
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fastlio_dir, 'launch', 'navigation.launch.py')),
        launch_arguments=nav_args.items()))

    # 2. RTK 驱动: gps 模式必须有; amcl 模式下跟着守卫开关走(与原行为一致)
    if mode == 'gps' or guard_on:
        actions.append(Node(
            package='wheeltec_dual_rtk_driver',
            executable='dual_rtk_driver_node',
            name='rtk_driver',
            parameters=[{
                'port': LaunchConfiguration('gps_port'),
                'baud': LaunchConfiguration('gps_baud'),
                'gps_frame_id': 'gps_link',
            }],
            output='screen',
            respawn=False))

    if mode == 'gps':
        # 3a. GPS -> 地图系位姿(位置 + 双天线航向, 已补天线杆臂)
        actions.append(Node(
            package='wheeltec_outdoor_nav',
            executable='gps_map_odom.py',
            name='gps_map_odom',
            parameters=[{'map_file': map_path}],
            output='screen'))
        # 3b. 全局 EKF, 发布 map -> odom_combined
        # 输出话题必须重映射: 局部 EKF 已经占用了 /odometry/filtered, 撞名的话两路
        # 估计会混在同一个话题上。set_pose 同理, 否则一次 set_pose 会同时重置两个 EKF。
        actions.append(Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global_node',
            parameters=[os.path.join(outdoor_pkg, 'config', 'ekf_global.yaml')],
            remappings=[('odometry/filtered', 'odometry/global'),
                        ('set_pose', 'ekf_global/set_pose')],
            output='screen'))
    elif guard_on:
        # 3. GPS 定位守卫(仅 amcl 模式; gps 模式下 AMCL 不发 TF, 救它没有意义)
        actions.append(Node(
            package='wheeltec_outdoor_nav',
            executable='gps_localization_guard.py',
            name='gps_localization_guard',
            parameters=[{
                'enable_guard': True,
                # 守卫据此找同名的 <地图名>.gps.yaml 并校验 map_name
                'map_file': map_path,
                'calibration_file': LaunchConfiguration('calibration_file'),
                'amcl_topic': '/amcl_pose',
                'gps_fix_topic': '/gps/fix',
                'gps_heading_topic': '/gps/euler',
                'initialpose_topic': '/initialpose',
                'max_covariance': LaunchConfiguration('max_covariance'),
                'max_gps_amcl_distance': LaunchConfiguration('max_gps_amcl_distance'),
                'bad_count_threshold': 3,
                # 1=差分/浮点及以上。0 会拿米级的单点解去救援(2026-09-16 改)
                'min_gps_quality': 1,
                'check_rate': 1.0,
                # 平方后即下发给 AMCL 的 initialpose 协方差; 标定 RMS 0.38m,
                # 给 0.05 等于声称准到 5cm, 过度自信反而难恢复(2026-09-16 改)
                'gps_xy_std': 0.5,
                'gps_heading_std': 0.5,
            }],
            output='screen',
            respawn=False))

    return actions


def generate_launch_description():
    nav_dir = get_package_share_directory('wheeltec_nav2')
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(nav_dir, 'map', 'OUTDOOR_MAP.yaml'),
            description='户外地图文件'),
        DeclareLaunchArgument(
            'localization', default_value='gps',
            description='定位模式: gps=RTK+全局EKF(默认, 地图无标定时自动退回); '
                        'amcl=点云配准+GPS守卫'),
        DeclareLaunchArgument(
            'enable_gps_guard', default_value='true',
            description='amcl 模式下是否启用 GPS 守卫(gps 模式不用它)'),
        DeclareLaunchArgument('gps_port', default_value='/dev/wheeltec_gnss',
                              description='GPS串口设备'),
        DeclareLaunchArgument('gps_baud', default_value='115200',
                              description='GPS波特率'),
        DeclareLaunchArgument(
            'calibration_file',
            default_value='~/wheeltec_ros2/outdoor_maps/gps_map_calibration.yaml',
            description='守卫的兼容回退路径(正常应使用与地图同名的 .gps.yaml)'),
        DeclareLaunchArgument('use_camera', default_value='false',
                              description='是否启用相机(户外通常不需要)'),
        DeclareLaunchArgument(
            'scan_max_z', default_value='2.0',
            description='户外 /scan 障碍带上限(米), 须与存图时的 max_z 一致'),
        DeclareLaunchArgument('max_covariance', default_value='0.5',
                              description='守卫: AMCL 协方差阈值 (m^2)'),
        DeclareLaunchArgument('max_gps_amcl_distance', default_value='5.0',
                              description='守卫: GPS-AMCL 最大偏差 (m)'),
        OpaqueFunction(function=_setup),
    ])
