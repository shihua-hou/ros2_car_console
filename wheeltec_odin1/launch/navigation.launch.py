"""
Odin1(留形科技 空间记忆模组) 重定位导航模式 —— 独立于 MID360s/N10Plus 的第三种
可选模式。Odin1 自带完整定位(map->odom->odin1_base_link, 见官方SLAM/重定位API),
因此本模式**不用** AMCL、不用 nav2_map_server、不用 wheeltec_fastlio 的
auto_relocalize——那些都是给"外部提供2D地图+激光扫描匹配"的定位方式准备的,
Odin1 自己就是定位来源, 同时跑两套会在 map/odom 帧上打架(TF 单亲结构冲突)。

只保留 Nav2 的 controller/planner/behavior/bt_navigator/waypoint_follower
(即 nav2_bringup 的 navigation_launch.py, 不是完整的 bringup_launch.py)。

【硬件到货前状态说明 —— 请通读, 这一段比 mapping.launch.py 更不确定】
1. Odin1 相对底盘 base_footprint 的安装偏移(下面 x/y/z/yaw 参数)全部是占位 0,
   必须等硬件实装后卷尺实测回填, 且必须与 mapping.launch.py 用的值一致。
2. 全局代价地图(global_costmap)**没有用静态栅格地图**——Odin1 不像
   FAST-LIO/N10Plus 那样能直接产出 2D occupancy grid, 目前用 rolling_window
   (20x20m, 随机器人滚动)的方式做"伪全局"代价地图, 全局路径规划范围有限,
   不是真正基于预建地图的全局规划。真要做静态地图, 思路是: 建图时把
   /odin1/cloud_slam 落盘成 pcd, 再复用 wheeltec_fastlio 的 pcd2pgm 工具转 2D
   地图——这部分还没做, 是后续工作。
3. 已读 host_sdk_sample 源码确认, 其内部 TF 广播: odom为 odin1_base_link 和
   map 两者的公共"父帧"(不是标准 map->odom->base_link 顺序, 而是 odom 同时是
   两者的父节点)。tf2 查询任意两帧间变换本身不要求 map 一定是根节点, 理论上
   costmap/controller一样能正常算出 map->base_footprint, 但如果重定位/闭环
   修正体现在 odom->map 这条支路上, 可能表现为"map这个全局参考系本身发生
   跳变"(而不是通常预期的"机器人在map里的位置被修正"), 具体表现如何、要不要
   额外写桥接节点纠正, 必须等硬件到手后用
   `ros2 run tf2_tools view_frames` + `ros2 topic echo /tf` 实测确认。

用法:
  ros2 launch wheeltec_odin1 navigation.launch.py
  ros2 launch wheeltec_odin1 navigation.launch.py map_name:=bedroom
前提: 已用 mapping.launch.py 建过图并存图, 得到 wheeltec_odin1/map/<map_name>.bin
"""
import glob
import os
import subprocess
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

EXT_PATTERN = ('component_container_isolated|livox_ros_driver2_node|'
               'wheeltec_robot_node|fastlio_mapping|lslidar_driver_node|'
               'async_slam_toolbox_node|host_sdk_sample')


def _abort_if_already_running():
    probe = subprocess.run(['pgrep', '-af', EXT_PATTERN],
                           capture_output=True, text=True)
    lines = [l for l in probe.stdout.strip().splitlines() if l]
    if lines:
        raise RuntimeError(
            '\n检测到已有建图/导航实例正在运行(MID360/N10Plus/Odin1), 拒绝重复启动!\n'
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


def _launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('wheeltec_odin1')
    wheeltec_launch_dir = os.path.join(
        get_package_share_directory('turn_on_wheeltec_robot'), 'launch')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    param_file = LaunchConfiguration('params')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_name = LaunchConfiguration('map_name').perform(context) or 'odin1_map'

    # 把选中的地图名写进一份临时配置副本(relocalization_map_abs_path)
    map_bin_path = os.path.join(pkg_dir, 'map', f'{map_name}.bin')
    base_yaml_path = os.path.join(pkg_dir, 'config', 'control_command_navigation.yaml')
    with open(base_yaml_path) as f:
        cfg = yaml.safe_load(f)
    cfg['register_keys']['relocalization_map_abs_path'] = map_bin_path
    tmp_dir = '/tmp/wheeltec_odin1_launch'
    os.makedirs(tmp_dir, exist_ok=True)
    tmp_yaml = os.path.join(tmp_dir, 'control_command_navigation_runtime.yaml')
    with open(tmp_yaml, 'w') as f:
        yaml.safe_dump(cfg, f)

    # Odin1 相对 base_footprint 的安装位置, 必须与 mapping.launch.py 一致
    lidar_x = LaunchConfiguration('lidar_x', default='0.0')
    lidar_y = LaunchConfiguration('lidar_y', default='0.0')
    lidar_z = LaunchConfiguration('lidar_z', default='0.2')
    lidar_yaw = LaunchConfiguration('lidar_yaw', default='0.0')

    # 底盘串口(电机驱动), 不启动EKF——Odin1自己出全局定位,
    # 不需要也不能同时跑轮式EKF的 TF(会跟Odin1的map/odom树打架)
    base_serial = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wheeltec_launch_dir, 'base_serial.launch.py')),
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )
    robot_mode_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wheeltec_launch_dir, 'robot_mode_description.launch.py')),
    )

    odin1_driver = Node(
        package='odin_ros_driver',
        executable='host_sdk_sample',
        name='host_sdk_sample',
        output='screen',
        parameters=[{'config_file': tmp_yaml}],
    )

    # 把 Odin1 固定在底盘上产生的偏移, 接到它自己的TF树末端(odin1_base_link),
    # 让 base_footprint 也纳入这棵树, costmap/控制器才能算出到它的变换
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odin1_to_base_tf',
        arguments=['--x', lidar_x, '--y', lidar_y, '--z', lidar_z,
                   '--yaw', lidar_yaw, '--pitch', '0', '--roll', '0',
                   '--frame-id', 'odin1_base_link', '--child-frame-id', 'base_footprint'],
    )

    waypoint_cycle = Node(
        name='waypoint_cycle',
        package='nav2_waypoint_cycle',
        executable='nav2_waypoint_cycle',
    )

    # 只要 controller/planner/behavior/bt_navigator/waypoint_follower,
    # 不要 map_server/amcl(nav2_bringup 完整版 bringup_launch.py 会带上这两个,
    # Odin1模式故意不用完整版, 避免和它自己的定位打架)
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': param_file,
            'autostart': 'true',
        }.items(),
    )

    return [base_serial, joint_state_publisher_node, robot_mode_description,
            odin1_driver, lidar_tf, waypoint_cycle, nav2_navigation]


def generate_launch_description():
    _abort_if_already_running()
    _clean_stale_dds_shm()
    pkg_dir = get_package_share_directory('wheeltec_odin1')
    return LaunchDescription([
        DeclareLaunchArgument('params',
            default_value=os.path.join(pkg_dir, 'config', 'param_odin1.yaml'),
            description='nav2参数文件'),
        DeclareLaunchArgument('map_name', default_value='odin1_map',
            description='要加载重定位的地图名(不含.bin后缀, 对应mapping时的map_name)'),
        DeclareLaunchArgument('lidar_x', default_value='0.0'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.2'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),
        OpaqueFunction(function=_launch_setup),
    ])
