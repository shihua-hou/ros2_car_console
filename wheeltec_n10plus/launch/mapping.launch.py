"""
N10Plus 2D激光雷达 + slam_toolbox 建图 (与 MID360s+FAST-LIO 是完全独立的模式,
互不影响, 但同样占用底盘串口/cmd_vel, 不能与 MID360 建图/导航同时运行)

【硬件到货前状态说明】
- lslidar_driver 包已编译验证通过(不依赖实际硬件即可编译)
- 本 launch 未接实际 N10Plus 硬件测试过, serial_port/udev/量程等需按
  wheeltec_n10plus/README.md 的清单逐项核对

用法:
  ros2 launch wheeltec_n10plus mapping.launch.py
建图完成后 Ctrl+C 退出, 再调用 slam_toolbox 的保存地图服务:
  ros2 run nav2_map_server map_saver_cli -f <保存路径>/N10PLUS_MAP
  (或 ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'N10PLUS_MAP'}}")
"""
import glob
import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 与 MID360 (wheeltec_fastlio) 共用同一套"防重复启动/清SHM"约定, 探测特征
# 里加入本模式自己的关键进程名, 双向都能防止互相打架
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
    """清理被强杀进程残留的 FastDDS 共享内存/信号量文件。

    进程被 kill -9 或段错误退出后, /dev/shm 会残留已锁定的
    sem.fastrtps_portXXXX_mutex, 新启动的节点(尤其nav2容器)尝试加锁时
    会在 futex 上永久死锁(表现为100%CPU且无日志)。

    只删没有任何活进程在用的文件(2026-09-18 改)。原来按进程名猜"还有没有别的
    ROS 进程在跑", 名单总有漏的(RTK驱动/gps_map_odom/ros2 daemon/手敲的命令),
    会删掉活进程的文件使其通信断开; FastDDS 还靠 *_el 锁文件判断端口/段的主人
    是否活着, 锁文件被删, 后来的进程会把活端口当空闲占用, 两个进程共用一个端口。
    现在直接问内核: 被某个进程映射(/proc/*/maps)或打开(/proc/*/fd)就是在用。
    同一段/端口的几个文件(本体、_el 锁、sem.*_mutex)算一组, 组内有一个在用或
    10 秒内新建(防止和正在启动的进程赛跑), 整组保留。
    """
    import time
    groups = {}
    for f in (glob.glob('/dev/shm/fastrtps_*') +
              glob.glob('/dev/shm/sem.fastrtps_*') +
              glob.glob('/dev/shm/fast_datasharing*')):
        key = os.path.basename(f)
        if key.startswith('sem.'):
            key = key[4:]
        for suffix in ('_mutex', '_el', '_sl'):
            if key.endswith(suffix):
                key = key[:-len(suffix)]
                break
        groups.setdefault(key, []).append(f)
    if not groups:
        return
    used = set()  # 在用文件的 inode
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open('/proc/%s/maps' % pid) as fh:
                for line in fh:
                    if ' /dev/shm/' in line:
                        used.add(line.split()[4])
            fds = os.listdir('/proc/%s/fd' % pid)
        except OSError:
            continue
        for fd in fds:
            link = '/proc/%s/fd/%s' % (pid, fd)
            try:
                if os.readlink(link).startswith('/dev/shm/'):
                    used.add(str(os.stat(link).st_ino))
            except OSError:
                pass
    now = time.time()
    for files in groups.values():
        try:
            stats = [os.stat(f) for f in files]
        except OSError:
            continue
        if any(str(s.st_ino) in used or now - max(s.st_mtime, s.st_ctime) < 10.0
               for s in stats):
            continue
        for f in files:
            try:
                os.remove(f)
            except OSError:
                pass


def generate_launch_description():
    _abort_if_already_running()
    _clean_stale_dds_shm()
    pkg_dir = get_package_share_directory('wheeltec_n10plus')
    wheeltec_launch_dir = os.path.join(
        get_package_share_directory('turn_on_wheeltec_robot'), 'launch')

    # N10Plus 在小车上的安装位置(相对base_footprint), 2026-07-22 实测回填:
    # 轴心距车头10cm, 雷达在车头内侧缩进6cm -> 轴心前方4cm; 离地高度20cm
    lidar_x = LaunchConfiguration('lidar_x', default='0.04')
    lidar_y = LaunchConfiguration('lidar_y', default='0.0')
    lidar_z = LaunchConfiguration('lidar_z', default='0.2')
    lidar_yaw = LaunchConfiguration('lidar_yaw', default='0.0')

    # 底盘(串口+EKF: odom_combined->base_footprint), 与MID360建图用法一致
    wheeltec_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wheeltec_launch_dir, 'turn_on_wheeltec_robot.launch.py')),
        launch_arguments={'carto_slam': 'false', 'robot_nav': 'false'}.items(),
    )

    n10plus_driver = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        namespace='x10',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'n10plus.yaml')],
    )

    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_n10plus_tf',
        arguments=['--x', lidar_x, '--y', lidar_y, '--z', lidar_z,
                   '--yaw', lidar_yaw, '--pitch', '0', '--roll', '0',
                   '--frame-id', 'base_footprint', '--child-frame-id', 'n10plus_laser'],
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config',
                                 'mapper_params_online_async.yaml'),
                    {'use_sim_time': False}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('lidar_x', default_value='0.04'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.2'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),
        wheeltec_robot,
        n10plus_driver,
        lidar_tf,
        slam_toolbox_node,
    ])
