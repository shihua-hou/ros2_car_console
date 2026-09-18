"""
Odin1(留形科技 空间记忆模组) SLAM 建图模式 —— 与 MID360s/N10Plus 完全独立的
第三种可选模式, 不启动底盘EKF(Odin1自带完整的map->odom->odin1_base_link定位,
不需要轮式里程计融合), 只启动底盘串口(实际驱动电机用)。

【硬件到货前状态说明 —— 请通读】
- odin_ros_driver 已从官方仓库克隆并编译成功(host_sdk_sample 等4个可执行文件),
  但从未接过实际 Odin1 硬件跑过, 以下均为按官方文档/源码推断的最佳配置:
  1. USB设备号 2207:0019 来自官方文档, 未实机验证
  2. custom_map_mode/relocalization_map_abs_path 等参数语义来自官方 wiki,
     "./set_param.sh save_map 1" 触发存图的具体交互方式建议到货后先看
     odin_ros_driver/RELOCALIZATION_GUIDE.md 原文确认一遍
  3. 读过 host_sdk_sample 源码(include/host_sdk_sample.h), 确认其 TF 广播用的
     固定坐标系名: imu_link / odin1_base_link / odom / map (均硬编码, 配置文件
     改不了名字)。但源码里 "odom->map" 的父子关系跟标准 REP105 (map为根,
     odom为其子节点) 顺序相反——map 实际上是 odom 的子节点而非父节点。
     tf2 查询任意两帧的相对变换不要求 map 一定是树根, 所以功能上 Nav2 大概率
     仍能正常算出 map->base_footprint, 但如果它们的重定位/闭环校正是打在这条
     odom->map 分支上, 理论上会体现为"map这个全局参考系本身发生跳变", 而不是
     通常预期的"机器人在map里的位置修正"——实际效果如何、要不要额外写一个
     反转桥接节点, 必须等硬件到手后用 ros2 run tf2_tools view_frames 和
     ros2 topic echo /tf 实测确认, 这里先按"能用"的最简方案接, 有问题再改。

用法:
  ros2 launch wheeltec_odin1 mapping.launch.py
  ros2 launch wheeltec_odin1 mapping.launch.py map_name:=bedroom
建图完成(在小车周围走一圈, 尽量形成闭环)后, 保存地图:
  cd ~/wheeltec_ros2/src/odin_ros_driver && ./set_param.sh save_map 1
  (地图存到 wheeltec_odin1/map/<map_name>.bin; 由于该SDK的存图目标文件名只能
  在节点启动时确定, 不能像MID360/N10Plus那样在建图过程中临时改名, 所以
  map_name 必须作为启动参数传入, 不是存图时才选——这一段未接硬件测试过)
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


def _launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('wheeltec_odin1')
    wheeltec_launch_dir = os.path.join(
        get_package_share_directory('turn_on_wheeltec_robot'), 'launch')
    map_name = LaunchConfiguration('map_name').perform(context) or 'odin1_map'
    map_dir = os.path.join(pkg_dir, 'map')
    os.makedirs(map_dir, exist_ok=True)

    # SDK 的存图目标文件名只在节点启动时读取一次, 存图触发(set_param.sh)时
    # 改不了, 所以要在这里(启动时)把 map_name 写进一份临时配置副本
    base_yaml_path = os.path.join(pkg_dir, 'config', 'control_command_mapping.yaml')
    with open(base_yaml_path) as f:
        cfg = yaml.safe_load(f)
    cfg['register_keys']['mapping_result_dest_dir'] = map_dir
    cfg['register_keys']['mapping_result_file_name'] = f'{map_name}.bin'
    tmp_dir = '/tmp/wheeltec_odin1_launch'
    os.makedirs(tmp_dir, exist_ok=True)
    tmp_yaml = os.path.join(tmp_dir, 'control_command_mapping_runtime.yaml')
    with open(tmp_yaml, 'w') as f:
        yaml.safe_dump(cfg, f)

    # 只要底盘串口(驱动电机), 不要EKF: Odin1自己出定位, 不需要轮式里程计融合,
    # 且EKF默认会广播 odom_combined->base_footprint TF, 会跟下面Odin1自己的
    # TF树产生冲突(同一帧不能有两个父节点)
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

    return [base_serial, joint_state_publisher_node, robot_mode_description, odin1_driver]


def generate_launch_description():
    _abort_if_already_running()
    _clean_stale_dds_shm()
    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='odin1_map',
            description='本次建图保存的地图文件名(不含.bin后缀)'),
        OpaqueFunction(function=_launch_setup),
    ])
