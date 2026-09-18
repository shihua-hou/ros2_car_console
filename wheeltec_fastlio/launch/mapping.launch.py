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
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PCD_DIR = '/home/cat/wheeltec_ros2/src/FAST_LIO/PCD'


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


def _abort_if_already_running():
    """检测到已有建图/导航实例时拒绝启动。
    双实例会把组件加载进同名 nav2 容器、串口/雷达冲突、CPU 过载, 系统必乱。"""
    probe = subprocess.run(
        ['pgrep', '-af',
         'component_container_isolated|livox_ros_driver2_node|'
         'wheeltec_robot_node|fastlio_mapping|astra_camera_node|nav2_waypoint_cycle'],
        capture_output=True, text=True)
    lines = [l for l in probe.stdout.strip().splitlines() if l]
    if lines:
        raise RuntimeError(
            '\n检测到已有建图/导航实例正在运行, 拒绝重复启动!\n'
            '冲突进程:\n  ' + '\n  '.join(lines[:6]) +
            '\n请先在原终端 Ctrl+C 关闭它; 如果关不掉, 执行:\n'
            '  pkill -9 -f "component_container_isolated|livox_ros_driver2_node"\n'
            '然后重新启动本 launch。')


def _check_disk():
    """建图前检查磁盘余量。

    FAST-LIO 每 100 帧(约10秒)落一段 PCD, 每段约 16MB —— 也就是 **每分钟约 100MB**。
    2026-08-06 实际发生过: 建图忘了关, 跑了 47 分钟, 攒下 286 段 4.6GB 把 29G 的盘
    写满; 磁盘满之后连 colcon 的编译产物都会被写坏(install 里出现 0 字节文件)。
    所以这里按余量分三档: <2G 直接拒绝启动, <5G 警告并给出可建图时长。
    """
    try:
        st = os.statvfs(PCD_DIR)
    except OSError:
        return
    free_gb = st.f_bavail * st.f_frsize / (1 << 30)
    minutes = int(free_gb * 1024 / 100)      # 约 100MB/分钟
    if free_gb < 2.0:
        raise RuntimeError(
            f'\n磁盘只剩 {free_gb:.1f}GB, 拒绝启动建图!\n'
            f'建图每分钟约产生 100MB 的 PCD 分段, 现在最多只够建 {minutes} 分钟,\n'
            f'而磁盘写满会导致编译产物损坏(实测踩过)。请先腾空间:\n'
            f'  rm -f {PCD_DIR}/*.pcd        # 清上次建图的点云(存过图就可以删)\n'
            f'  sudo journalctl --vacuum-size=50M\n'
            f'  df -h /')
    if free_gb < 5.0:
        print(f'\n\033[33m[警告] 磁盘剩余 {free_gb:.1f}GB, 约够建图 {minutes} 分钟'
              f'(每分钟约100MB)。建图结束记得 save_map 后清理 PCD。\033[0m\n')
    else:
        print(f'磁盘剩余 {free_gb:.1f}GB, 约够建图 {minutes} 分钟(每分钟约100MB)')


def _handle_stale_pcd():
    """处理上次建图遗留的分段点云。

    这些分段必须先清掉, 否则 save_map 会把两次建图的点云混在一起 —— FAST-LIO
    每次都重新初始化世界系, 混出来的是废图。
    但**不再静默删除**(2026-09-14 改): 删掉的可能是用户还没来得及存图的数据。
    要清理就显式传 clear_pcd:=true, 否则直接拒绝启动并说明怎么处理。
    """
    stale = sorted(glob.glob(os.path.join(PCD_DIR, 'scans*.pcd')))
    if not stale:
        return
    if any(a.strip() == 'clear_pcd:=true' for a in sys.argv):
        for f in stale:
            os.remove(f)
        print(f'\033[33m[PCD] 已清理上次建图遗留的 {len(stale)} 个分段\033[0m')
        return
    raise RuntimeError(
        f'\n检测到上次建图遗留的 {len(stale)} 个 PCD 分段, 拒绝启动!\n'
        '直接开始新建图, save_map 会把新旧两次的点云混在一起(生成废图)。\n'
        '请二选一:\n'
        '  1) 旧数据不要了(已经存过图) —— 清理后启动:\n'
        '     ros2 launch wheeltec_fastlio mapping.launch.py clear_pcd:=true\n'
        '  2) 旧数据还要 —— 先备份再启动:\n'
        f'     mv {PCD_DIR}/scans*.pcd <备份目录>/\n'
        '(网页控制台点"建图"会弹窗确认, 不用手敲命令)\n')


def generate_launch_description():
    _abort_if_already_running()
    _clean_stale_dds_shm()
    _handle_stale_pcd()
    _check_disk()
    pkg_dir = get_package_share_directory('wheeltec_fastlio')
    wheeltec_launch_dir = os.path.join(
        get_package_share_directory('turn_on_wheeltec_robot'), 'launch')

    rviz_use = LaunchConfiguration('rviz', default='false')
    # 雷达安装外参(相对base_footprint) —— 2026-08-06 雷达移到相机上方下倾约45°、
    # 同日又前移了一次, 以下为前移后的终值。完整标定记录见 navigation.launch.py 的同名段落。
    #   pitch/roll: IMU重力法 vs 地面拟合两法差 0.71°, 取地面拟合
    #   lidar_z:    地面拟合
    #   lidar_x:    车前0.5m放纸箱, 雷达和相机比较各自到该立面的垂距, 向量投影解出
    # **建图和导航两处必须完全一致**, 改一处就要改另一处。
    lidar_x = LaunchConfiguration('lidar_x', default='0.08')
    lidar_y = LaunchConfiguration('lidar_y', default='0.0')
    lidar_z = LaunchConfiguration('lidar_z', default='0.28')
    lidar_yaw = LaunchConfiguration('lidar_yaw', default='0.0')
    lidar_pitch = LaunchConfiguration('lidar_pitch', default='0.292')  # +16.7° 下倾(2026-08-07 傍晚重新固定雷达后实测)
    lidar_roll = LaunchConfiguration('lidar_roll', default='0.006')   # +0.32°

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

    # 【正后方扇区屏蔽】建图时人跟在车后遥控, 不屏蔽的话人会被当成静态结构建进
    # 地图(而且他一直在动, 喂给里程计也是负担)。默认 90 = 屏蔽正后方 ±45°。
    # 实测该扇区占全部点的 26%, 屏蔽后仍有 270° 视野, 对 FAST-LIO 的约束绰绰有余。
    # 不想屏蔽就 blind_back_deg:=0。屏蔽只作用于建图链路(FAST-LIO 的 preprocess),
    # 导航的 /scan 走 pointcloud_to_laserscan, 不受影响 —— 导航时后方仍然能看见。
    blind_back_deg = LaunchConfiguration('blind_back_deg', default='90.0')

    # FAST-LIO2 建图, TF: camera_init -> body
    fast_lio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'wheeltec_mid360.yaml'),
            {'use_sim_time': False},
            # 必须显式声明 value_type: 不写的话 launch 把 '90.0' 当字符串传下去,
            # 而节点声明的是 double, 起不来会直接报 InvalidParameterTypeException
            {'preprocess.blind_back_deg':
                ParameterValue(blind_back_deg, value_type=float)},
        ],
    )

    # 雷达安装位置静态TF
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_livox_tf',
        arguments=['--x', lidar_x, '--y', lidar_y, '--z', lidar_z,
                   '--yaw', lidar_yaw, '--pitch', lidar_pitch, '--roll', lidar_roll,
                   '--frame-id', 'base_footprint', '--child-frame-id', 'livox_frame'],
    )

    # 建图时也起相机 —— **只起彩色流, 不起深度、不起 depth_obstacle_filter**。
    # 建图本身完全不用相机(定位靠雷达), 这一路纯粹是给网页控制台看实时画面用的:
    # 遥控建图时能看见车头前方, 比盯着点云好判断该往哪走。
    # 320x240@15 的彩色流开销很小(建图阶段 FAST-LIO 才是 CPU 大头), 不影响建图。
    # 不需要就 use_camera:=false。
    use_camera = LaunchConfiguration('use_camera', default='true')
    astra_camera = Node(
        condition=IfCondition(use_camera),
        package='astra_camera',
        executable='astra_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=[{
            'camera_name': 'camera',
            'camera_link_frame_id': 'camera_mount_link',
            'vendor_id': '0x2bc5',
            'device_num': 1,
            'connection_delay': 100,
            'enable_point_cloud': False,
            'enable_colored_point_cloud': False,
            'enable_depth': False,      # 建图不需要深度, 省 CPU 和带宽
            'enable_color': True,
            'color_width': 640, 'color_height': 480, 'color_fps': 30,
            'enable_ir': False,
            'publish_tf': False,        # 建图期间不挂相机TF, 免得和 camera_init 树混淆
            'depth_registration': False,
            'oni_log_level': 'none',
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'fastlio.rviz')],
        condition=IfCondition(rviz_use),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('lidar_x', default_value='0.08'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.28'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.292',
            description='雷达俯仰角(弧度); 已改回水平安装'),
        DeclareLaunchArgument('lidar_roll', default_value='0.006',
            description='雷达横滚角(弧度); 已改回水平安装'),
        DeclareLaunchArgument('use_camera', default_value='true',
            description='建图时是否起相机(仅彩色流, 给网页看实时画面; 不参与建图)'),
        DeclareLaunchArgument('blind_back_deg', default_value='90.0',
            description='屏蔽雷达正后方多少度的扇区(全角), 防止跟车的人被建进地图; 0=不屏蔽'),
        DeclareLaunchArgument('clear_pcd', default_value='false',
            description='true=启动前清掉上次建图遗留的PCD分段; false=有遗留就拒绝启动'),
        wheeltec_robot,
        livox_driver,
        fast_lio,
        lidar_tf,
        astra_camera,
        rviz_node,
    ])
