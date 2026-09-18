"""
MID360 3D雷达导航 (基于 wheeltec_nav2 框架)
架构:
  MID360点云 -> pointcloud_to_laserscan -> /scan ---------\
  Astra深度图 -> depth_obstacle_filter -> /camera/obstacle_points -> nav2 costmap
  AMCL: map -> odom_combined | EKF(轮式+IMU): odom_combined -> base_footprint
  地图: 由 FAST-LIO2 建图 + save_map.launch.py 生成的 WHEELTEC3D
  (定位只用雷达; 相机只喂 costmap 做避障, 不参与定位, 相机坏了不影响定位)
用法:
  ros2 launch wheeltec_fastlio navigation.launch.py
  ros2 launch wheeltec_fastlio navigation.launch.py map:=/path/to/other.yaml
  ros2 launch wheeltec_fastlio navigation.launch.py use_camera:=false   # 纯雷达
"""
import glob
import os
import subprocess
import sys
import yaml
from ament_index_python.packages import get_package_share_directory


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


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


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

    # ---------------- 雷达安装外参(相对base_footprint), 必须与建图时一致 ----------------
    # 2026-08-06 最终状态: 雷达**改回水平安装(360°)**, 装在相机正上方。
    # 当天中途曾下倾约42°并为此改过一整套东西(/scan拆两路、地图高带2.0/1.2、
    # pcd2pgm按安装角算地面搜索轴), 现已全部回退, 见本文件各处注释。
    #
    # 【俯仰和横滚】2026-08-06 最后一次改装: 由水平改为**下倾约 11°**(为了让地面
    #   有回波 —— 水平时镜面地面几乎收不到地面点, FAST-LIO 的 z 缺约束会漂)。
    #   ① IMU重力法: 雷达系"上"方向 (-0.1898, 0.0055, 0.9818)
    #      -> roll +0.32°, pitch +10.94°   (静止判据: 加速度 std 实测 0.004~0.006)
    #   ② 地面拟合法: -> roll +0.25°, pitch +10.32°
    #   两法互差 0.62°。**取①(IMU)**: 因为②给出的高度(0.446m)已被证明是错的
    #   (卷尺 0.25, 见下), 说明它锁在了镜面鬼点平面上, 它的法向同样不可信。
    #   => pitch = 0.191 rad, roll = 0.006 rad
    #
    # 【高度】lidar_z = **0.28m**。
    #   ⚠️ 这个值的来源在当天变过两次, 结论是"**能不能信点云, 取决于雷达倾没倾**":
    #   · 水平安装时点云拟合出 0.443/0.446, 卷尺是 0.25 —— 差 19cm, **点云完全不可信**。
    #     镜面地面在掠射角下几乎无回波, 镜面产生的"地下鬼点"把低处点RANSAC整个带偏,
    #     而且残差只有13mm、内点数千, **残差小内点多完全不说明拟合对**。
    #   · 下倾 11° 之后地面回波够了, 点云反而比卷尺准。三条独立线索一致偏向 0.28:
    #       点云 z 直方图出现地面尖峰(水平时没有峰) -> 0.2875
    #       pcd2pgm 的 RANSAC 拟合                  -> 0.27
    #       后向视场几何(见下)                       -> 0.28 比 0.25 更贴合实测
    #     卷尺那 3cm 的差多半来自"光心比底座高 8cm"这个经验值(实际可能是 11cm)。
    #   结论: **倾装后可以信点云; 水平安装时只能信卷尺。**
    #
    # 【前后位置】lidar_x = **0.08m** —— 用户卷尺实测。
    #   相机镜头在 0.09, 即雷达光心比镜头靠后 1cm, 与"装在相机正上方"吻合,
    #   也落在历史实测的光心-镜头前后差区间(-1.2cm ~ +2.1cm)内。
    #   若要用点云复核: 车前 0.5m 放个纸箱, 雷达和相机各自拟合它的立面, 解
    #   d_L - d_C = n·(O_L - O_C) 得 dx 再加 camera_x。参照物不需要垂直也不需要
    #   正对车头(所以要用完整向量投影而不是简单相减), 但**不能拿纸箱标 camera_pitch**
    #   —— 箱子后仰几度, "法向竖直"判据就偏几度。
    # ⚠️ Livox 对无回波点填 (0,0,0), 拟合前必须先滤掉(否则原点处几十万假点
    #    会把平面拟合整个带偏 —— 曾算出 lidar_x=+0.546 的鬼值)。
    lidar_x = LaunchConfiguration('lidar_x', default='0.08')
    lidar_y = LaunchConfiguration('lidar_y', default='0.0')
    lidar_z = LaunchConfiguration('lidar_z', default='0.28')
    lidar_yaw = LaunchConfiguration('lidar_yaw', default='0.0')
    lidar_pitch = LaunchConfiguration('lidar_pitch', default='0.292')  # +16.7°(地面拟合), 正=下倾
    lidar_roll = LaunchConfiguration('lidar_roll', default='0.006')   # +0.32°

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

    # 3D点云 转 2D激光 /scan (取 base_footprint 坐标系下 0.15~0.45m 高度带)
    #
    # 2026-08-06: 雷达一度下倾42°, 那时后方视野整片翻到天花板上, 0.15~0.45m 这条
    # 碰撞带在正后方一个点都收不到(方位覆盖只剩57%), 被迫拆成两路——/scan 用
    # 0.15~2.0m 高带保定位、/scan_obstacle 用 0.05~0.45m 保避障。**雷达改回水平后
    # 这个问题不存在了**: 实测 0.15~0.45m 带方位覆盖 100%(72/72个扇区)、正后方
    # 1628 点, 所以已撤回单路, costmap 和 AMCL 共用这一路。
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
            'min_height': ParameterValue(LaunchConfiguration('scan_min_z'),
                                         value_type=float),
            # 与 save_map 的 max_z 必须一致 —— 地图和 /scan 不同带, 似然场会把
            # 差集当失配。户外模式两边一起提到 2.0(见 outdoor_navigation.launch.py)
            'max_height': ParameterValue(LaunchConfiguration('scan_max_z'),
                                         value_type=float),
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
                   '--yaw', lidar_yaw, '--pitch', lidar_pitch, '--roll', lidar_roll,
                   '--frame-id', 'base_footprint', '--child-frame-id', 'livox_frame'],
    )

    # ---------------- Astra 深度相机 (补 MID360 的近距离/低矮障碍盲区) ----------------
    # MID360 装在车上0.30m高, 扫出的 /scan 只取 0.15~0.45m 高度带, 车前
    # 0.5m 以内和地面上的矮物(拖鞋、凳子横档、门槛)基本看不见。相机朝前下方看,
    # 把这些补进 costmap。use_camera:=false 可一键退回纯雷达导航。
    use_camera = LaunchConfiguration('use_camera', default='true')

    # 找不到 astra_camera 包时给一句人话。launch_ros 自己报的是
    # "package 'astra_camera' not found, searching: [...]" 后面跟一屏路径,
    # 真正的原因(终端环境是编译相机之前 source 的)全被淹没了。
    if 'use_camera:=false' not in ' '.join(sys.argv):
        try:
            get_package_share_directory('astra_camera')
        except Exception:
            raise RuntimeError(
                '\n找不到 astra_camera 包。99%是当前终端的环境太旧'
                '(在编译相机包之前 source 的), 不是真的没装。\n'
                '解决: 重开一个终端, 或执行\n'
                '  source ~/wheeltec_ros2/install/setup.bash\n'
                '确认装没装: ls ~/wheeltec_ros2/install/astra_camera\n'
                '没装就编译: colcon build --packages-select astra_camera_msgs astra_camera\n'
                '不想用相机: ros2 launch wheeltec_fastlio navigation.launch.py use_camera:=false')

    # 直接起驱动节点(不 include astra.launch.xml): 那个 xml 没暴露
    # camera_link_frame_id, 而我们必须换掉这个名字 —— 见下面 camera_tf 的说明。

    # 标定内参(可选): tagdocking 包的 ost.yaml 存在时让驱动加载, 否则用驱动默认内参。
    # 标定方法: tagdocking/scripts/calibrate_camera (需 ssh -X 跑 cameracalibrator)。
    # 文件不存在时留空 —— astra 驱动 color_info_url 为空就不建 CameraInfoManager,
    # 不刷 warn; 标定后 colcon build --packages-select tagdocking 把 ost.yaml 装进 install 即生效。
    # 注意: ost 的 [name] 段必须是 "rgb_camera"(驱动 setupCameraInfoManager 硬编码的
    # CameraInfoManager 名), 否则 isCalibrated() 返 false 静默回退默认内参;
    # 标定分辨率须 == color_width/height(640x480), 否则 getColorCameraInfo() 会 warn 回退。
    calibration_ost_path = os.path.join(
        get_package_share_directory('tagdocking'),
        'config', 'calibration', 'ost.yaml')
    color_info_url = ('file://' + calibration_ost_path
                      if os.path.exists(calibration_ost_path) else '')

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
            'color_info_url': color_info_url,
            'vendor_id': '0x2bc5',
            'device_num': 1,
            'connection_delay': 100,
            # 驱动自带的稠密点云(640x480 XYZ ≈ 3.7MB/帧 @18Hz)不要, 由
            # depth_obstacle_filter 直接读深度图抽样反投影, 省一个数量级的带宽
            'enable_point_cloud': False,
            'enable_colored_point_cloud': False,
            'enable_depth': True,
            'depth_width': 640, 'depth_height': 480, 'depth_fps': 30,
            # 彩色只给网页控制台看, 320x240@15 足够, 省CPU和无线带宽
            'enable_color': True,
            'color_width': 640, 'color_height': 480, 'color_fps': 30,
            'enable_ir': False,          # IR 与彩色互斥(驱动会自己关掉IR), 用不上
            'publish_tf': True,          # camera_mount_link -> camera_depth_optical_frame
            'tf_publish_rate': 0.0,      # 相机内部外参是固定的, 发一次静态TF即可
            'depth_registration': False,
            'oni_log_level': 'none',
        }],
    )

    # 相机安装位置(相对 base_footprint), **2026-08-05 本车实测**:
    #   镜头离地 0.21m; 前后位置在车头往回 1cm —— 车头在 x=+0.10
    #   (footprint 是 [[-0.40,±0.175],[0.10,±0.175]], 轴心距车头10cm),
    #   所以 camera_x = 0.10 - 0.01 = 0.09。
    #   camera_y 未测, 按居中处理(0)。
    # camera_pitch: **2026-08-06 相机已由下倾21°改成平视, 实测 1.0°(=0.017rad)**。
    # 标定方法与上次相同(地面是镜面抛光的, 掠射角下红外全被反射走、拿不到地面点,
    # 所以拟合正前方的立面 —— 上次用纸箱, 这次直接用车前 0.5m 的墙), 两条独立判据:
    #   ① 立面是垂直的 -> 法向的竖直分量归零 => 俯角 1.0°
    #   ② 立面坐在地上 -> 面最低点高度应为0 => 1°时 +0.011m (0°:+0.002, 3°:+0.029)
    # 两条在1°附近交汇, 同时复核了 camera_z=0.21 仍然成立(偏差约1cm)。
    # 内点 245788/269600, RMS 0.7mm。
    #
    # ⚠️ 平视之后相机的职责变小了: 镜头在 0.21m 高、竖直半视场约22.5°, 视野下沿
    # 打到地面已在 0.21/tan(22.5°) ≈ 0.51m 之外, 车前 0.5m 内的**矮物已看不到**。
    # 好在雷达下倾42°之后自己补上了这块: 雷达视野下沿(体系 -49°)落地在
    # 0.307/tan(49°) ≈ 0.27m 处, 即车体系 x≈0.35m(车头在 0.10m), 车头前 25cm 就有点。
    # 所以低矮障碍现在主要靠雷达, 相机退化成中距离补充。
    # 注意这组数**故意不再读** turn_on_wheeltec_robot/config/robot_model.yaml 的
    # base_to_camera(原厂支架值 [0.166, 0.001, 0.090]) —— 本车相机不在原厂支架上,
    # 高度差了 12cm, 用它会把地面整片当成障碍。
    #   camera_x: 驱动轮轴心 -> 镜头的前后距离(镜头在轴心前为正)
    #   camera_z: 镜头离地高度
    #   camera_pitch: 镜头俯角, 向下看为正(弧度, 5°≈0.087)
    # 倾角填错1~2°就会在3m外把地面抬成障碍, 见 depth_obstacle_filter.cpp 注释。
    #
    # 子frame 用 camera_mount_link 而不是 camera_link: 底盘 launch 和 URDF 各自
    # 已经在发 camera_link 了, 再发一个就是同一个 child 多个 parent,
    # tf2 树被反复重挂, 相机位姿时对时错。
    camera_x = LaunchConfiguration('camera_x', default='0.09')
    camera_y = LaunchConfiguration('camera_y', default='0.0')
    camera_z = LaunchConfiguration('camera_z', default='0.21')
    camera_yaw = LaunchConfiguration('camera_yaw', default='0.0')
    camera_pitch = LaunchConfiguration('camera_pitch', default='0.017')

    camera_tf = Node(
        condition=IfCondition(use_camera),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=['--x', camera_x, '--y', camera_y, '--z', camera_z,
                   '--yaw', camera_yaw, '--pitch', camera_pitch, '--roll', '0',
                   '--frame-id', 'base_footprint',
                   '--child-frame-id', 'camera_mount_link'],
    )

    # 深度图 -> 地面以上的稀疏障碍点云 (发布在相机光心系, 供 costmap raytrace)
    # ---------------- 相机避障开关 (2026-08-06 默认改为**关闭**) ----------------
    # 用户实测判定: 相机喂进来的障碍点"严重干扰局部障碍的生成逻辑", 决定不再用相机
    # 做局部避障。原因链条见 CLAUDE.md 5.7"连锁改动三":
    #   相机改平视 -> 镜面地面反投影出 -0.17~-0.36m 的鬼点 -> 部分落进高度带被当障碍
    #   -> 相机 FOV 只有58°, 误标的格子背后没有射线去清 -> 持续累积把车围死。
    #   把 min_z 提到 0.15 只能缓解, 没有根治。
    # 现在相机的职责退回到"只给网页看实时画面": use_camera 仍默认 true(起驱动),
    # 但 camera_avoid 默认 false —— 不起 depth_obstacle_filter, 不产生障碍点云。
    # 同时 param_S100_diff.yaml 的 local_costmap 已把 camera 从 observation_sources
    # 里摘掉, **两边都要打开才会真的生效**(避免只改一边造成"以为开了其实没开")。
    # 低矮障碍现在全靠雷达: 下倾40°后视野下沿落地在车头前约25cm处。
    # 想重新启用: camera_avoid:=true 且把 param 里 observation_sources 加回 camera。
    camera_avoid = LaunchConfiguration('camera_avoid', default='false')

    depth_filter = Node(
        condition=IfCondition(camera_avoid),
        package='wheeltec_fastlio',
        executable='depth_obstacle_filter',
        name='depth_obstacle_filter',
        output='screen',
        remappings=[('depth/image_raw', '/camera/depth/image_raw'),
                    ('depth/camera_info', '/camera/depth/camera_info'),
                    ('camera/obstacle_points', '/camera/obstacle_points')],
        parameters=[{
            'target_frame': 'base_footprint',
            'stride': 4,          # 640x480 -> 160x120 抽样, costmap 用不到更密
            'min_range': 0.35,    # Astra S 近距离盲区约0.4m
            # max_range 4.0->2.5 (2026-08-06): 相机改平视后, 远处的角度误差被放大,
            # 且 2.5m 以外雷达覆盖得比相机好, 留着只会引入噪声。
            'max_range': 2.5,
            # min_z 0.05->0.15 (2026-08-06)。**0.05 的前提已经不成立了**:
            # 当初敢用 0.05 是因为"地面镜面抛光、掠射角下压根测不到地面点, 所以
            # 地面不可能被误判成障碍"(见 CLAUDE.md 5.6)。但相机改平视之后, 镜面
            # 地面不再是"测不到", 而是**测出一堆假点**(实测反投影出 -0.17~-0.36m
            # 的地下鬼点, 即镜像反射), 其中一部分会落在 0.05~0.15 带里被当成障碍。
            # 实测后果: 车停着不动, local costmap 的 lethal 格 7 秒内从 81 涨到 126
            # 并持续累积(相机视野只有58°, marked 的格子没有射线去清) -> 车被膨胀
            # 层围死、下了目标点走不动。停掉相机滤波节点后 lethal 稳定在 80~92,
            # 改成 min_z=0.15 + max_range=2.5 后同样稳定, 且近距离真障碍仍在。
            # 换到地毯/哑光地面可以再往下试 0.10, 但要重做这个"清空后看涨不涨"的测试。
            'min_z': 0.15,
            'max_z': 1.20,
            'voxel_size': 0.05,   # 与 costmap 分辨率同量级
            'max_rate': 10.0,
            'max_points': 4000,
        }],
    )

    waypoint_cycle = Node(
        name='waypoint_cycle',
        package='nav2_waypoint_cycle',
        executable='nav2_waypoint_cycle',
    )

    # 扫描匹配自动重定位: 启动后自动定位(无需2D Pose Estimate),
    # 定位丢失可手动触发: ros2 service call /relocalize std_srvs/srv/Trigger
    # 户外 gps 定位模式下关闭(见 outdoor_navigation.launch.py): 那时 GPS 直接给
    # 绝对位姿, 点云配准的重定位只会反复给出错误坐标。室内默认开, 行为不变。
    auto_relocalize = Node(
        condition=IfCondition(LaunchConfiguration('use_auto_relocalize',
                                                  default='true')),
        package='wheeltec_fastlio',
        executable='auto_relocalize',
        name='auto_relocalize',
        output='screen',
        parameters=[{
            'auto_on_startup': True,
            'accept_score': 0.55,
            'watchdog_en': True,     # 绑架检测: 位姿匹配分持续过低时自动全局重定位
            # 严格分(σ=0.08)阈值: 正确位姿 0.6~0.8, 被搬动 0.1~0.2。
            # 0.4 只抓得住"整个搬走"这种硬绑架, 0.4~0.6 之间的**缓慢漂移**会一直
            # 挂着不触发 —— 2026-08-06 用户反馈"一直漂"。提到 0.5 让它早一点介入;
            # 同时把连续次数从2提到3(阈值高了更容易偶发误判, 多要一次证据),
            # 净效果是"漂得多一点就纠正, 但不会因为一帧抖动就乱跳"。
            'watchdog_score': 0.5,
            'watchdog_count': 3,
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
        DeclareLaunchArgument('lidar_x', default_value='0.08'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.28'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.292',
            description='雷达下倾角(弧度), 2026-08-06 实测 10.94°(IMU重力法)'),
        DeclareLaunchArgument('lidar_roll', default_value='0.006',
            description='雷达横滚角(弧度), 实测 0.32°'),
        DeclareLaunchArgument('use_auto_relocalize', default_value='true',
            description='是否启动点云配准自动重定位(户外 RTK 定位时应关闭)'),
        DeclareLaunchArgument('scan_min_z', default_value='0.15',
            description='/scan 障碍带下限(米, 地面上方)'),
        DeclareLaunchArgument('scan_max_z', default_value='0.45',
            description='/scan 障碍带上限(米)。必须与存图时的 max_z 一致'),
        DeclareLaunchArgument('use_camera', default_value='true',
            description='是否启动 Astra 相机驱动(网页实时画面用; false=完全不起相机)'),
        DeclareLaunchArgument('camera_avoid', default_value='false',
            description='是否用相机做局部避障(默认关, 见上方说明; 开启还需同时改 param 的 observation_sources)'),
        DeclareLaunchArgument('camera_x', default_value='0.09'),
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='0.21'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0'),
        DeclareLaunchArgument('camera_pitch', default_value='0.017',
            description='相机俯角(弧度), 向下看为正; 2026-08-06 改平视后实测 1.0°'),
        imu_processor,
        wheeltec_robot,
        livox_driver,
        cloud_to_scan,
        lidar_tf,
        astra_camera,
        camera_tf,
        depth_filter,
        waypoint_cycle,
        auto_relocalize,
        nav2_bringup,
    ])
