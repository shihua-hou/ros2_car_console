"""
户外建图 Launch 文件

功能：
  1. 启动 FAST-LIO2 建图（与室内完全相同）
  2. 启动 RTK 驱动（可选，检测到设备才启动）
  3. 记录雷达和GPS双轨迹（用于后处理对齐）

使用方法：
  ros2 launch wheeltec_outdoor_nav outdoor_mapping.launch.py
  ros2 launch wheeltec_outdoor_nav outdoor_mapping.launch.py enable_gps:=false  # 纯雷达建图

特性：
  - GPS不可用时自动降级为纯雷达建图
  - 轨迹记录节点会标记GPS是否可用
  - 完全复用室内建图的所有参数
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('wheeltec_outdoor_nav')
    fastlio_dir = get_package_share_directory('wheeltec_fastlio')

    # ============ 参数声明 ============
    enable_gps_arg = DeclareLaunchArgument(
        'enable_gps',
        default_value='true',
        description='是否启用GPS记录（GPS不可用时自动禁用）'
    )

    gps_port_arg = DeclareLaunchArgument(
        'gps_port',
        default_value='/dev/wheeltec_gnss',
        description='GPS串口设备'
    )

    gps_baud_arg = DeclareLaunchArgument(
        'gps_baud',
        default_value='115200',
        description='GPS波特率'
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='~/wheeltec_ros2/outdoor_maps/',
        description='轨迹输出目录'
    )

    blind_back_arg = DeclareLaunchArgument(
        'blind_back_deg',
        default_value='0',
        # 2026-09-15 一度改成 90(想屏蔽跟车的人), 当天实测后改回 0:
        # 车往前开时刚压过的地面正好在正后方, 那里距离近、入射角好, 是地面回波
        # 最密的区域。屏蔽后每段少约 40% 地面格(9782->5986), 可通行区大幅缩水;
        # 该扇区原本还参与里程计约束。户外地面覆盖本就稀疏, 这个代价比"人被建
        # 进地图"大得多。人是移动的, 多走两圈会被后续观测覆盖掉。
        description='屏蔽雷达正后方扇区角度(全角度数); 户外建议 0, 地面覆盖优先'
    )

    clear_pcd_arg = DeclareLaunchArgument(
        'clear_pcd',
        default_value='false',
        description='true=启动前清掉上次建图遗留的PCD分段; false=有遗留就拒绝启动'
    )

    # ============ 节点定义 ============

    # 1. FAST-LIO2 建图（完全复用室内的）
    fastlio_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fastlio_dir, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'blind_back_deg': LaunchConfiguration('blind_back_deg'),
            'clear_pcd': LaunchConfiguration('clear_pcd')
        }.items()
    )

    # 2. RTK 驱动（条件启动）
    rtk_driver = Node(
        package='wheeltec_dual_rtk_driver',
        executable='dual_rtk_driver_node',
        name='rtk_driver',
        parameters=[{
            'port': LaunchConfiguration('gps_port'),
            'baud': LaunchConfiguration('gps_baud'),
            'gps_frame_id': 'gps_link'
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_gps'))
    )

    # 3. 轨迹记录节点（始终启动，GPS不可用时只记录雷达）
    trajectory_recorder = Node(
        package='wheeltec_outdoor_nav',
        executable='trajectory_recorder.py',
        name='trajectory_recorder',
        parameters=[{
            'lidar_odom_topic': '/Odometry',
            'gps_topic': '/gps/fix',
            'output_dir': LaunchConfiguration('output_dir'),
            'record_rate': 5.0
        }],
        output='screen'
    )

    return LaunchDescription([
        # 参数
        enable_gps_arg,
        gps_port_arg,
        gps_baud_arg,
        output_dir_arg,
        blind_back_arg,
        clear_pcd_arg,

        # 节点
        fastlio_mapping,
        rtk_driver,
        trajectory_recorder,
    ])
