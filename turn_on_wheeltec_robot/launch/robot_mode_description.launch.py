import os,yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml
from launch_ros.parameter_descriptions import ParameterFile

def load_yaml(file_path: Path) -> dict:
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)
    
def spawn_robot_nodes(context, *args, **kwargs):
    """真正构造节点的函数，由 OpaqueFunction 调用"""
    # 1. 取得参数文件路径
    param_file = LaunchConfiguration('wheeltec_param_yaml').perform(context)
    robot_model_file = LaunchConfiguration('robot_model_yaml').perform(context)
    cfg = load_yaml(Path(param_file))
    model_cfg = load_yaml(Path(robot_model_file))

    # 2. 取车型
    car_mode = LaunchConfiguration('car_mode').perform(context) or cfg['car_mode']
    print(f'car_mode:{car_mode}')
    if car_mode not in model_cfg['robot_model']:
        raise ValueError(f'Unknown car_mode "{car_mode}" in {param_file}')

    model_cfg = model_cfg['robot_model'][car_mode]

    # 3. 构造节点
    urdf_path = os.path.join(
        get_package_share_directory('wheeltec_robot_urdf'),
        'urdf',
        f'{car_mode}_robot.urdf'
    )
    
    actions = [
        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            arguments=[urdf_path],
        ),

        # base → laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=[*map(str, model_cfg['base_to_laser']), 'base_footprint', 'laser'],
        ),

        # base → camera
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera',
            arguments=[*map(str, model_cfg['base_to_camera']), 'base_footprint', 'camera_link'],
        ),
        
        # base → link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_link',
            arguments=[*map(str, model_cfg['base_to_link']), 'base_footprint', 'base_link'],
        ),
        # base → gyro
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_gyro',
            arguments=[*map(str, model_cfg['base_to_gyro']), 'base_footprint', 'gyro_link'],
        ),

        # base → radar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_radar',
            arguments=[*map(str, model_cfg['base_to_radar']), 'base_footprint', 'radar'],
        ),
    ]
    
    return actions

# ------------- Launch 描述 -------------
def generate_launch_description():
    return LaunchDescription([
        # 1. 声明参数
        DeclareLaunchArgument(
            'wheeltec_param_yaml',
            default_value=os.path.join(
                get_package_share_directory('turn_on_wheeltec_robot'),
                'config', 'wheeltec_param.yaml'),
            description='Path to wheeltec_param.yaml'
        ),
        DeclareLaunchArgument(
            'robot_model_yaml',
            default_value=os.path.join(
                get_package_share_directory('turn_on_wheeltec_robot'),
                'config', 'robot_model.yaml'),
            description='Path to robot_model.yaml'
        ),
        DeclareLaunchArgument(
            'car_mode',
            default_value='',   # 空字符串表示使用 yaml 里的默认值
            description='Which robot mode to launch (mini_akm, mini_mec...)'
        ),

        # 2. 用 OpaqueFunction 在运行时解析 yaml 并构造节点
        OpaqueFunction(function=spawn_robot_nodes)
    ])

