import os
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    ultrasonic_pose = launch_ros.actions.Node(
            package='turn_on_wheeltec_robot', 
            executable='ultrasonic_enum.py', 
            name='ultrasonic_pose',
            #output='screen',
    )
    ultrasonic_point = launch_ros.actions.Node(
            package='turn_on_wheeltec_robot', 
            executable='ultrasonic_points.py', 
            name='ultrasonic_point',
            #output='screen',
    )
    
    base_to_obstacles = launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_obstacles',
            arguments=['0.10', '0.00', '0.1','0', '0','0','base_link','obstacles_link'],
    )
    
    return LaunchDescription([
        ultrasonic_pose,ultrasonic_point,base_to_obstacles
    ])

