import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory

gps_wpf_dir = get_package_share_directory("wheeltec_gps_driver")

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="mapviz",
            executable="mapviz",
            name="mapviz"
        ),
        launch_ros.actions.Node(
            package="swri_transform_util",
            executable="initialize_origin.py",
            name="initialize_origin",
            remappings=[("fix", "/gps/fix"),],
            parameters=[
                {"local_xy_frame": "map"},
                {"local_xy_origin": "swri"},
                {"local_xy_origins": """[
                    {"name": "swri",
                        "latitude": 22.9711153333,
                        "longitude": 113.9057971,
                        "altitude": 25.8,
                        "heading": 0.0},
                ]"""},
            ]
        )
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="swri_transform",
             arguments=["0", "0", "0", "0", "0", "0", "map", "origin"]
        )
    ])
