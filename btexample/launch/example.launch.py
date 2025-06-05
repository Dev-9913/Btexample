# launch/example.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_module_list():
    return str([
        "btexample.nodes",     # your custom nodes
        "ros_bt_py.nodes",      # built-in BT node classes
        "ros_bt_py.ros_nodes",  # ROS-interface BT nodes
    ])

def generate_launch_description():
    ros_bt_py_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_bt_py"),
                "launch",
                "ros_bt_py.launch.py"
            ])
        ),
        launch_arguments={
            "node_modules": generate_module_list()
        }.items(),
    )

    return LaunchDescription([ros_bt_py_launch])
