#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    declare_my_param = DeclareLaunchArgument(
        'my_parameter',
        default_value='earth',  # Default value for reset_goal_button
        description='Parameter to view'
    )
    my_param = Node(
        package="cpp_parameters",
        executable="minimal_param_node",
        name="custom_minimal_param_node",
        output="screen",
        emulate_tty=True,
        parameters=[{"my_parameter": LaunchConfiguration('my_parameter')}]
    )

    ld = LaunchDescription()
    ld.add_action(declare_my_param)
    ld.add_action(my_param)

    return ld