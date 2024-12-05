#!/usr/bin/env python

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter

def generate_launch_description():

    params_arg = DeclareLaunchArgument('params', default_value='params.yml')
    config = PathJoinSubstitution([
        get_package_share_directory("ackermann_trajectory_control"), "config",
        LaunchConfiguration('params')
    ])

    node_name_default = 'ackermann_trajectory_control_node'
    node_name_arg = DeclareLaunchArgument('node_name', default_value=node_name_default)
    input_trajectory_arg = DeclareLaunchArgument('input_trajectory', default_value='~/input_trajectory')
    input_ego_data_arg = DeclareLaunchArgument('input_ego_data', default_value='~/input_ego_data')
    output_arg = DeclareLaunchArgument('output', default_value='~/ctrl_cmds')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='False')
    log_level_arg = DeclareLaunchArgument("log_level", default_value="info", description="ROS logging level (debug, info, warn, error, fatal)")

    node = Node(
        package="ackermann_trajectory_control",
        executable="ackermann_trajectory_control_node",
        name=LaunchConfiguration('node_name'),
        namespace="",
        output="screen",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[config],
        remappings=[('~/input_trajectory', LaunchConfiguration('input_trajectory')),
                    ('~/input_ego_data', LaunchConfiguration('input_ego_data')),
                    ('~/ctrl_cmds', LaunchConfiguration('output'))]
    )

    return LaunchDescription([
        params_arg,
        node_name_arg,
        input_trajectory_arg,
        input_ego_data_arg,
        output_arg,
        use_sim_time_arg,
        log_level_arg,
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        node
    ])
