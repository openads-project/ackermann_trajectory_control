#!/usr/bin/env python

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from tracetools_launch.action import Trace

def generate_launch_description():

    params_arg = DeclareLaunchArgument('params', default_value='params.yml')
    config = PathJoinSubstitution([
        get_package_share_directory("ackermann_trajectory_control"), "config",
        LaunchConfiguration('params')
    ])

    name_default = 'ackermann_trajectory_control'
    name_arg = DeclareLaunchArgument('name', default_value=name_default)
    trajectory_topic_arg = DeclareLaunchArgument('trajectory_topic', default_value='~/trajectory')
    ego_data_topic_arg = DeclareLaunchArgument('ego_data_topic', default_value='~/ego_data')
    controls_topic_arg = DeclareLaunchArgument('controls_topic', default_value='~/controls')
    lat_control_active_arg = DeclareLaunchArgument('lat_control_active_topic', default_value='~/lat_control_active')
    lon_control_active_arg = DeclareLaunchArgument('lon_control_active_topic', default_value='~/lon_control_active')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='False')
    trace_arg = DeclareLaunchArgument('trace', default_value='False', description='Enable tracing')
    log_level_arg = DeclareLaunchArgument("log_level", default_value="info", description="ROS logging level (debug, info, warn, error, fatal)")

    node = Node(
        package="ackermann_trajectory_control",
        executable="ackermann_trajectory_control_node",
        name=LaunchConfiguration('name'),
        namespace="",
        output="screen",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[config],
        remappings=[('~/trajectory', LaunchConfiguration('trajectory_topic')),
                    ('~/ego_data', LaunchConfiguration('ego_data_topic')),
                    ('~/controls', LaunchConfiguration('controls_topic')),
                    ('~/lat_control_active', LaunchConfiguration('lat_control_active_topic')),
                    ('~/lon_control_active', LaunchConfiguration('lon_control_active_topic'))]
    )

    trace_action = Trace(
        session_name='trace',
        dual_session=True,
        condition=IfCondition(LaunchConfiguration('trace')),
    )

    return LaunchDescription([
        params_arg,
        name_arg,
        trajectory_topic_arg,
        ego_data_topic_arg,
        controls_topic_arg,
        lat_control_active_arg,
        lon_control_active_arg,
        use_sim_time_arg,
        trace_arg,
        log_level_arg,
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        node,
        trace_action
    ])
