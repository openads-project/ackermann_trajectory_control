#!/usr/bin/env python

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter


def generate_launch_description():

    params_arg = DeclareLaunchArgument('params', default_value='params.yml')
    config = PathJoinSubstitution([
        get_package_share_directory("trajectory_control"), "config",
        LaunchConfiguration('params')
    ])

    node_name_default = 'trajectory_control_node'
    node_name_arg = DeclareLaunchArgument('node_name',
                                          default_value=node_name_default)
    input_trajectory_arg = DeclareLaunchArgument('input_trajectory',
                                            default_value='~/input_trajectory')
    input_ego_data_arg = DeclareLaunchArgument('input_ego_data',
                                             default_value='~/input_ego_data')
    output_arg = DeclareLaunchArgument('output',
                                             default_value='~/ctrl_cmds')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='False')

    node = Node(
        package="trajectory_control",
        executable="trajectory_control_node",
        name=LaunchConfiguration('node_name'),
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[config],
        remappings=[('~/input_trajectory', LaunchConfiguration('input_trajectory')),
                        ('~/input_ego_data', LaunchConfiguration('input_ego_data')),
                        ('~/ctrl_cmds', LaunchConfiguration('output'))]
    )

    node_group = GroupAction(actions=[
        SetParameter(name='use_sim_time',
                     value=LaunchConfiguration('use_sim_time'),
                     condition=LaunchConfigurationNotEquals(
                         'use_sim_time', "None")), node
    ])

    return LaunchDescription([
        params_arg,
        node_name_arg,
        input_trajectory_arg,
        input_ego_data_arg,
        output_arg,
        use_sim_time_arg,
        node_group
    ])
