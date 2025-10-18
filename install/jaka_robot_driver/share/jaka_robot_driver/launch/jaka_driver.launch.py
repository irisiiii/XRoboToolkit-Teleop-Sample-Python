#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包的共享目录路径
    pkg_share = get_package_share_directory('jaka_robot_driver')
    
    # 声明参数文件路径参数
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            pkg_share, 'config', 'jaka_driver_params.yaml'
        ]),
        description='参数文件路径'
    )
    
    # 声明机器人IP参数
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.2.200',
        description='机器人IP地址'
    )
    
    # 声明仿真模式参数
    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='是否启用仿真模式'
    )
    
    # 真实模式节点
    jaka_driver_node = Node(
        package='jaka_robot_driver',
        executable='jaka_driver_node',
        name='jaka_driver_node',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('simulation')),
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'robot_ip': LaunchConfiguration('robot_ip'),
            }
        ],
        arguments=[
            '--ros-args',
            '--log-level', 'jaka_driver_node:=info'
        ]
    )
    
    # 仿真模式节点
    jaka_simulator_node = Node(
        package='jaka_robot_driver',
        executable='jaka_simulator_node',
        name='jaka_simulator_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('simulation')),
        arguments=[
            '--ros-args',
            '--log-level', 'jaka_simulator_node:=info'
        ]
    )
    
    return LaunchDescription([
        # 声明参数
        params_file_arg,
        robot_ip_arg,
        simulation_arg,
        
        # 启动节点（根据仿真参数选择）
        jaka_driver_node,
        jaka_simulator_node
    ]) 