from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction, Shutdown


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("frame_sync"), "config", "frame_sync.yaml"
    )
    # 声明launch参数
    output_frequency_arg = DeclareLaunchArgument(
        "output_frequency",
        default_value="30.0",
        description="Output frequency for frame sync node in Hz",
    )

    force_frequency_arg = DeclareLaunchArgument(
        "force_frequency",
        default_value="false",
        description="Whether to force frequency by repeating last frame",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level (debug, info, warn, error)",
    )

    return LaunchDescription(
        [
            # 声明参数
            output_frequency_arg,
            force_frequency_arg,
            log_level_arg,
            # 启动相机
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "launch",
                    "orbbec_camera",
                    "multi_camera_synced.launch.py",
                ],
                output="screen",
            ),
            # 延迟启动机械臂状态发布节点，确保相机先启动
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="piper",
                        executable="pub_dual_state_and_pose",
                        output="screen",
                        arguments=[
                            "--ros-args",
                            "--log-level",
                            LaunchConfiguration("log_level"),
                        ],
                    )
                ],
            ),
            # 延迟启动frame_sync节点，确保上游节点都已启动
            # 支持两种数据采集启动方式：
            # 1. 手动服务调用：ros2 service call /start_capture std_srvs/srv/Trigger "{}"
            # 2. 夹爪序列自动触发：快速操作左夹爪完成 闭合(<5)->开启(>20)->闭合(<5)->开启(>20) 序列
            TimerAction(
                period=6.0,
                actions=[
                    Node(
                        package="frame_sync",
                        executable="frame_sync_node",
                        output="screen",
                        parameters=[config_file],
                        arguments=[
                            "--ros-args",
                            "--log-level",
                            LaunchConfiguration("log_level"),
                        ],
                    )
                ],
            ),
        ]
    )
