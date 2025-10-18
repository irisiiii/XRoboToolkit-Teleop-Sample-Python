from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction, Shutdown
from launch.actions import ExecuteProcess

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("frame_sync"), "config", "frame_sync.yaml"
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level (debug, info, warn, error)",
    )
    return LaunchDescription(
        [
            log_level_arg,
            Node(
                package="frame_sync",
                executable="frame_sync_node",
                output="screen",
                parameters=[config_file],
                emulate_tty=True,      # <-- 关键：为子进程分配伪终端（pty）
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
            )
        ]
    )
