# 文件路径：/root/vr/frame_sync/launch/start_full_system.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取 frame_sync 配置文件路径
    config_file = os.path.join(
        get_package_share_directory("frame_sync"), "config", "frame_sync.yaml"
    )
    
    # 声明 launch 参数
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level (debug, info, warn, error)",
    )

    return LaunchDescription(
        [
            # 声明参数
            log_level_arg,
            
            # # [1] 启动 JAKA 机器人驱动 (立即启动)
            # ExecuteProcess(
            #     cmd=[
            #         "ros2",
            #         "launch",
            #         "jaka_robot_driver",
            #         "jaka_driver.launch.py",
            #     ],
            #     output="screen",
            #     name="jaka_driver",
            # ),
            
            # [2] 启动左相机 (延迟5秒)
            TimerAction(
                period=5.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "launch",
                            "realsense2_camera",
                            "rs_launch.py",
                            "serial_no:=_230322276887",
                            "camera_name:=left_camera",
                            "camera_namespace:=camera1",
                            "enable_color:=true",
                            "enable_depth:=false",
                            "pointcloud.enable:=false",
                        ],
                        output="screen",
                        name="left_camera",
                        respawn=False,  # 禁止自动重启
                    )
                ],
            ),
            
            # [3] 启动右相机 (延迟13秒，确保左相机完全启动)
            TimerAction(
                period=13.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "launch",
                            "realsense2_camera",
                            "rs_launch.py",
                            "serial_no:=_218622278744",
                            "camera_name:=right_camera",
                            "camera_namespace:=camera2",
                            "enable_color:=true",
                            "enable_depth:=false",
                            "pointcloud.enable:=false",
                        ],
                        output="screen",
                        name="right_camera",
                        respawn=False,  # 禁止自动重启
                    )
                ],
            ),
            
            # [4] 启动奥比中光相机 (延迟21秒)
            TimerAction(
                period=21.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "launch",
                            "orbbec_camera",
                            "gemini_330_series.launch.py",
                            "enable_color:=true",
                            "enable_depth:=false",
                            "enable_pointcloud:=false",
                            "color_fps:=30",
                        ],
                        output="screen",
                        name="orbbec_camera",
                        respawn=False,  # 禁止自动重启
                    )
                ],
            ),
            
            # [5] 启动机器人状态转换器 (延迟28秒)
            TimerAction(
                period=28.0,
                actions=[
                    Node(
                        package="robot_state_converter",
                        executable="converter",
                        output="screen",
                        name="robot_state_converter",
                        respawn=False,  # 禁止自动重启
                        arguments=[
                            "--ros-args",
                            "--log-level",
                            LaunchConfiguration("log_level"),
                        ],
                    )
                ],
            ),
            
            # [6] 启动夹爪驱动 (延迟32秒)
            TimerAction(
                period=32.0,
                actions=[
                    Node(
                        package="jiazhua_driver",
                        executable="jiazhua_node",
                        output="screen",
                        name="jiazhua_driver",
                        respawn=False,  # 禁止自动重启
                        arguments=[
                            "--ros-args",
                            "--log-level",
                            LaunchConfiguration("log_level"),
                        ],
                    )
                ],
            ),
            
            # [7] 启动帧同步节点 (延迟32秒，确保所有上游节点都已启动)
            # 支持两种数据采集启动方式：
            # 1. 手动服务调用：ros2 service call /start_capture std_srvs/srv/Trigger "{}"
            # 2. 夹爪序列自动触发：快速操作左夹爪完成 闭合(<5)->开启(>20)->闭合(<5)->开启(>20) 序列
            # TimerAction(
            #     period=5.0,
            #     actions=[
            #         Node(
            #             package="frame_sync",
            #             executable="frame_sync_node",
            #             output="screen",
            #             name="frame_sync_node",
            #             parameters=[config_file],
            #             arguments=[
            #                 "--ros-args",
            #                 "--log-level",
            #                 LaunchConfiguration("log_level"),
            #             ],
            #         )
            #     ],
            # ),
        ]
    )