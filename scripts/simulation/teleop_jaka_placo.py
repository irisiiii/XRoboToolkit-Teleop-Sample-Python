import os
import tyro

from xrobotoolkit_teleop.simulation.placo_teleop_controller import (
    PlacoTeleopController,
)
from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH


def main(
    robot_urdf_path: str = os.path.join(ASSET_PATH, "jaka/dual_arm_fixed.urdf"),
    scale_factor: float = 1.5,
):
    """
    Main function to run the JAKA dual arm teleoperation with Placo.
    """
    # JAKA双臂机器人配置
    # 根据您的URDF文件，左臂末端连杆是 "lt"，右臂末端连杆是 "rt"
    config = {
        "left_arm": {
            "link_name": "lt",  # 左臂末端工具连杆
            "pose_source": "left_controller",
            "control_trigger": "left_grip",
            "control_mode": "pose",  # 6DOF控制（位置+姿态）
        },
        "right_arm": {
            "link_name": "rt",  # 右臂末端工具连杆
            "pose_source": "right_controller", 
            "control_trigger": "right_grip",
            "control_mode": "pose",  # 6DOF控制（位置+姿态）
        },
    }

    # 创建并初始化遥操作控制器
    controller = PlacoTeleopController(
        robot_urdf_path=robot_urdf_path,
        manipulator_config=config,
        scale_factor=scale_factor,
    )

    # 添加关节正则化任务，保持机器人在自然姿态
    joints_task = controller.solver.add_joints_task()
    
    # 为JAKA双臂设置默认关节位置（所有关节设为0度，您可以根据需要调整）
    default_joints = {
        # 左臂默认位置
        "l-j1":-0.9552536062015364 ,
        "l-j2":-0.557702509182268 , 
        "l-j3": -0.5616294999992554,
        "l-j4": -1.0983706048650714,  # 轻微弯曲手肘
        "l-j5": 0.5683839242044734,
        "l-j6": -0.49378855197423577,
        "l-j7": 0.07195992505972622,
        # 右臂默认位置
        "r-j1": 0.0,
        "r-j2": 0.0,
        "r-j3": 0.0, 
        "r-j4": -0.5,  # 轻微弯曲手肘
        "r-j5": 0.0,
        "r-j6": 0.0,
        "r-j7": 0.0,
    }
    
    joints_task.set_joints(default_joints)
    joints_task.configure("joints_regularization", "soft", 1e-4)

    print("Starting JAKA dual arm teleoperation...")
    print("Control mapping:")
    print("  - Left controller -> Left arm (lt)")
    print("  - Right controller -> Right arm (rt)")
    print("  - Hold grip buttons to activate arm control")
    print("  - Open http://localhost:7000/static/ to view the robot")

    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
