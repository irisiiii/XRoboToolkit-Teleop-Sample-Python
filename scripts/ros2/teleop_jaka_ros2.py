import os
import time
import threading
from typing import Dict, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from jaka_robot_interfaces.msg import RobotStateDual, ServoJointCommand

# 导入XRoboToolkit的核心组件
from xrobotoolkit_teleop.common.data_logger import DataLogger
from xrobotoolkit_teleop.simulation.placo_teleop_controller import PlacoTeleopController
from xrobotoolkit_teleop.utils.geometry import R_HEADSET_TO_WORLD
from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH

# JAKA双臂机器人配置（与之前相同）
DEFAULT_JAKA_URDF_PATH = os.path.join(ASSET_PATH, "jaka/robot.urdf")

INITIAL_JOINT_POSITIONS = {
    'left': [-0.9552536062015364, -0.557702509182268, -0.5616294999992554, 
             -1.0983706048650714, 0.5683839242044734, -0.49378855197423577, 
             0.07195992505972622],
    'right': [-2.309978171599535, 0.7248177850607251, 3.7072713174536753, 
              -0.892910445320299, -0.3917915104876871, 0.5498659808408135, 
              -0.7628136028766417]
}

DEFAULT_JAKA_MANIPULATOR_CONFIG = {
    "left_arm": {
        "link_name": "lt",
        "pose_source": "left_controller",
        "control_trigger": "left_grip",
        "control_mode": "pose",
    },
    "right_arm": {
        "link_name": "rt",
        "pose_source": "right_controller", 
        "control_trigger": "right_grip",
        "control_mode": "pose",
    },
}


class JAKATeleopROS2Node(Node):
    """JAKA双臂机器人遥操作ROS2节点"""
    
    def __init__(self):
        super().__init__('jaka_teleop_node')
        
        # 数据记录器
        self.data_logger = DataLogger(log_dir="logs/jaka_dual_arm")
        self.is_logging = False
        self.prev_b_button_state = False
        
        # 创建Placo遥操作控制器（用于IK求解）
        self.placo_controller = PlacoTeleopController(
            robot_urdf_path=DEFAULT_JAKA_URDF_PATH,
            manipulator_config=DEFAULT_JAKA_MANIPULATOR_CONFIG,
            floating_base=False,
            R_headset_world=R_HEADSET_TO_WORLD,
            scale_factor=1.5,
            q_init=None,
            dt=0.02  # 50Hz
        )
        
        # ROS2发布者和订阅者
        self.servo_joint_pub = self.create_publisher(
            ServoJointCommand, 
            '/servo_joint_command', 
            10
        )
        
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        self.robot_state_sub = self.create_subscription(
            RobotStateDual,
            '/robot_state_dual',
            self.robot_state_callback,
            10
        )
        
        # 机器人状态
        self.current_joint_positions = {
            'left': INITIAL_JOINT_POSITIONS['left'].copy(),
            'right': INITIAL_JOINT_POSITIONS['right'].copy()
        }
        
        # 控制状态
        self.start_time = time.time()
        
        # 创建定时器
        self.control_timer = self.create_timer(0.02, self.control_loop)  # 50Hz
        
        self.get_logger().info("JAKA双臂遥操作节点已启动")
        self.get_logger().info("控制说明:")
        self.get_logger().info("  - 左手控制器控制左臂")
        self.get_logger().info("  - 右手控制器控制右臂")
        self.get_logger().info("  - 按住握持按钮激活控制")
        self.get_logger().info("  - 按B键开始/停止数据记录")
    
    def robot_state_callback(self, msg: RobotStateDual):
        """机器人状态回调"""
        try:
            if len(msg.joint_pos_left.joint_values) >= 7:
                self.current_joint_positions['left'] = list(msg.joint_pos_left.joint_values[:7])
            if len(msg.joint_pos_right.joint_values) >= 7:
                self.current_joint_positions['right'] = list(msg.joint_pos_right.joint_values[:7])
                
        except Exception as e:
            self.get_logger().error(f"处理机器人状态时出错: {e}")
    
    def update_placo_robot_state(self):
        """更新Placo机器人状态"""
        # 更新当前关节位置到Placo
        left_joints = self.current_joint_positions['left']
        right_joints = self.current_joint_positions['right']
        
        if len(left_joints) >= 7:
            self.placo_controller.placo_robot.state.q[7:14] = left_joints[:7]
        
        if len(right_joints) >= 7:
            self.placo_controller.placo_robot.state.q[14:21] = right_joints[:7]
        
        self.placo_controller.placo_robot.update_kinematics()
    
    def control_loop(self):
        """主控制循环"""
        try:
            # 更新机器人状态
            self.update_placo_robot_state()
            
            # 执行IK更新
            self.placo_controller._update_ik()
            
            # 检查按钮状态
            self.check_logging_button()
            
            # 获取求解后的关节角度
            left_target = self.placo_controller.placo_robot.state.q[7:14].copy()
            right_target = self.placo_controller.placo_robot.state.q[14:21].copy()
            
            # 检查哪些臂是激活的
            left_active = self.placo_controller.active.get("left_arm", False)
            right_active = self.placo_controller.active.get("right_arm", False)
            
            # 只有激活的臂才发送新的目标位置
            if left_active or right_active:
                self.send_joint_command(
                    left_joints=left_target if left_active else None,
                    right_joints=right_target if right_active else None
                )
            
            # 发布关节状态
            self.publish_joint_states(left_target, right_target)
            
            # 更新meshcat可视化
            self.placo_controller._update_placo_viz()
            
            # 记录数据
            if self.is_logging:
                self.log_current_state(left_target, right_target)
                
        except Exception as e:
            self.get_logger().error(f"控制循环出错: {e}")
    
    def send_joint_command(self, left_joints=None, right_joints=None):
        """发送关节命令"""
        try:
            servo_msg = ServoJointCommand()
            
            # 设置左臂目标（如果激活）
            if left_joints is not None:
                servo_msg.joint_pos_left.joint_values = left_joints.tolist()
            else:
                servo_msg.joint_pos_left.joint_values = self.current_joint_positions['left']
            
            # 设置右臂目标（如果激活）
            if right_joints is not None:
                servo_msg.joint_pos_right.joint_values = right_joints.tolist()
            else:
                servo_msg.joint_pos_right.joint_values = self.current_joint_positions['right']
            
            self.servo_joint_pub.publish(servo_msg)
            
        except Exception as e:
            self.get_logger().error(f"发送关节命令时出错: {e}")
    
    def publish_joint_states(self, left_joints, right_joints):
        """发布关节状态用于可视化"""
        try:
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            
            # 关节名称
            joint_names = ([f"l-j{i+1}" for i in range(7)] + 
                          [f"r-j{i+1}" for i in range(7)])
            
            # 关节位置
            joint_positions = left_joints.tolist() + right_joints.tolist()
            
            joint_state_msg.name = joint_names
            joint_state_msg.position = joint_positions
            
            self.joint_state_pub.publish(joint_state_msg)
            
        except Exception as e:
            self.get_logger().error(f"发布关节状态时出错: {e}")
    
    def check_logging_button(self):
        """检查数据记录按钮"""
        b_button_state = self.placo_controller.xr_client.get_button_state_by_name("B")
        
        if b_button_state and not self.prev_b_button_state:
            self.is_logging = not self.is_logging
            if self.is_logging:
                self.get_logger().info("开始数据记录")
            else:
                self.get_logger().info("停止数据记录，保存数据...")
                self.data_logger.save()
                self.data_logger.reset()
        
        self.prev_b_button_state = b_button_state
    
    def log_current_state(self, left_joints, right_joints):
        """记录当前状态"""
        try:
            timestamp = time.time() - self.start_time
            data_entry = {
                "timestamp": timestamp,
                "qpos": {
                    "left": self.current_joint_positions['left'],
                    "right": self.current_joint_positions['right']
                },
                "qpos_des": {
                    "left": left_joints.tolist(),
                    "right": right_joints.tolist()
                }
            }
            
            self.data_logger.add_entry(data_entry)
            
        except Exception as e:
            self.get_logger().error(f"记录数据时出错: {e}")


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        node = JAKATeleopROS2Node()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n收到键盘中断，正在关闭...")
    except Exception as e:
        print(f"发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
