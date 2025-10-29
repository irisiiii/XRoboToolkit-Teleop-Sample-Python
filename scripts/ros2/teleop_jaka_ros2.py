import os
import time
import threading
from typing import Dict, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from jaka_robot_interfaces.msg import RobotStateDual, ServoJointCommand
from jiazhua_interfaces.msg import JiaZhuaDualCmd
from frame_sync_msgs.msg import StampedFloat64MultiArray

# 导入XRoboToolkit的核心组件
from xrobotoolkit_teleop.common.data_logger import DataLogger
from xrobotoolkit_teleop.simulation.placo_teleop_controller import PlacoTeleopController
from xrobotoolkit_teleop.utils.geometry import (
    R_HEADSET_TO_WORLD, 
    apply_delta_pose, 
    quat_diff_as_angle_axis
)
from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH
import meshcat.transformations as tf

# JAKA双臂机器人配置
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
        
        # === 修复：正确创建数据记录器和日志状态 ===
        self.data_logger = DataLogger(log_dir="logs/jaka_dual_arm")
        self.is_logging = False
        self.prev_b_button_state = False
        
        # 回初始位姿按钮状态跟踪
        self.prev_x_button_state = False  # 左手柄X键
        self.prev_a_button_state = False  # 右手柄A键
        
        # 创建Placo遥操作控制器（用于IK求解）
        # 注意：这里会自动创建XrClient实例
        self.placo_controller = PlacoTeleopController(
            robot_urdf_path=DEFAULT_JAKA_URDF_PATH,
            manipulator_config=DEFAULT_JAKA_MANIPULATOR_CONFIG,
            floating_base=False,
            R_headset_world=R_HEADSET_TO_WORLD,
            scale_factor=1.5,
            q_init=None,
            dt=0.02  # 50Hz
        )
        
        # 位姿记忆状态变量
        # 末端执行器参考位姿（机器人坐标系）
        self.ref_ee_xyz = {"left_arm": None, "right_arm": None}
        self.ref_ee_quat = {"left_arm": None, "right_arm": None}
        
        # 控制器参考位姿（XR坐标系）
        self.ref_controller_xyz = {"left_arm": None, "right_arm": None}
        self.ref_controller_quat = {"left_arm": None, "right_arm": None}
        
        # 激活状态
        self.active = {"left_arm": False, "right_arm": False}
        
        # 夹爪状态
        self.gripper_states = {"left": 1.0, "right": 1.0}  # 1.0=松开, 0.0=夹紧
        self.gripper_speed = 0.5  # 夹爪速度
        
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
        
        # 夹爪命令发布者
        self.gripper_cmd_pub = self.create_publisher(
            JiaZhuaDualCmd,
            '/jiazhua_cmd',
            10
        )
        
        # 夹爪状态订阅者
        self.left_gripper_state_sub = self.create_subscription(
            StampedFloat64MultiArray,
            '/left_arm/jiazhua_state',
            self.left_gripper_state_callback,
            10
        )
        
        self.right_gripper_state_sub = self.create_subscription(
            StampedFloat64MultiArray,
            '/right_arm/jiazhua_state',
            self.right_gripper_state_callback,
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
        self.get_logger().info("  - 松开握持按钮保持位姿记忆")
        self.get_logger().info("  - 按B键开始/停止数据记录")
        self.get_logger().info("  - 按前方按钮控制夹爪开合")
        self.get_logger().info("  - 按左手柄X键，左臂回初始位姿")
        self.get_logger().info("  - 按右手柄A键，右臂回初始位姿")
        self.get_logger().info("✅ XrClient使用placo_controller实例，避免重复创建")
        self.get_logger().info("✅ 数据记录器和日志状态已正确初始化")
        self.get_logger().info("✅ 夹爪控制功能已添加")
        self.get_logger().info("✅ 回初始位姿功能已添加")
    
    def robot_state_callback(self, msg: RobotStateDual):
        """机器人状态回调"""
        try:
            if len(msg.joint_pos_left.joint_values) >= 7:
                self.current_joint_positions['left'] = list(msg.joint_pos_left.joint_values[:7])
            if len(msg.joint_pos_right.joint_values) >= 7:
                self.current_joint_positions['right'] = list(msg.joint_pos_right.joint_values[:7])
                
        except Exception as e:
            self.get_logger().error(f"处理机器人状态时出错: {e}")
    
    def left_gripper_state_callback(self, msg: StampedFloat64MultiArray):
        """左夹爪状态回调"""
        try:
            if len(msg.data) > 0:
                self.gripper_states["left"] = msg.data[0]
        except Exception as e:
            self.get_logger().error(f"处理左夹爪状态时出错: {e}")
    
    def right_gripper_state_callback(self, msg: StampedFloat64MultiArray):
        """右夹爪状态回调"""
        try:
            if len(msg.data) > 0:
                self.gripper_states["right"] = msg.data[0]
        except Exception as e:
            self.get_logger().error(f"处理右夹爪状态时出错: {e}")
    
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
    
    def _get_link_pose(self, link_name: str):
        """获取连杆的当前世界位姿"""
        T_world_link = self.placo_controller.placo_robot.get_T_world_frame(link_name)
        pos = T_world_link[:3, 3]
        quat = tf.quaternion_from_matrix(T_world_link)
        return pos, quat
    
    def _process_xr_pose(self, xr_pose, src_name):
        """处理XR控制器位姿，计算增量变化"""
        # 获取位置和方向
        controller_xyz = np.array([xr_pose[0], xr_pose[1], xr_pose[2]])
        controller_quat = [
            xr_pose[6],  # w
            xr_pose[3],  # x
            xr_pose[4],  # y
            xr_pose[5],  # z
        ]

        # 应用坐标系变换
        controller_xyz = R_HEADSET_TO_WORLD @ controller_xyz

        R_transform = np.eye(4)
        R_transform[:3, :3] = R_HEADSET_TO_WORLD
        R_quat = tf.quaternion_from_matrix(R_transform)
        controller_quat = tf.quaternion_multiply(
            tf.quaternion_multiply(R_quat, controller_quat),
            tf.quaternion_conjugate(R_quat),
        )

        # 计算增量变化（核心逻辑）
        if self.ref_controller_xyz[src_name] is None:
            # 第一次激活，设置参考位置
            self.ref_controller_xyz[src_name] = controller_xyz
            self.ref_controller_quat[src_name] = controller_quat

            delta_xyz = np.zeros(3)
            delta_rot = np.array([0.0, 0.0, 0.0])
        else:
            # 计算增量变化
            delta_xyz = (controller_xyz - self.ref_controller_xyz[src_name]) * self.placo_controller.scale_factor
            delta_rot = quat_diff_as_angle_axis(self.ref_controller_quat[src_name], controller_quat)

        return delta_xyz, delta_rot
    
    def update_gripper_control(self):
        """更新夹爪控制（使用前方按钮）"""
        try:
            # 获取左右前方按钮状态
            left_trigger = self.placo_controller.xr_client.get_key_value_by_name("left_trigger")
            right_trigger = self.placo_controller.xr_client.get_key_value_by_name("right_trigger")
            
            # 前方按钮值范围：0.0（完全松开）到 1.0（完全按下）
            # 夹爪值范围：1.0（松开）到 0.0（夹紧）
            # 所以需要反向映射：trigger_value -> 1.0 - trigger_value
            
            left_gripper_val = 1.0 - left_trigger
            right_gripper_val = 1.0 - right_trigger
            
            # 创建夹爪命令消息
            gripper_cmd = JiaZhuaDualCmd()
            gripper_cmd.val_left = left_gripper_val
            gripper_cmd.speed_left = self.gripper_speed
            gripper_cmd.val_right = right_gripper_val
            gripper_cmd.speed_right = self.gripper_speed
            
            # 发布夹爪命令
            self.gripper_cmd_pub.publish(gripper_cmd)
            
            # 可选：记录夹爪状态变化
            if abs(left_gripper_val - self.gripper_states.get("left", 1.0)) > 0.1:
                self.get_logger().info(f"左夹爪: {left_gripper_val:.2f} (前方按钮: {left_trigger:.2f})")
            if abs(right_gripper_val - self.gripper_states.get("right", 1.0)) > 0.1:
                self.get_logger().info(f"右夹爪: {right_gripper_val:.2f} (前方按钮: {right_trigger:.2f})")
                
        except Exception as e:
            self.get_logger().error(f"夹爪控制出错: {e}")
    
    def _update_ik_with_memory(self):
        """带位姿记忆的IK更新（核心修改）"""
        # 更新机器人状态
        self.update_placo_robot_state()
        self.placo_controller.placo_robot.update_kinematics()

        for src_name, config in DEFAULT_JAKA_MANIPULATOR_CONFIG.items():
            # 使用placo_controller的xr_client
            xr_grip_val = self.placo_controller.xr_client.get_key_value_by_name(config["control_trigger"])
            current_active = xr_grip_val > 0.9

            if current_active:
                # 第一次激活时记录参考位姿
                if not self.active[src_name]:
                    # 从不激活变为激活：记录当前机器人末端位姿作为参考
                    self.get_logger().info(f"{src_name} 激活")
                    self.ref_ee_xyz[src_name], self.ref_ee_quat[src_name] = self._get_link_pose(config["link_name"])
                    # 清空控制器参考位置，让_process_xr_pose重新设置
                    self.ref_controller_xyz[src_name] = None
                    self.ref_controller_quat[src_name] = None

                # 获取当前XR控制器位姿
                xr_pose = self.placo_controller.xr_client.get_pose_by_name(config["pose_source"])
                delta_xyz, delta_rot = self._process_xr_pose(xr_pose, src_name)
                
                # 基于参考位姿计算目标位姿
                if self.ref_ee_xyz[src_name] is not None:
                    if config.get("control_mode", "pose") == "position":
                        # 位置控制模式
                        target_xyz = self.ref_ee_xyz[src_name] + delta_xyz
                        self.placo_controller.effector_task[src_name].target_world = target_xyz
                    else:
                        # 完整位姿控制模式
                        target_xyz, target_quat = apply_delta_pose(
                            self.ref_ee_xyz[src_name],
                            self.ref_ee_quat[src_name],
                            delta_xyz,
                            delta_rot,
                        )
                        target_pose = tf.quaternion_matrix(target_quat)
                        target_pose[:3, 3] = target_xyz
                        self.placo_controller.effector_task[src_name].T_world_frame = target_pose

            else:
                # 停用时清空参考位姿
                if self.active[src_name]:
                    # 从激活变为不激活
                    self.get_logger().info(f"{src_name} 停用")
                    self.ref_ee_xyz[src_name] = None
                    self.ref_ee_quat[src_name] = None
                    self.ref_controller_xyz[src_name] = None
                    self.ref_controller_quat[src_name] = None

            # 更新激活状态
            self.active[src_name] = current_active

        # 执行IK求解
        try:
            self.placo_controller.solver.solve(True)
        except RuntimeError as e:
            self.get_logger().error(f"IK solver failed: {e}")
    
    def control_loop(self):
        """主控制循环"""
        try:
            # 使用带记忆的IK更新
            self._update_ik_with_memory()
            
            # 更新夹爪控制
            self.update_gripper_control()
            
            # 检查按钮状态
            self.check_logging_button()
            
            # 检查回初始位姿按钮
            self.check_reset_buttons()
            
            # 获取求解后的关节角度
            left_target = self.placo_controller.placo_robot.state.q[7:14].copy()
            right_target = self.placo_controller.placo_robot.state.q[14:21].copy()
            
            # 检查哪些臂是激活的
            left_active = self.active.get("left_arm", False)
            right_active = self.active.get("right_arm", False)
            
            # 只有激活的臂才发送新的目标位置
            if left_active or right_active:
                self.send_joint_command(
                    left_joints=left_target if left_active else None,
                    right_joints=right_target if right_active else None
                )
            
            # 发布关节状态
            self.publish_joint_states(left_target, right_target)
            
            # 更新meshcat可视化
            if hasattr(self.placo_controller, '_update_placo_viz'):
                self.placo_controller._update_placo_viz()
            
            # 记录数据
            if self.is_logging:
                self.log_current_state(left_target, right_target)
                
        except Exception as e:
            self.get_logger().error(f"控制循环出错: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
    
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
        # 使用placo_controller的xr_client
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
    
    def check_reset_buttons(self):
        """检查回初始位姿按钮"""
        # 获取X键状态（左手柄）
        x_button_state = self.placo_controller.xr_client.get_button_state_by_name("X")
        
        # 检测X键按下事件（边沿触发）
        if x_button_state and not self.prev_x_button_state:
            self.get_logger().info("🔄 左臂回初始位姿")
            # 发送左臂初始关节位置
            self.send_joint_command(
                left_joints=np.array(INITIAL_JOINT_POSITIONS['left']),
                right_joints=None
            )
            # 同时更新Placo机器人状态
            self.placo_controller.placo_robot.state.q[7:14] = INITIAL_JOINT_POSITIONS['left']
            # 停用左臂控制，清空参考位姿
            self.active["left_arm"] = False
            self.ref_ee_xyz["left_arm"] = None
            self.ref_ee_quat["left_arm"] = None
            self.ref_controller_xyz["left_arm"] = None
            self.ref_controller_quat["left_arm"] = None
        
        self.prev_x_button_state = x_button_state
        
        # 获取A键状态（右手柄）
        a_button_state = self.placo_controller.xr_client.get_button_state_by_name("A")
        
        # 检测A键按下事件（边沿触发）
        if a_button_state and not self.prev_a_button_state:
            self.get_logger().info("🔄 右臂回初始位姿")
            # 发送右臂初始关节位置
            self.send_joint_command(
                left_joints=None,
                right_joints=np.array(INITIAL_JOINT_POSITIONS['right'])
            )
            # 同时更新Placo机器人状态
            self.placo_controller.placo_robot.state.q[14:21] = INITIAL_JOINT_POSITIONS['right']
            # 停用右臂控制，清空参考位姿
            self.active["right_arm"] = False
            self.ref_ee_xyz["right_arm"] = None
            self.ref_ee_quat["right_arm"] = None
            self.ref_controller_xyz["right_arm"] = None
            self.ref_controller_quat["right_arm"] = None
        
        self.prev_a_button_state = a_button_state
    
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
                },
                "active_status": {
                    "left": self.active.get("left_arm", False),
                    "right": self.active.get("right_arm", False)
                },
                "gripper_states": {
                    "left": self.gripper_states.get("left", 1.0),
                    "right": self.gripper_states.get("right", 1.0)
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
