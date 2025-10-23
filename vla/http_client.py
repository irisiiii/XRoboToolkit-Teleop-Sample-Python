#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GR00T + Piper单臂HTTP客户端 (简化版)
参考官方示例，不使用RTC，直接请求-执行模式
"""

import time
import numpy as np
import requests
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
import json
import cv2

# 导入自定义消息类型
from jaka_robot_interfaces.msg import ServoJointCommand, JointValue
from jiazhua_interfaces.msg import JiaZhuaDualCmd
from frame_sync_msgs.msg import StampedFloat64MultiArray


class SimpleGr00tClient(Node):
    """简单的GR00T HTTP客户端"""
    
    def __init__(self, server_host="192.168.1.88", server_port=8000):
        super().__init__('groot_client')
        
        self.server_url = f"http://{server_host}:{server_port}/act"
        self.bridge = CvBridge()
        
        # 数据缓存
        self.wrist_image = None
        self.top_image = None
        self.joint_positions = None
        self.gripper_position = None
        
        # 订阅话题
        self.create_subscription(
            Image, 
            '/camera1/left_camera/color/image_rect_raw',
            self.wrist_image_callback, 
            10
        )
        self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.top_image_callback,
            10
        )
        self.create_subscription(
            JointState,
            '/left_arm/joint_states',
            self.joint_state_callback,
            10
        )
        self.create_subscription(
            StampedFloat64MultiArray,
            '/left_arm/jiazhua_state',
            self.gripper_callback,
            10
        )
        
        # 发布话题
        self.arm_pub = self.create_publisher(
            ServoJointCommand,
            '/servo_joint_command',
            10
        )
        self.gripper_pub = self.create_publisher(
            JiaZhuaDualCmd,
            '/jiazhua_cmd',
            10
        )
        
        print(f"✓ 客户端初始化完成，服务器: {self.server_url}")
    
    def wrist_image_callback(self, msg):
        """左臂相机回调"""
        self.wrist_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    
    def top_image_callback(self, msg):
        """头部相机回调"""
        self.top_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    
    def joint_state_callback(self, msg):
        """关节状态回调"""
        if len(msg.position) >= 7:
            self.joint_positions = np.array(msg.position[:7], dtype=np.float32)
    
    def gripper_callback(self, msg):
        """夹爪状态回调"""
        if len(msg.data) > 0:
            self.gripper_position = np.array([msg.data[0]], dtype=np.float32)
    
    def get_observation(self):
        """获取当前观测数据"""
        if (self.wrist_image is None or self.top_image is None or 
            self.joint_positions is None or self.gripper_position is None):
            return None
        
        # Resize wrist_image to 480x640
        wrist_image_resized = cv2.resize(self.wrist_image, (640, 480))
        
        # 构造observation字典，键名需要匹配服务器期望的格式
        obs = {
            "video.wrist_left": np.expand_dims(
                wrist_image_resized, axis=0
            ).astype(np.uint8),
            "video.top": np.expand_dims(
                self.top_image, axis=0
            ).astype(np.uint8),
            "state.left_arm_joint": np.expand_dims(
                self.joint_positions,
                axis=0
            ).astype(np.float32),
            "state.left_gripper": np.expand_dims(
                self.gripper_position,
                axis=0
            ).astype(np.float32),
        }
        
        return obs
    
    def send_action(self, action):
        """发送动作到机械臂和夹爪
        
        Args:
            action: 8维数组 [7个关节角度, 1个夹爪位置]
        """
        # 发布机械臂指令
        arm_cmd = ServoJointCommand()
        arm_cmd.joint_pos_left = JointValue()
        arm_cmd.joint_pos_left.joint_values = action[:7].tolist()
        arm_cmd.joint_pos_right = JointValue()
        arm_cmd.joint_pos_right.joint_values = [0.0] * 7  # 右臂不动
        self.arm_pub.publish(arm_cmd)
        
        # 发布夹爪指令
        gripper_cmd = JiaZhuaDualCmd()
        gripper_cmd.val_left = float(action[7])
        gripper_cmd.speed_left = 0.5
        gripper_cmd.val_right = 1.0  # 右手不动，保持张开
        gripper_cmd.speed_right = 0.5
        self.gripper_pub.publish(gripper_cmd)
    
    def request_action(self, obs, task_description="Complete the task"):
        """向服务器请求动作"""
        try:
            # 添加任务描述
            obs["annotation.human.action.task_description"] = [task_description]
            
            # 将numpy数组转换为列表以便JSON序列化
            obs_serializable = {}
            for key, value in obs.items():
                if isinstance(value, np.ndarray):
                    obs_serializable[key] = value.tolist()
                else:
                    obs_serializable[key] = value
            
            # 发送HTTP请求
            response = requests.post(
                self.server_url,
                json={"observation": obs_serializable},
                timeout=10.0
            )
            
            if response.status_code == 200:
                action_data = response.json()
                return action_data
            else:
                print(f"❌ 服务器返回错误: {response.status_code}")
                return None
                
        except Exception as e:
            print(f"❌ 请求失败: {e}")
            return None
    
    def run_control_loop(self, task_description="Pick up the object", 
                        max_steps=1000, control_hz=20):
        """运行控制循环"""
        print(f"=== 开始控制循环 ===")
        print(f"任务: {task_description}")
        print(f"控制频率: {control_hz} Hz")
        
        control_interval = 1.0 / control_hz
        step_count = 0
        
        # 等待数据就绪
        print("等待传感器数据...")
        while (self.wrist_image is None or self.top_image is None or
               self.joint_positions is None or self.gripper_position is None):
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        print("✓ 传感器数据就绪")
        
        try:
            while step_count < max_steps:
                loop_start = time.time()
                
                # 1. 获取观测
                obs = self.get_observation()
                if obs is None:
                    print("⚠️ 观测数据不完整")
                    time.sleep(0.1)
                    continue
                
                # 2. 请求动作chunk
                print(f"\n[步骤 {step_count}] 请求动作...")
                action_chunk = self.request_action(obs, task_description)
                
                if action_chunk is None or 'action' not in action_chunk:
                    print("❌ 未获取到有效动作")
                    break
                
                # 3. 执行动作chunk（假设返回20步，每步50ms）
                actions = np.array(action_chunk['action'])
                if len(actions.shape) == 1:
                    actions = np.expand_dims(actions, axis=0)
                
                num_actions = actions.shape[0]
                print(f"✓ 收到 {num_actions} 步动作，开始执行...")
                
                # 执行每一步动作
                for i in range(num_actions):
                    action_start = time.time()
                    
                    # 发送动作
                    self.send_action(actions[i])
                    
                    # 处理ROS回调
                    rclpy.spin_once(self, timeout_sec=0.0)
                    
                    # 控制频率
                    elapsed = time.time() - action_start
                    sleep_time = control_interval - elapsed
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                    
                    step_count += 1
                    
                    if step_count >= max_steps:
                        break
                
                # 打印循环时间
                loop_time = time.time() - loop_start
                print(f"本轮耗时: {loop_time:.2f}秒 (请求+执行{num_actions}步)")
                
        except KeyboardInterrupt:
            print("\n用户中断")
        except Exception as e:
            print(f"❌ 控制循环异常: {e}")
            import traceback
            traceback.print_exc()
        finally:
            print(f"=== 控制循环结束，共执行 {step_count} 步 ===")


def main():
    """主函数"""
    rclpy.init()
    
    client = SimpleGr00tClient(
        server_host="192.168.1.88",
        server_port=8000
    )
    
    try:
        # 运行控制循环
        client.run_control_loop(
            task_description="Pick up the radish plush toy and put it into the basket",
            max_steps=10000,
            control_hz=20
        )
    except KeyboardInterrupt:
        print("\n程序被中断")
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
