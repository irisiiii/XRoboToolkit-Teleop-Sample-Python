import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Quaternion
from jaka_robot_interfaces.msg import RobotStateDual
from frame_sync_msgs.msg import StampedFloat64MultiArray
import math

def euler_to_quaternion(roll, pitch, yaw):
    """将欧拉角转换为四元数"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    
    return q

class RobotStateConverter(Node):
    def __init__(self):
        super().__init__('robot_state_converter')
        
        # 定义 QoS 配置 - 使用 RELIABLE 策略以兼容更多订阅者
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # 使用可靠传输
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1  # 只保留最新的一条消息
        )
        
        # 订阅机器人状态
        self.robot_subscription = self.create_subscription(
            RobotStateDual,
            '/robot_state_dual',
            self.robot_state_callback,
            qos_profile)
        
        # 订阅夹爪状态
        self.left_gripper_subscription = self.create_subscription(
            StampedFloat64MultiArray,
            '/left_arm/jiazhua_state',
            self.left_gripper_callback,
            qos_profile)
        
        self.right_gripper_subscription = self.create_subscription(
            StampedFloat64MultiArray,
            '/right_arm/jiazhua_state',
            self.right_gripper_callback,
            qos_profile)
        
        # 发布左右臂分离的JointState消息
        self.left_joint_publisher = self.create_publisher(JointState, '/left_arm/joint_states', qos_profile)
        self.right_joint_publisher = self.create_publisher(JointState, '/right_arm/joint_states', qos_profile)
        
        # 发布左右臂TCP位姿
        self.left_tcp_publisher = self.create_publisher(PoseStamped, '/left_arm/tcp_pose', qos_profile)
        self.right_tcp_publisher = self.create_publisher(PoseStamped, '/right_arm/tcp_pose', qos_profile)
        
        # 发布灵巧手关节状态
        self.left_gripper_publisher = self.create_publisher(StampedFloat64MultiArray, '/left_arm/gripper_state', qos_profile)
        self.right_gripper_publisher = self.create_publisher(StampedFloat64MultiArray, '/right_arm/gripper_state', qos_profile)
        
        # 保留原来的合并发布（向后兼容）
        self.combined_publisher = self.create_publisher(JointState, '/jaka_joint_state', qos_profile)
        
        self.get_logger().info('Robot State Converter started')
        self.get_logger().info('Subscribing to: /robot_state_dual, /lingqiaoshou')
        self.get_logger().info('Publishing arm data to: /left_arm/*, /right_arm/*')
        
    def robot_state_callback(self, msg):
        timestamp = self.get_clock().now().to_msg()
        
        # === 左臂关节状态 ===
        left_joint_state = JointState()
        left_joint_state.header.stamp = timestamp
        left_joint_state.header.frame_id = 'left_arm_base'
        left_joint_state.name = [f'left_joint_{i+1}' for i in range(len(msg.joint_pos_left.joint_values))]
        left_joint_state.position = list(msg.joint_pos_left.joint_values)
        self.left_joint_publisher.publish(left_joint_state)
        
        # === 右臂关节状态 ===
        right_joint_state = JointState()
        right_joint_state.header.stamp = timestamp
        right_joint_state.header.frame_id = 'right_arm_base'
        right_joint_state.name = [f'right_joint_{i+1}' for i in range(len(msg.joint_pos_right.joint_values))]
        right_joint_state.position = list(msg.joint_pos_right.joint_values)
        self.right_joint_publisher.publish(right_joint_state)
        
        # === 左臂TCP位姿 ===
        left_tcp_pose = PoseStamped()
        left_tcp_pose.header.stamp = timestamp
        left_tcp_pose.header.frame_id = 'left_arm_base'
        # 直接使用CartesianPose的字段
        left_tcp_pose.pose.position.x = msg.end_pose_left.x
        left_tcp_pose.pose.position.y = msg.end_pose_left.y
        left_tcp_pose.pose.position.z = msg.end_pose_left.z
        
        # 将欧拉角转换为四元数
        q = euler_to_quaternion(msg.end_pose_left.rx, msg.end_pose_left.ry, msg.end_pose_left.rz)
        left_tcp_pose.pose.orientation = q
        self.left_tcp_publisher.publish(left_tcp_pose)
        
        # === 右臂TCP位姿 ===
        right_tcp_pose = PoseStamped()
        right_tcp_pose.header.stamp = timestamp
        right_tcp_pose.header.frame_id = 'right_arm_base'
        # 直接使用CartesianPose的字段
        right_tcp_pose.pose.position.x = msg.end_pose_right.x
        right_tcp_pose.pose.position.y = msg.end_pose_right.y
        right_tcp_pose.pose.position.z = msg.end_pose_right.z
        
        # 将欧拉角转换为四元数
        q = euler_to_quaternion(msg.end_pose_right.rx, msg.end_pose_right.ry, msg.end_pose_right.rz)
        right_tcp_pose.pose.orientation = q
        self.right_tcp_publisher.publish(right_tcp_pose)
        
        # === 合并的关节状态（向后兼容）===
        combined_joint_state = JointState()
        combined_joint_state.header.stamp = timestamp
        combined_joint_state.header.frame_id = 'base_link'
        combined_joint_state.name = left_joint_state.name + right_joint_state.name
        combined_joint_state.position = left_joint_state.position + right_joint_state.position
        self.combined_publisher.publish(combined_joint_state)

    def left_gripper_callback(self, msg):
        # 直接转发左夹爪数据到 frame_sync 期望的话题
        self.left_gripper_publisher.publish(msg)
    
    def right_gripper_callback(self, msg):
        # 直接转发右夹爪数据到 frame_sync 期望的话题
        self.right_gripper_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    converter = RobotStateConverter()
    
    try:
        rclpy.spin(converter)
    except KeyboardInterrupt:
        pass
    finally:
        converter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()