#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp" // 使用float类型消息控制夹爪参数
#include "jiazhua_driver/jiazhua.h"
#include "jiazhua_interfaces/msg/jia_zhua_dual_cmd.hpp"
#include "frame_sync_msgs/msg/stamped_float64_multi_array.hpp"

using std::placeholders::_1;
using JiaZhuaDualCmd = jiazhua_interfaces::msg::JiaZhuaDualCmd;  // 引用消息类型

class JiaZhuaNode : public rclcpp::Node
{
public:
    JiaZhuaNode() : Node("jiazhua_node"), jiazhua_("can1")
    {
        subscription_ = this->create_subscription<JiaZhuaDualCmd>(
            "jiazhua_cmd", 10,
            std::bind(&JiaZhuaNode::cmd_callback, this, std::placeholders::_1));

        // === 新增：创建状态发布者 ===
        left_state_pub_ = this->create_publisher<frame_sync_msgs::msg::StampedFloat64MultiArray>(
            "/left_arm/jiazhua_state", 10);
        right_state_pub_ = this->create_publisher<frame_sync_msgs::msg::StampedFloat64MultiArray>(
            "/right_arm/jiazhua_state", 10);
            
        // === 新增：创建定时器定期发布状态 ===
        state_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 20Hz频率发布状态
            std::bind(&JiaZhuaNode::publish_states, this));
            
        // 初始化当前状态
        current_left_val_ = 1.0;   // 初始为松开状态
        current_right_val_ = 1.0;  // 初始为松开状态

        RCLCPP_INFO(this->get_logger(), "jiazhua_node started, listening to /jiazhua_cmd");
    }

private:
    void cmd_callback(const JiaZhuaDualCmd::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Left claw: val=%f speed=%f; Right claw: val=%f speed=%f",
                    msg->val_left, msg->speed_left, msg->val_right, msg->speed_right);

        // 依次控制左手夹爪（id=1）和右手夹爪（id=2）
        jiazhua_.set_jiazhua(1, msg->val_left, msg->speed_left, 1, 1, 1);
        jiazhua_.set_jiazhua(2, msg->val_right, msg->speed_right, 1, 1, 1);

         // === 新增：更新当前状态 ===
         current_left_val_ = msg->val_left;
         current_right_val_ = msg->val_right;
    }

    // === 新增：发布夹爪状态的函数 ===
    void publish_states()
    {
        auto now = this->get_clock()->now();
        
        // 发布左夹爪状态
        auto left_msg = frame_sync_msgs::msg::StampedFloat64MultiArray();
        left_msg.header.stamp = now;
        left_msg.header.frame_id = "left_gripper";
        left_msg.data = {current_left_val_};  // 只有一个值
        left_state_pub_->publish(left_msg);
        
        // 发布右夹爪状态
        auto right_msg = frame_sync_msgs::msg::StampedFloat64MultiArray();
        right_msg.header.stamp = now;
        right_msg.header.frame_id = "right_gripper";
        right_msg.data = {current_right_val_};  // 只有一个值
        right_state_pub_->publish(right_msg);
    }

    jiazhua jiazhua_;
    rclcpp::Subscription<JiaZhuaDualCmd>::SharedPtr subscription_;

    // === 新增：状态发布相关成员变量 ===
    rclcpp::Publisher<frame_sync_msgs::msg::StampedFloat64MultiArray>::SharedPtr left_state_pub_;
    rclcpp::Publisher<frame_sync_msgs::msg::StampedFloat64MultiArray>::SharedPtr right_state_pub_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    float current_left_val_;
    float current_right_val_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JiaZhuaNode>());
    rclcpp::shutdown();
    return 0;
}