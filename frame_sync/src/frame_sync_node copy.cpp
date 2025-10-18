#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "frame_sync/ConsoleUI.hpp"
#include "frame_sync/DataCollecter.hpp"
#include "frame_sync/DataWriter.hpp"
#include "frame_sync_msgs/msg/stamped_float64_multi_array.hpp"
#include "frame_sync_msgs/msg/synced_data.hpp"
class FrameSyncNode : public rclcpp::Node {
 public:
  FrameSyncNode() : Node("frame_sync_node") {
    declare_parameters();
    data_collector_ = std::make_shared<DataCollecter>(
        this->get_parameter("dataset_save_path").as_string());
    data_collector_->prepare();
    data_collector_->start();

    ui().log("Frame Sync Node has started.");

    enable_limit_numbers_of_frames_to_save_ =
        this->get_parameter("enable_limit_numbers_of_frames_to_save").as_bool();

    limit_numbers_of_frames_to_save_ =
        this->get_parameter("limit_numbers_of_frames_to_save").as_int();
    enable_save_dataset_ = this->get_parameter("enable_save_dataset").as_bool();
    ui().log("enable_save_dataset: %d", enable_save_dataset_);
    ui().log("enable_limit_numbers_of_frames_to_save: %d",
             enable_limit_numbers_of_frames_to_save_);
    ui().log("limit_numbers_of_frames_to_save: %d",
             limit_numbers_of_frames_to_save_);
    ui().getStatus().session_target_count = limit_numbers_of_frames_to_save_;
    number_of_frames_processed_ = 0;

    // 初始化夹爪状态监听相关变量
    gripper_trigger_state_ = GripperTriggerState::WAITING_FIRST_CLOSE;
    gripper_sequence_start_time_ = this->get_clock()->now();
    gripper_trigger_timeout_sec_ =
        this->get_parameter("gripper_trigger_timeout_sec").as_double();
    gripper_trigger_countdown_sec_ =
        this->get_parameter("gripper_trigger_countdown_sec").as_double();
    enable_continuous_collection_ =
        this->get_parameter("enable_continuous_collection").as_bool();

    // 初始化数据采集状态管理
    collection_state_ = CollectionState::WAITING_FOR_TRIGGER;
    ui().getStatus().state = CollectionState::WAITING_FOR_TRIGGER;
    countdown_start_time_ = this->get_clock()->now();
    countdown_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&FrameSyncNode::countdownTimerCallback, this));
    countdown_timer_->cancel();  // 初始时停止倒计时定时器

    // 初始化时间记录相关变量
    has_collection_started_ = false;

    // 初始化相机订阅者数组
    std::vector<std::string> camera_topics =
        this->get_parameter("camera_topic_names").as_string_array();
    for (const auto &topic : camera_topics) {
      image_subs_.emplace_back(
          std::make_shared<
              message_filters::Subscriber<sensor_msgs::msg::Image>>(this,
                                                                    topic));
    }
    // 初始化机械臂订阅者数组
    std::vector<std::string> arm_topics =
        this->get_parameter("arm_joints_topic_names").as_string_array();
    for (const auto &topic : arm_topics) {
      arm_subs_.emplace_back(
          std::make_shared<
              message_filters::Subscriber<sensor_msgs::msg::JointState>>(
              this, topic));
    }
    std::vector<std::string> pose_topics =
        this->get_parameter("tcp_topic_names").as_string_array();
    for (const auto &topic : pose_topics) {
      pose_subs_.emplace_back(
          std::make_shared<
              message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(
              this, topic));
    }
    std::vector<std::string> gripper_state_topics =
        this->get_parameter("gripper_topic_names").as_string_array();
    for (const auto &topic : gripper_state_topics) {
      gripper_state_subs_.emplace_back(
          std::make_shared<message_filters::Subscriber<
              frame_sync_msgs::msg::StampedFloat64MultiArray>>(this, topic));
    }
    SyncedData_pub_ = this->create_publisher<frame_sync_msgs::msg::SyncedData>(
        "/syncedData", 500);
    // 保留原有的手动服务调用方式，同时支持夹爪状态监听自动触发
    start_service_ = this->create_service<std_srvs::srv::Trigger>(
        "start_capture",
        std::bind(&FrameSyncNode::startCaptureService, this,
                  std::placeholders::_1, std::placeholders::_2));
    // 使用ApproximateTime策略
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image,
        sensor_msgs::msg::Image, sensor_msgs::msg::JointState,
        sensor_msgs::msg::JointState, geometry_msgs::msg::PoseStamped,
        geometry_msgs::msg::PoseStamped,
        frame_sync_msgs::msg::StampedFloat64MultiArray,
        frame_sync_msgs::msg::StampedFloat64MultiArray>
        ApproxSyncPolicy;
    sync_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
        ApproxSyncPolicy(10), *image_subs_[0], *image_subs_[1], *image_subs_[2],
        *arm_subs_[0], *arm_subs_[1], *pose_subs_[0], *pose_subs_[1],
        *gripper_state_subs_[0], *gripper_state_subs_[1]);

    sync_->registerCallback(std::bind(
        &FrameSyncNode::callback, this, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
        std::placeholders::_5, std::placeholders::_6, std::placeholders::_7,
        std::placeholders::_8, std::placeholders::_9));
    ui().log("Arm and camera sync node initialized");
  }
  ~FrameSyncNode() {
    ui().log("Frame Sync Node is shutting down.");
    // Cleanup code here
  }

 private:
  void declare_parameters() {
    this->declare_parameter<std::vector<std::string>>(
        "camera_topic_names",
        {"/left_camera/color/image_raw", "/right_camera/color/image_raw",
         "/top_camera/color/image_raw"});
    this->declare_parameter<std::vector<std::string>>(
        "arm_joints_topic_names",
        {"/left_arm/joint_states", "/right_arm/joint_states"});
    this->declare_parameter<std::vector<std::string>>(
        "tcp_topic_names", {"/left_arm/tcp_pose", "/right_arm/tcp_pose"});
    this->declare_parameter<std::vector<std::string>>(
        "gripper_topic_names",
        {"/left_arm/gripper_state", "/right_arm/gripper_state"});
    this->declare_parameter<bool>("enable_limit_numbers_of_frames_to_save",
                                  true);
    this->declare_parameter<int>("limit_numbers_of_frames_to_save", 240);
    this->declare_parameter<bool>("enable_save_dataset", true);
    // 添加夹爪触发相关参数
    this->declare_parameter<double>("gripper_trigger_timeout_sec",
                                    5.0);  // 夹爪序列检测超时时间
    this->declare_parameter<double>("gripper_trigger_countdown_sec",
                                    3.0);  // 夹爪触发后倒计时秒数
    this->declare_parameter<bool>("enable_continuous_collection",
                                  true);  // 是否开启连续采集模式
    this->declare_parameter<std::string>("dataset_save_path",
                                         "./data");  // dataset保存路径
  }

  // 倒计时定时器回调函数
  void countdownTimerCallback() {
    if (collection_state_ != CollectionState::COUNTDOWN_ACTIVE) {
      return;
    }

    auto current_time = this->get_clock()->now();
    double elapsed = (current_time - countdown_start_time_).seconds();
    double remaining = gripper_trigger_countdown_sec_ - elapsed;

    if (remaining <= 0) {
      // 倒计时结束，开始数据采集
      collection_state_ = CollectionState::COLLECTING_DATA;
      ui().getStatus().state = CollectionState::COLLECTING_DATA;
      started_ = true;
      countdown_timer_->cancel();

      // 只有在非第一次采集时才重置DataWriter状态
      if (!is_first_collection_) {
      } else {
        is_first_collection_ = false;  // 标记第一次采集已经开始
      }

      ui().log("\033[1;32m倒计时结束！数据采集开始...\033[0m");
    } else {
      // 显示倒计时
      int remaining_int = static_cast<int>(std::ceil(remaining));
      ui().log("\033[1;33m数据采集倒计时: %d 秒\033[0m\r", remaining_int);
    }
  }

  // 手动服务回调函数 - 支持手动启动数据采集
  void startCaptureService(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if (collection_state_ != CollectionState::WAITING_FOR_TRIGGER) {
      res->success = false;
      res->message = "Data collection is not in waiting state!";
      return;
    }

    // 手动服务调用直接开始采集，不使用倒计时
    collection_state_ = CollectionState::COLLECTING_DATA;
    ui().getStatus().state = CollectionState::COLLECTING_DATA;
    started_ = true;

    // 只有在非第一次采集时才重置DataWriter状态
    if (!is_first_collection_) {
    } else {
      is_first_collection_ = false;  // 标记第一次采集已经开始
    }

    res->success = true;
    res->message = "Manual data collection started!";
    ui().log("数据采集已通过手动服务调用启动");
  }

  // 夹爪触发状态枚举
  enum class GripperTriggerState {
    WAITING_FIRST_CLOSE,   // 等待第一次闭合 (<5)
    WAITING_FIRST_OPEN,    // 等待第一次开启 (>20)
    WAITING_SECOND_CLOSE,  // 等待第二次闭合 (<5)
    WAITING_SECOND_OPEN,   // 等待第二次开启 (>20)
    TRIGGERED              // 已触发数据采集
  };

  // 检查夹爪状态并处理触发逻辑 - 仅在等待触发状态时生效
  void checkGripperTrigger(
      const frame_sync_msgs::msg::StampedFloat64MultiArray::ConstSharedPtr
          &gripper_state) {
    if (collection_state_ != CollectionState::WAITING_FOR_TRIGGER ||
        gripper_state->data.empty()) {
      return;  // 如果不在等待触发状态或夹爪数据为空，则不处理
    }

    double gripper_value = gripper_state->data[0];  // 获取第一个数据值
    auto current_time = this->get_clock()->now();

    // 检查超时，如果超时则重置状态
    if ((current_time - gripper_sequence_start_time_).seconds() >
        gripper_trigger_timeout_sec_) {
      gripper_trigger_state_ = GripperTriggerState::WAITING_FIRST_CLOSE;
      gripper_sequence_start_time_ = current_time;
    }

    switch (gripper_trigger_state_) {
      case GripperTriggerState::WAITING_FIRST_CLOSE:
        if (gripper_value < 0.5) {
          gripper_trigger_state_ = GripperTriggerState::WAITING_FIRST_OPEN;
          ui().log("检测到第一次夹爪闭合 (%.2f)", gripper_value);
        }
        break;

      case GripperTriggerState::WAITING_FIRST_OPEN:
        if (gripper_value > 0.5) {
          gripper_trigger_state_ = GripperTriggerState::WAITING_SECOND_CLOSE;
          ui().log("检测到第一次夹爪开启 (%.2f)", gripper_value);
        }
        break;

      case GripperTriggerState::WAITING_SECOND_CLOSE:
        if (gripper_value < 0.5) {
          gripper_trigger_state_ = GripperTriggerState::WAITING_SECOND_OPEN;
          ui().log("检测到第二次夹爪闭合 (%.2f)", gripper_value);
        }
        break;

      case GripperTriggerState::WAITING_SECOND_OPEN:
        if (gripper_value > 0.5) {
          gripper_trigger_state_ = GripperTriggerState::TRIGGERED;
          collection_state_ = CollectionState::COUNTDOWN_ACTIVE;
          ui().getStatus().state = CollectionState::COUNTDOWN_ACTIVE;
          countdown_start_time_ = this->get_clock()->now();
          countdown_timer_->reset();  // 重新启动倒计时定时器
          ui().log("检测到完整夹爪序列！准备开始数据采集... (%.2f)",
                   gripper_value);
          ui().log("夹爪序列：闭合->开启->闭合->开启，倒计时开始！\n");
        }
        break;

      case GripperTriggerState::TRIGGERED:
        // 已经触发，不需要额外处理
        break;
    }
  }

  void callback(
      const sensor_msgs::msg::Image::ConstSharedPtr &image1,
      const sensor_msgs::msg::Image::ConstSharedPtr &image2,
      const sensor_msgs::msg::Image::ConstSharedPtr &image3,
      const sensor_msgs::msg::JointState::ConstSharedPtr &arm1_state,
      const sensor_msgs::msg::JointState::ConstSharedPtr &arm2_state,
      const geometry_msgs::msg::PoseStamped::ConstSharedPtr &tcp1,
      const geometry_msgs::msg::PoseStamped::ConstSharedPtr &tcp2,
      const frame_sync_msgs::msg::StampedFloat64MultiArray::ConstSharedPtr
          &gripper1_state,
      const frame_sync_msgs::msg::StampedFloat64MultiArray::ConstSharedPtr
          &gripper2_state) {
    // 打印时间戳以验证同步
    double time1 =
        image1->header.stamp.sec + image1->header.stamp.nanosec * 1e-9;
    double time2 =
        image2->header.stamp.sec + image2->header.stamp.nanosec * 1e-9;
    double time3 =
        image3->header.stamp.sec + image3->header.stamp.nanosec * 1e-9;
    double time_arm1 =
        arm1_state->header.stamp.sec + arm1_state->header.stamp.nanosec * 1e-9;
    double time_arm2 =
        arm2_state->header.stamp.sec + arm2_state->header.stamp.nanosec * 1e-9;
    double time_tcp1 =
        tcp1->header.stamp.sec + tcp1->header.stamp.nanosec * 1e-9;
    double time_tcp2 =
        tcp2->header.stamp.sec + tcp2->header.stamp.nanosec * 1e-9;
    double time_gripper1 = gripper1_state->header.stamp.sec +
                           gripper1_state->header.stamp.nanosec * 1e-9;
    double time_gripper2 = gripper2_state->header.stamp.sec +
                           gripper2_state->header.stamp.nanosec * 1e-9;
    double avg_time = (time1 + time2 + time3 + time_arm1 + time_arm2 +
                       time_tcp1 + time_tcp2 + time_gripper1 + time_gripper2) /
                      9.0;
    // 检查夹爪触发逻辑 - 仅在等待触发状态时进行检查
    checkGripperTrigger(gripper1_state);
    if (isFrist) {
      ui().log("\033[1;32m提醒:支持两种数据采集启动方式\033[0m\n");
      ui().log("\033[1;36m方式1 - 手动服务调用(立即开始):\033[0m\n");
      ui().log(
          "ros2 service call /start_capture std_srvs/srv/Trigger \"{}\"\n");
      ui().log(
          "\033[1;36m方式2 - 夹爪序列自动触发(倒计时 %.2f 秒后开始):\033[0m\n",
          gripper_trigger_countdown_sec_);
      ui().log(
          "\033[1;33m夹爪序列：闭合(<5) -> 开启(>20) -> 闭合(<5) -> "
          "开启(>20)\033[0m\n");
      if (enable_continuous_collection_) {
        ui().log(
            "\033[1;35m连续采集模式：每轮采集 %d 帧后自动重新开始\033[0m\n",
            limit_numbers_of_frames_to_save_);
      } else {
        ui().log("\033[1;35m单次采集模式：采集 %d 帧后自动关闭\033[0m\n",
                 limit_numbers_of_frames_to_save_);
      }
      ui().log("\033[1;32m停止数据采集:按 Ctrl+C 结束当前控制台\033[0m\n");
      isFrist = false;
    }

    // 只有在COLLECTING_DATA状态时才进行数据处理
    if (collection_state_ != CollectionState::COLLECTING_DATA) return;

    // 添加：记录采集开始时间（仅在第一帧时记录）
    if (!has_collection_started_) {
      collection_start_time_ = this->get_clock()->now();
      has_collection_started_ = true;
      ui().log("\033[1;32m数据采集开始，记录开始时间\033[0m");
    }

    if (!started_ || !enable_save_dataset_) return;
    auto collected_data = std::make_shared<CollectionDataType>();
    collected_data->left_image = image1;
    collected_data->right_image = image2;
    collected_data->top_image = image3;
    collected_data->left_joint_states = arm1_state;
    collected_data->right_joint_states = arm2_state;
    collected_data->tcp1 = tcp1;
    collected_data->tcp2 = tcp2;
    collected_data->gripper1_state = gripper1_state;
    collected_data->gripper2_state = gripper2_state;
    collected_data->stamp =
        std::make_shared<const rclcpp::Time>(this->get_clock()->now());
    collected_data->id = number_of_frames_processed_;

    data_collector_->collect(collected_data);
    number_of_frames_processed_++;
    if (enable_limit_numbers_of_frames_to_save_) {
      if (number_of_frames_processed_ >= limit_numbers_of_frames_to_save_) {
        // 添加：计算并传递时间间隔
        auto collection_end_time = this->get_clock()->now();
        double duration_seconds =
            (collection_end_time - collection_start_time_).seconds();

        ui().log("达到帧数限制 (%d 帧)，本轮采集完成",
                 limit_numbers_of_frames_to_save_);
        ui().log("\033[1;36m采集耗时: %.2f 秒\033[0m", duration_seconds);

        // 结束当前session采集，并且使得下一组session就绪
        data_collector_->next_session(duration_seconds);

        // 重置采集状态，准备下一轮
        collection_state_ = CollectionState::COLLECTION_COMPLETE;
        ui().getStatus().state = CollectionState::COLLECTION_COMPLETE;
        started_ = false;
        has_collection_started_ = false;  // 重置时间记录标志

        if (enable_continuous_collection_) {
          // 连续采集模式：重置状态，等待下一次触发
          resetForNextCollection();
          ui().log("\033[1;36m连续采集模式：准备下一轮数据采集...\033[0m");
        } else {
          // 单次采集模式：关闭节点
          ui().log("单次采集模式：采集完成，节点将关闭...");
          rclcpp::shutdown();
        }
        return;
      }
    }
  }

  // 重置状态，准备下一轮采集
  void resetForNextCollection() {
    collection_state_ = CollectionState::WAITING_FOR_TRIGGER;
    ui().getStatus().state = CollectionState::WAITING_FOR_TRIGGER;
    gripper_trigger_state_ = GripperTriggerState::WAITING_FIRST_CLOSE;
    gripper_sequence_start_time_ = this->get_clock()->now();
    number_of_frames_processed_ = 0;
    countdown_timer_->cancel();
    has_collection_started_ = false;  // 重置时间记录标志

    // 注意：不在这里重置DataWriter，而是等到下一次真正开始采集时再重置
    // 这样避免了在数据还没完全保存时就清空数组的问题

    // 显示等待提示
    if (isFrist) {
      isFrist = false;  // 防止重复显示
    }
    ui().log("\033[1;32m等待下一次触发...\033[0m\n");
  }
  void uiThread() {}
  std::vector<
      std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>>
      image_subs_;
  std::vector<std::shared_ptr<
      message_filters::Subscriber<sensor_msgs::msg::JointState>>>
      arm_subs_;
  std::vector<std::shared_ptr<
      message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>>
      pose_subs_;
  std::vector<std::shared_ptr<message_filters::Subscriber<
      frame_sync_msgs::msg::StampedFloat64MultiArray>>>
      gripper_state_subs_;
  std::shared_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<
          sensor_msgs::msg::Image, sensor_msgs::msg::Image,
          sensor_msgs::msg::Image, sensor_msgs::msg::JointState,
          sensor_msgs::msg::JointState, geometry_msgs::msg::PoseStamped,
          geometry_msgs::msg::PoseStamped,
          frame_sync_msgs::msg::StampedFloat64MultiArray,
          frame_sync_msgs::msg::StampedFloat64MultiArray>>>
      sync_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>
      debug_image_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr>
      debug_arm_subs_;

  std::shared_ptr<DataCollecter> data_collector_;
  rclcpp::Publisher<frame_sync_msgs::msg::SyncedData>::SharedPtr
      SyncedData_pub_;
  bool isFrist = true;
  bool is_first_collection_ = true;  // 标记是否为第一次采集
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
      start_service_;  // 手动服务调用支持
  bool started_ = false;
  bool enable_limit_numbers_of_frames_to_save_;
  int limit_numbers_of_frames_to_save_;
  int number_of_frames_processed_ = 0;
  bool enable_save_dataset_;

  // 添加夹爪触发相关成员变量
  GripperTriggerState gripper_trigger_state_;
  rclcpp::Time gripper_sequence_start_time_;
  double gripper_trigger_timeout_sec_;
  double gripper_trigger_countdown_sec_;  // 倒计时时长
  bool enable_continuous_collection_;     // 是否开启连续采集

  // 添加数据采集状态管理相关成员变量
  CollectionState collection_state_;
  rclcpp::Time countdown_start_time_;
  rclcpp::TimerBase::SharedPtr countdown_timer_;

  // 添加时间记录相关成员变量
  rclcpp::Time collection_start_time_;   // 采集开始时间
  bool has_collection_started_ = false;  // 是否已开始采集标志
};

int main(int argc, char **argv) {
  setvbuf(stdout, NULL, _IONBF, 0);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrameSyncNode>();
  ui().start();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
