#pragma once
#include <nlohmann/json.hpp>
#include <rclcpp/time.hpp>

#include "frame_sync/DataWriter.hpp"
#include "frame_sync_msgs/msg/synced_data.hpp"
struct CollectionDataType {
  std::shared_ptr<const sensor_msgs::msg::Image> left_image;
  std::shared_ptr<const sensor_msgs::msg::Image> right_image;
  std::shared_ptr<const sensor_msgs::msg::Image> top_image;
  std::shared_ptr<const sensor_msgs::msg::JointState> left_joint_states;
  std::shared_ptr<const sensor_msgs::msg::JointState> right_joint_states;
  std::shared_ptr<const geometry_msgs::msg::PoseStamped> tcp1;
  std::shared_ptr<const geometry_msgs::msg::PoseStamped> tcp2;
  std::shared_ptr<const frame_sync_msgs::msg::StampedFloat64MultiArray>
      gripper1_state;
  std::shared_ptr<const frame_sync_msgs::msg::StampedFloat64MultiArray>
      gripper2_state;
  std::shared_ptr<const rclcpp::Time> stamp;  // const 版本，避免修改
  int id;
};
class DataCollecter {
 public:
  DataCollecter(std::string base_dir);
  int start();
  int stop();
  ~DataCollecter();
  int collect(std::shared_ptr<frame_sync_msgs::msg::SyncedData> msg);
  int collect(std::shared_ptr<CollectionDataType> collected_data);
  int prepare();
  int next_session(double collection_duration_seconds);
  void collectLoop();
  void waitForQueueEmpty();  // 等待队列清空
  std::string getCurrentSessionName() {
    std::ostringstream oss;
    oss << std::setw(4) << std::setfill('0') << current_session_id_;
    return oss.str();
  }

 private:
  int current_session_id_ = 0;
  int current_smaple_id_ = 0;
  int sendToDataWriter(const sensor_msgs::msg::Image &msg,
                       const std::string &filename);
  int process(std::shared_ptr<const CollectionDataType> collected_data);
  std::string base_dir_;
  DataWriter dataWriter_;
  nlohmann::json json_array_;
  std::thread collecter_thread_;
  std::atomic<bool> running_{false};
  std::condition_variable cv_;
  std::queue<std::shared_ptr<const CollectionDataType> > queue_, buffer_queue_;
  std::mutex buffer_queue_mutex_;
};