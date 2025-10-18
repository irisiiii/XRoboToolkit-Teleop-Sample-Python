#include "frame_sync/DataCollecter.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sys/stat.h>

#include <filesystem>
#include <fstream>

#include "frame_sync/ConsoleUI.hpp"
int DataCollecter::process(
    std::shared_ptr<const CollectionDataType> collected_data) {
  sendToDataWriter(*(collected_data->left_image),
                   base_dir_ + "/" + getCurrentSessionName() + "/left/" +
                       std::to_string(collected_data->id) + ".png");
  sendToDataWriter(*(collected_data->right_image),
                   base_dir_ + "/" + getCurrentSessionName() + "/right/" +
                       std::to_string(collected_data->id) + ".png");
  sendToDataWriter(*(collected_data->top_image),
                   base_dir_ + "/" + getCurrentSessionName() + "/top/" +
                       std::to_string(collected_data->id) + ".png");
  return 0;
}
void DataCollecter::collectLoop() {
  while (running_) {
    std::unique_lock<std::mutex> lock(buffer_queue_mutex_);
    cv_.wait(lock, [this] { return !buffer_queue_.empty() || !running_; });
    if (!running_ && queue_.empty() && buffer_queue_.empty()) {
      return;
    }
    while (!buffer_queue_.empty()) {
      queue_.push(buffer_queue_.front());
      buffer_queue_.pop();
    }
    lock.unlock();
    while (!queue_.empty()) {
      auto now = queue_.front();
      process(now);
      queue_.pop();
    }
  }
}
int DataCollecter::start() {
  dataWriter_.start();
  if (running_) {
    return -1;  // 已经在运行
  }
  running_ = true;
  collecter_thread_ = std::thread(&DataCollecter::collectLoop, this);
  return 0;
}
int DataCollecter::stop() {
  dataWriter_.stop();
  if (!running_) {
    return -1;  // 已经停止
  }
  running_ = false;
  cv_.notify_all();  // 唤醒 writeLoop 线程以处理剩余数据
  if (collecter_thread_.joinable()) {
    collecter_thread_.join();
  }
  return 0;
}
int DataCollecter::collect(std::shared_ptr<CollectionDataType> collected_data) {
  std::unique_lock<std::mutex> lock(buffer_queue_mutex_);
  auto const_collected_data =
      std::const_pointer_cast<const CollectionDataType>(collected_data);
  buffer_queue_.push(const_collected_data);
  cv_.notify_one();
  lock.unlock();
  nlohmann::json data;
  data["id"] = collected_data->id;
  data["left_arm_tcp"] = {collected_data->tcp1->pose.position.x,
                          collected_data->tcp1->pose.position.y,
                          collected_data->tcp1->pose.position.z,
                          collected_data->tcp1->pose.orientation.x,
                          collected_data->tcp1->pose.orientation.y,
                          collected_data->tcp1->pose.orientation.z,
                          collected_data->tcp1->pose.orientation.w};
  data["right_arm_tcp"] = {collected_data->tcp2->pose.position.x,
                           collected_data->tcp2->pose.position.y,
                           collected_data->tcp2->pose.position.z,
                           collected_data->tcp2->pose.orientation.x,
                           collected_data->tcp2->pose.orientation.y,
                           collected_data->tcp2->pose.orientation.z,
                           collected_data->tcp2->pose.orientation.w};
  data["left_arm_joint"] = collected_data->left_joint_states->position;
  data["right_arm_joint"] = collected_data->right_joint_states->position;
  data["left_arm_gripper_state"] = collected_data->gripper1_state->data;
  data["right_arm_gripper_state"] = collected_data->gripper2_state->data;
  data["stamp"] = collected_data->stamp->seconds() +
                  collected_data->stamp->nanoseconds() * 1e-9;
  json_array_.push_back(data);
  current_smaple_id_++;
  ui().getStatus().session_collected_count = current_smaple_id_;
  return 0;
}
int DataCollecter::collect(
    std::shared_ptr<frame_sync_msgs::msg::SyncedData> msg) {
  nlohmann::json data;
  return 0;
}
int DataCollecter::prepare() {
  int max_id = -1;
  // Check if base directory is writable
  if (std::filesystem::exists(base_dir_)) {
    for (const auto &entry : std::filesystem::directory_iterator(base_dir_)) {
      if (entry.is_directory()) {
        std::string name = entry.path().filename().string();
        try {
          int id = std::stoi(name);
          if (id > max_id) max_id = id;
        } catch (...) {
        }
      }
    }
  }
  current_session_id_ = max_id + 1;
  ui().getStatus().session_id = current_session_id_;
  return 0;
}
int DataCollecter::next_session(double collection_duration_seconds) {
  std::string json_file_path =
      base_dir_ + "/" + getCurrentSessionName() + "/obs.json";
  // 创建根JSON对象，包含元数据和数据数组
  std::shared_ptr<nlohmann::json> root_json =
      std::make_shared<nlohmann::json>();
  // 添加元数据
  (*root_json)["metadata"] = {{"total_frames", current_smaple_id_},
                              {"format_version", "2.0"},
                              {"timestamp", std::time(nullptr)}};

  // 添加采集时间间隔（如果有效）
  if (collection_duration_seconds > 0) {
    (*root_json)["metadata"]["collection_duration_seconds"] =
        std::round(collection_duration_seconds * 100.0) /
        100.0;  // 保留两位小数
  }

  // 添加数据数组
  (*root_json)["data"] = json_array_;
  dataWriter_.addTask(std::make_shared<WriteTask>(WriteTask{
      .data = root_json, .type = WriteType::JSON, .path = json_file_path}));

  if (collection_duration_seconds > 0) {
    ui().log("\033[1;32m共处理 %d 帧数据，采集耗时: %.2f 秒\033[0m",
             current_smaple_id_, collection_duration_seconds);
  } else {
    ui().log("\033[1;32m共处理 %d 帧数据\033[0m", current_smaple_id_);
  }
  json_array_.clear();

  current_smaple_id_ = 0;
  current_session_id_++;
  ui().getStatus().session_id = current_session_id_;
  ui().getStatus().session_collected_count = current_smaple_id_;

  return 0;
}

void DataCollecter::waitForQueueEmpty() {
  // 等待所有队列中的数据被处理完成
  // 使用简单的轮询方式，避免条件变量死锁
  int max_wait_cycles = 100;  // 最多等待5秒 (100 * 50ms)
  int wait_cycles = 0;
  
  while (wait_cycles < max_wait_cycles) {
    std::unique_lock<std::mutex> lock(buffer_queue_mutex_);
    bool queues_empty = buffer_queue_.empty() && queue_.empty();
    lock.unlock();
    
    if (queues_empty) {
      break;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    wait_cycles++;
  }
  
  // 额外等待一小段时间，确保所有异步写入完成
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

int DataCollecter::sendToDataWriter(const sensor_msgs::msg::Image &msg,
                                    const std::string &filename) {
  try {
    // 验证图像尺寸
    if (msg.width <= 0 || msg.height <= 0) {
      std::cout << "无效的图像尺寸: width=" << msg.width
                << ", height=" << msg.height << std::endl;
      return -1;
    }
    // 验证数据长度是否与尺寸和编码匹配
    if (msg.data.empty()) {
      std::cout << "图像数据为空: " << filename.c_str() << std::endl;
      return -1;
    }
    // 检查编码格式
    if (msg.encoding != sensor_msgs::image_encodings::BGR8 &&
        msg.encoding != sensor_msgs::image_encodings::RGB8) {
      std::cout << "不支持的图像编码: " << msg.encoding
                << ", 期望: " << sensor_msgs::image_encodings::BGR8
                << std::endl;
      return -1;
    }
    // 验证数据长度 (BGR8: 3 字节每像素)
    size_t expected_size = msg.width * msg.height * 3;
    if (msg.data.size() != expected_size) {
      std::cout << "图像数据长度不匹配: 期望=" << expected_size
                << ", 实际=" << msg.data.size() << std::endl;
      return -1;
    }

    // 转换为 OpenCV 格式
    cv_bridge::CvImagePtr cv_ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    std::shared_ptr<cv::Mat> saved_image = std::make_shared<cv::Mat>();
    cv::cvtColor(cv_ptr->image, *saved_image, cv::COLOR_RGB2BGR);

    // 验证转换后的图像
    if (saved_image->empty()) {
      std::cout << "转换后的 OpenCV 图像为空" << std::endl;
      return -1;
    }
    dataWriter_.addTask(std::make_shared<WriteTask>(WriteTask{
        .data = saved_image, .type = WriteType::IMAGE, .path = filename}));
    // 保存图像
    // cv::imwrite(filename, saved_image);
  } catch (cv_bridge::Exception &e) {
    std::cout << "cv_bridge 异常: " << e.what() << std::endl;
  } catch (cv::Exception &e) {
    std::cout << "OpenCV 异常: " << e.what() << std::endl;
  } catch (std::exception &e) {
    std::cout << "标准异常: " << e.what() << std::endl;
  }
  return 0;
}
DataCollecter::~DataCollecter() { stop(); }
DataCollecter::DataCollecter(std::string base_dir) {
  base_dir_ = base_dir;
  ui().log("数据保存路径: %s\n", base_dir_.c_str());
}