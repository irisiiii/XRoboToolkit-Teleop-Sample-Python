#include "frame_sync/DataWriter.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sys/stat.h>

#include <ctime>  // 用于 std::time
#include <fstream>
#include <iomanip>  // 用于 std::setprecision
#include <iostream>
#ifdef OK
#undef OK
#endif
#include <opencv2/opencv.hpp>
#include <stdexcept>

#include "frame_sync/ConsoleUI.hpp"
namespace fs = std::filesystem;

DataWriter::DataWriter() {}
int DataWriter::start() {
  if (running_) {
    return -1;  // 已经在运行
  }
  running_ = true;
  writer_thread_ = std::thread(&DataWriter::writeLoop, this);
  return 0;
}
int DataWriter::stop() {
  if (!running_) {
    return -1;  // 已经停止
  }
  running_ = false;
  cv_.notify_all();  // 唤醒 writeLoop 线程以处理剩余数据
  if (writer_thread_.joinable()) {
    writer_thread_.join();
  }
  return 0;
}

void DataWriter::writeLoop() {
  while (running_) {
    std::unique_lock<std::mutex> lock(buffer_task_queue_mutex_);

    cv_.wait(lock, [this] { return !buffer_task_queue_.empty() || !running_; });
    if (!running_ && task_queue_.empty() && buffer_task_queue_.empty()) {
      return;
    }
    while (!buffer_task_queue_.empty()) {
      task_queue_.push(buffer_task_queue_.front());
      buffer_task_queue_.pop();
    }
    lock.unlock();
    while (!task_queue_.empty()) {
      auto now = task_queue_.front();

      processTask(now);
      task_queue_.pop();
      ui().getStatus().saved_count++;
      ui().getStatus().buffered_count = buffer_task_queue_.size();
    }
  }
}
bool DataWriter::ensureDirectoryExists(const std::string& filePath) {
  try {
    fs::path p(filePath);
    if (p.has_parent_path()) {
      fs::create_directories(p.parent_path());
      return true;
    }
    return true;  // 无父目录，直接文件
  } catch (const fs::filesystem_error& e) {
    ui().log("Error creating directory for %s: %s", filePath.c_str(), e.what());
    return false;
  }
}
int DataWriter::processTask(const WriteTask& task) {  // 通用：确保目录存在
  if (!ensureDirectoryExists(task.path)) {
    ui().log("Error: Unable to create directory for %s, skipping save",
             task.path.c_str());
    return -1;
  }
  if (task.type == WriteType::IMAGE) {
    auto img_ptr = std::get<std::shared_ptr<cv::Mat>>(task.data);
    if (img_ptr && !img_ptr->empty()) {
      bool success = cv::imwrite(task.path, *img_ptr);
      if (!success) {
        ui().log("Error: Failed to save image to %s", task.path.c_str());
      }
    } else {
      ui().log("Warning: Empty image data, skipping save to %s",
               task.path.c_str());
    }
  } else if (task.type == WriteType::JSON) {
    auto json_ptr = std::get<std::shared_ptr<nlohmann::json>>(task.data);
    if (json_ptr) {
      // 追加模式写入 JSON 文件（根据之前对话调整；这里用 out 覆盖，如需追加改成
      // std::ios::app）
      std::ofstream file(task.path, std::ios::out);  // 或 std::ios::app 以追加
      if (file.is_open()) {
        file << json_ptr->dump() << std::endl;
        file.close();
        ui().log("Success: JSON saved to %s", task.path.c_str());  // 可选日志
      } else {
        ui().log("Error: Unable to open JSON file for writing: %s",
                 task.path.c_str());
      }
    } else {
      ui().log("Error: Null JSON data, skipping save to %s", task.path.c_str());
    }
  }
  return 0;
}
int DataWriter::addTask(std::shared_ptr<WriteTask> task) {
  std::unique_lock<std::mutex> lock(buffer_task_queue_mutex_);
  buffer_task_queue_.push(std::move(*task));
  cv_.notify_one();
  lock.unlock();
  ui().getStatus().buffered_count = buffer_task_queue_.size();
  return 0;
}

DataWriter::~DataWriter() { stop(); }
