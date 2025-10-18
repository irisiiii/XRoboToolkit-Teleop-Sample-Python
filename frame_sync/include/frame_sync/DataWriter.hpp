#pragma once
#include <condition_variable>
#include <mutex>
#include <nlohmann/json.hpp>
#ifdef OK
#undef OK
#endif
#include <atomic>
#include <opencv2/opencv.hpp>
#include <queue>
#include <thread>
#include <variant>

#include "frame_sync_msgs/msg/synced_data.hpp"

enum class WriteType { JSON, IMAGE };

struct WriteTask {
  std::variant<std::shared_ptr<nlohmann::json>, std::shared_ptr<cv::Mat> >
      data;          // 二者之一：JSON 或 cv::Mat
  WriteType type;    // 类型标识："json" 或 "image"
  std::string path;  // 保存路径
};
class DataWriter {
 public:
  DataWriter();
  ~DataWriter();
  int addTask(std::shared_ptr<WriteTask> task);
  int start();
  int stop();

 private:
  void writeLoop();
  bool ensureDirectoryExists(const std::string &filePath);
  int processTask(const WriteTask &task);
  std::atomic<bool> running_{false};
  std::condition_variable cv_;
  std::thread writer_thread_;
  std::mutex buffer_task_queue_mutex_;
  std::queue<WriteTask> task_queue_, buffer_task_queue_;  // 队列：WriteTask
};
