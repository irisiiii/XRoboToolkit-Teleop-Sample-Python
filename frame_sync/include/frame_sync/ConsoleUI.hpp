#pragma once
#include <ncurses.h>

#include <atomic>
#include <mutex>
#include <queue>
#include <thread>
// enum class CollectionState { READY, COLLECTING, PAUSED, STOPPED, ERROR };
// 数据采集状态枚举 - 管理整个采集生命周期
enum class CollectionState {
  WAITING_FOR_TRIGGER,     // 等待触发（监听夹爪或服务调用）
  WAITING_FOR_DATA_READY,  // 等待数据准备就绪
  COUNTDOWN_ACTIVE,        // 倒计时进行中
  COLLECTING_DATA,         // 数据采集进行中
  COLLECTION_COMPLETE,     // 采集完成，等待重置
  ERROR,                   // 错误
  UNKNOWN,                 // 未知
  PAUSED,                  // 暂停
  STOPPED                  // 已停止
};
struct AcquisitionStatus {
  std::atomic<int> session_id;
  std::atomic<CollectionState> state;  // 就绪、采集中、暂停...
  std::atomic<int> saved_count = 0;
  std::atomic<int> buffered_count = 0;

  std::atomic<int> session_collected_count = 0;  // 当前session已采集的总帧数
  std::atomic<int> session_target_count = 0;  // 当前session目标总帧数
  std::atomic<bool> exit_flag = false;
};

class ConsoleUI {
 public:
  // 获取全局实例
  static ConsoleUI &instance() {
    static ConsoleUI inst;
    return inst;
  }
  ~ConsoleUI();
  ConsoleUI(const ConsoleUI &) = delete;
  ConsoleUI &operator=(const ConsoleUI &) = delete;
  ConsoleUI();
  void start();  // 启动 UI 线程
  void stop();   // 停止 UI 线程
  // 在主线程阻塞运行 UI（会调用 ncurses 初始化）
  void runBlocking();

  // 日志接口（非阻塞）：入队，由 UI 线程消费并写文件
  void log(const char *format, ...);

  // 关键日志：同步写文件并同时入队 (可选，用于必须立即落盘的情况)
  void logImmediate(const std::string &msg);
  AcquisitionStatus &getStatus() { return status_; }

 private:
  void drawUI();
  void uiLoop();
  void drawProgressBar(WINDOW *w, int y, int x, int width, long collected,
                       long remaining);
  void drainLogQueue();  // 将队列中的日志移动到 history 并写文件
  void mvwprintw_ansi(WINDOW *win, int y, int x, const std::string &s);
  AcquisitionStatus status_;
  std::thread ui_thread_;
  std::mutex log_mutex_;
  std::queue<std::string> log_queue_;
  std::vector<std::string> log_history_;
  std::atomic<bool> running_{false};
  // ncurses windows pointers (kept across draws)
  WINDOW *win_status_ = nullptr;
  WINDOW *win_log_ = nullptr;
};

inline ConsoleUI &ui() { return ConsoleUI::instance(); }