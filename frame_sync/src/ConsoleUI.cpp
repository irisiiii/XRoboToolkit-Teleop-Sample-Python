#include "frame_sync/ConsoleUI.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
using namespace std;
using namespace std::chrono_literals;
ConsoleUI::ConsoleUI() { setlocale(LC_CTYPE, ""); }
void ConsoleUI::start() {
  if (running_) return;
  running_ = true;
  ui_thread_ = std::thread(&ConsoleUI::uiLoop, this);
}
void ConsoleUI::stop() {
  if (!running_) return;
  // signal exit
  status_.exit_flag = true;
  running_ = false;
  if (ui_thread_.joinable()) ui_thread_.join();
}
static std::string timestamp_now() {
  auto t = std::time(nullptr);
  char buf[64];
  std::strftime(buf, sizeof(buf), "%F %T", std::localtime(&t));
  return std::string(buf);
}
void ConsoleUI::log(const char *format, ...) {
  va_list args;
  va_start(args, format);
  char buffer[4096];
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  std::string msg(buffer);
  std::ostringstream oss;
  oss << "[" << timestamp_now() << "] " << msg;
  std::lock_guard<std::mutex> lk(log_mutex_);
  log_queue_.push(oss.str());
}
void ConsoleUI::logImmediate(const std::string &msg) {
  std::string s = "[" + timestamp_now() + "] " + msg;
  {
    std::lock_guard<std::mutex> lk(log_mutex_);
    log_queue_.push(s);
  }
}
void ConsoleUI::uiLoop() {
  // ncurses init (this thread)
  initscr();
  cbreak();
  noecho();
  curs_set(0);
  nodelay(stdscr, TRUE);  // non-blocking getch
  keypad(stdscr, TRUE);

  // initial log
  logImmediate("UI 启动 (ConsoleUI) 。按 q 退出 UI。");

  while (!status_.exit_flag) {
    drawUI();

    // handle input simple demo: q to quit UI
    int ch = getch();
    if (ch != ERR) {
      if (ch == 'q' || ch == 'Q') {
        status_.exit_flag = true;
        log("用户按下 q，退出。");
        break;
      } else if (ch == KEY_RESIZE) {
        // will be handled on next draw via wresize
      }
    }

    std::this_thread::sleep_for(16.67ms);  // ~60fps
  }

  // cleanup ncurses
  endwin();
}
void ConsoleUI::drainLogQueue() {
  // move queue into history vector and append to file
  std::vector<std::string> tmp;
  {
    std::lock_guard<std::mutex> lk(log_mutex_);
    while (!log_queue_.empty()) {
      tmp.push_back(log_queue_.front());
      log_queue_.pop();
    }
  }
  if (tmp.empty()) return;

  // ensure log_ofs_ open (only UI thread touches log_ofs_)
  for (const auto &s : tmp) {
    // push to history (for UI)
    log_history_.push_back(s);
  }

  // cap history size
  if (log_history_.size() > 5000) {
    // remove older half
    log_history_.erase(log_history_.begin(), log_history_.begin() + 2000);
  }
}
void ConsoleUI::drawUI() {
  int rows, cols;
  getmaxyx(stdscr, rows, cols);
  int status_h = 4;
  int log_h = rows - status_h;
  if (log_h < 6) log_h = 6;

  // initialize windows once
  if (!win_status_) {
    win_status_ = newwin(status_h, cols, 0, 0);
    win_log_ = newwin(rows - status_h, cols, status_h, 0);
    scrollok(win_log_, TRUE);
    keypad(stdscr, TRUE);
    start_color();
    use_default_colors();
    init_pair(1, COLOR_BLUE, COLOR_BLACK);
    wattron(win_status_, COLOR_PAIR(1));
    init_pair(2, COLOR_CYAN, COLOR_BLACK);
    wattron(win_status_, COLOR_PAIR(2));
    init_pair(3, COLOR_YELLOW, COLOR_BLACK);
    wattron(win_status_, COLOR_PAIR(3) | A_BOLD);
    init_pair(4, COLOR_GREEN, COLOR_BLACK);
    wattron(win_status_, COLOR_PAIR(4));
    init_pair(5, COLOR_YELLOW, COLOR_BLACK);
    wattron(win_status_, COLOR_PAIR(5) | A_BOLD);
    init_pair(6, COLOR_RED, COLOR_BLACK);
    wattron(win_status_, COLOR_PAIR(6) | A_BOLD);
    init_pair(7, COLOR_WHITE, COLOR_BLACK);
    wattron(win_status_, COLOR_PAIR(7));
    init_pair(8, COLOR_MAGENTA, COLOR_BLACK);
    wattron(win_status_, COLOR_PAIR(8));
    init_pair(9, COLOR_BLACK, COLOR_WHITE);
    wattron(win_status_, COLOR_PAIR(9));
  } else {
    // handle resize
    wresize(win_status_, status_h, cols);
    wresize(win_log_, rows - status_h, cols);
    mvwin(win_log_, status_h, 0);
  }

  // Drain logs from queue -> history + file
  drainLogQueue();

  // draw status window
  werase(win_status_);
  box(win_status_, 0, 0);
  wattron(win_status_, COLOR_PAIR(5) | A_BOLD);
  mvwprintw(win_status_, 0, 2, " 实时状态 ");
  wattroff(win_status_, COLOR_PAIR(5) | A_BOLD);

  // copy status snapshot under local variables (minimize concurrency)
  CollectionState sstate;
  int sid;
  int session_collected_count, session_target_total;
  int saved_count, buffered_count;
  {
    sid = status_.session_id;
    sstate = status_.state;
    session_collected_count = status_.session_collected_count;
    session_target_total = status_.session_target_count;
    saved_count = status_.saved_count;
    buffered_count = status_.buffered_count;
  }

  mvwprintw(win_status_, 1, 2, "Session : ");
  wattron(win_status_, A_BOLD);
  std::ostringstream oss;
  oss << std::setw(4) << std::setfill('0') << sid;
  mvwprintw(win_status_, 1, 11, "%s", oss.str().c_str());
  wattroff(win_status_, A_BOLD);

  // 状态颜色
  int state_color = 7;
  std::string stateStr;
  // 假设 sstate 是 CollectionState 类型，state_color 是 int，stateStr 是
  // std::string
  switch (sstate) {
    case CollectionState::WAITING_FOR_TRIGGER:
      state_color = 1;
      stateStr = "等待触发事件";
      break;
    case CollectionState::WAITING_FOR_DATA_READY:
      state_color = 2;
      stateStr = "等待数据就绪";
      break;
    case CollectionState::COUNTDOWN_ACTIVE:
      state_color = 3;
      stateStr = "倒计时执行中";
      break;
    case CollectionState::COLLECTING_DATA:
      state_color = 4;
      stateStr = "数据采集中";
      break;
    case CollectionState::COLLECTION_COMPLETE:
      state_color = 5;
      stateStr = "采集完成待置";
      break;
    case CollectionState::ERROR:
      state_color = 6;
      stateStr = "发生错误";
      break;
    case CollectionState::UNKNOWN:
      state_color = 7;
      stateStr = "状态未知";
      break;
    case CollectionState::PAUSED:
      state_color = 8;
      stateStr = "系统暂停中";
      break;
    case CollectionState::STOPPED:
      state_color = 9;
      stateStr = "已停止";
      break;
    default:
      state_color = 7;  // 默认未知
      stateStr = "状态未知";
      break;
  }

  mvwprintw(win_status_, 1, cols / 3, "状态: ");
  wattron(win_status_, COLOR_PAIR(state_color) | A_BOLD);
  mvwprintw(win_status_, 1, cols / 3 + 6, "%s", stateStr.c_str());
  wattroff(win_status_, COLOR_PAIR(state_color) | A_BOLD);

  // counts
  mvwprintw(win_status_, 2, 2, "​​已保存: %ld", saved_count);
  mvwprintw(win_status_, 2, 20, "待保存: %ld", buffered_count);

  // progress bar with percentage
  int bar_x = 36;
  int bar_w = cols - bar_x - 12;
  if (bar_w < 10) bar_w = 10;
  drawProgressBar(win_status_, 2, bar_x, bar_w, session_collected_count,
                  session_target_total - session_collected_count);

  wrefresh(win_status_);

  // draw log window: show last lines that fit
  werase(win_log_);
  box(win_log_, 0, 0);
  int inner_h = (rows - status_h) - 2;
  int inner_w = cols - 2;
  int start = 0;
  if ((int)log_history_.size() > inner_h) start = log_history_.size() - inner_h;
  int y = 1;
  for (size_t i = start; i < log_history_.size(); ++i, ++y) {
    // ensure we don't overflow width; ncurses handles utf-8 printing
    mvwprintw(win_log_, y, 2, "%s", log_history_[i].c_str());
  }
  wrefresh(win_log_);
}
ConsoleUI::~ConsoleUI() { stop(); }
void ConsoleUI::runBlocking() {
  if (running_) {
    // 如果已经在运行（start 调用过），不要重复执行 blocking 模式
    return;
  }
  running_ = true;
  uiLoop();  // 在当前线程运行（会阻塞直到 UI 退出）
  running_ = false;
}
void ConsoleUI::drawProgressBar(WINDOW *w, int y, int x, int width,
                                long collected, long remaining) {
  long total = collected + remaining;
  int fill = 0;
  if (total > 0) {
    // compute percentage carefully
    double ratio = (double)collected / (double)total;
    fill = (int)(ratio * width + 0.5);
    if (fill < 0) fill = 0;
    if (fill > width) fill = width;
  }
  mvwprintw(w, y, x, "[");
  for (int i = 0; i < width; ++i) {
    if (i < fill)
      waddch(w, '=');
    else
      waddch(w, ' ');
  }
  wprintw(w, "]");
  // percentage
  int percent = 0;
  if (total > 0)
    percent = (int)((double)collected * 100.0 / (double)total + 0.5);
  mvwprintw(w, y, x + width + 3, "%3d%%", percent);
}
