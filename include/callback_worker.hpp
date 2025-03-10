#pragma once

#include <functional>
#include <thread>
#include <mutex>
#include <atomic>


class CallbackWorker
{
public:
  CallbackWorker(){}
  ~CallbackWorker() {
    kill_switch = true;
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  virtual void run() = 0;

  inline void start(){
    thread_ = std::thread([this] { run();});
  }

protected:
  inline void invoke(const std::function<void()> &task) {
    if (kill_switch) {
      return;
    }
    std::lock_guard<std::mutex> lock(invoke_queue_mutex);
    invoke_queue.push_back(task);
  }

  // Private variables

  std::atomic_bool request_to_terminate;
  std::atomic_bool kill_switch;
  std::thread thread_;

  std::mutex invoke_queue_mutex;
  std::vector<std::function<void()>> invoke_queue;

};
