#pragma once

#include <atomic>
#include <glim/util/extension_module.hpp>
#include <mutex>
#include <functional>
#include <thread>

#include <Eigen/Core>
using namespace glim;
using namespace std;

class RemoteViewer{
public:
  RemoteViewer();
  ~RemoteViewer();

  void invoke(const std::function<void()> &task);

private:
  void set_callbacks();
  void viewer_loop();

  bool drawable_filter(const std::string& name);
  void drawable_selection();

  // Private variables

  std::atomic_bool request_to_terminate;
  std::atomic_bool kill_switch;
  std::thread thread;

  std::mutex invoke_queue_mutex;
  std::vector<std::function<void()>> invoke_queue;

  bool enable_partial_rendering;
  int partial_rendering_budget;

  Eigen::Vector2f z_range;
  Eigen::Vector2f auto_z_range;
  int current_color_mode;
};
