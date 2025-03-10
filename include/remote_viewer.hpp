#pragma once

#include "callback_worker.hpp"
#include <glim/util/extension_module.hpp>

#include <Eigen/Core>
using namespace glim;
using namespace std;

class RemoteViewer: public CallbackWorker {
public:
  RemoteViewer();
  ~RemoteViewer();

private:
  void set_callbacks();
  virtual void run() override;

  bool drawable_filter(const std::string& name);
  void drawable_selection();
  void drawable_session_list();

  // Private variables
  bool enable_partial_rendering;
  int partial_rendering_budget;
  bool track;

  Eigen::Vector2f z_range;
  Eigen::Vector2f auto_z_range;
  int current_color_mode;
};
