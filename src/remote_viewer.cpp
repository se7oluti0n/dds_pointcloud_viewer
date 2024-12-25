#include "remote_viewer.hpp"

#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>


#include <glk/colormap.hpp>
#include <glk/texture.hpp>
#include <glk/texture_opencv.hpp>
#include <glk/thin_lines.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/indexed_pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/spdlog_sink.hpp>
#include <guik/viewer/light_viewer.hpp>

RemoteViewer::RemoteViewer() {

}

RemoteViewer::~RemoteViewer() {
  kill_switch = true;
  if (thread.joinable()) {
    thread.join();
  }
}

bool RemoteViewer::ok() const {
  return !request_to_terminate;
}

void RemoteViewer::invoke(const std::function<void()>& task) {
  if (kill_switch) {
    return;
  }
  std::lock_guard<std::mutex> lock(invoke_queue_mutex);
  invoke_queue.push_back(task);
}


void RemoteViewer::viewer_loop() {
  glim::Config config(glim::GlobalConfig::get_config_path("config_viewer"));

  auto viewer = guik::LightViewer::instance(Eigen::Vector2i(config.param("standard_viewer", "viewer_width", 2560), config.param("standard_viewer", "viewer_height", 1440)));
  viewer->enable_vsync();
  viewer->shader_setting().add("z_range", z_range);

  if (enable_partial_rendering) {
    viewer->enable_partial_rendering(1e-1);
    viewer->shader_setting().add("dynamic_object", 1);
  }

  auto submap_viewer = viewer->sub_viewer("submap");
  submap_viewer->set_pos(Eigen::Vector2i(100, 800));
  submap_viewer->set_draw_xy_grid(false);
  submap_viewer->use_topdown_camera_control(80.0);

  viewer->register_drawable_filter("selection", [this](const std::string& name) { return drawable_filter(name); });
  viewer->register_ui_callback("selection", [this] { drawable_selection(); });
  viewer->register_ui_callback("logging", guik::create_logger_ui(glim::get_ringbuffer_sink(), 0.5));

  while (!kill_switch) {
    if (!viewer->spin_once()) {
      request_to_terminate = true;
    }

    std::vector<std::function<void()>> tasks;
    {
      std::lock_guard<std::mutex> lock(invoke_queue_mutex);
      tasks.swap(invoke_queue);
    }

    for (const auto& task : tasks) {
      task();
    }
  }

  guik::LightViewer::destroy();
}
