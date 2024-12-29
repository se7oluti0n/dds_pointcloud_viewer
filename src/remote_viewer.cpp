#include "remote_viewer.hpp"

#include "callbacks.hpp"

#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>

#include <glk/colormap.hpp>
#include <glk/indexed_pointcloud_buffer.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <glk/texture.hpp>
#include <glk/texture_opencv.hpp>
#include <glk/thin_lines.hpp>
#include <guik/spdlog_sink.hpp>
#include <guik/viewer/light_viewer.hpp>

RemoteViewer::RemoteViewer() {

  kill_switch = false;
  request_to_terminate = false;

  current_color_mode = 0;
  z_range = Eigen::Vector2d(-2.0, 4.0).cast<float>();
  auto_z_range << 0.0f, 0.0f;


  // points_alpha = 1.0;
  // factors_alpha = 1.0;
  //
  // point_size = 1.0;
  // point_size_metric = false;
  // point_shape_circle =  true;


  enable_partial_rendering = false;
  partial_rendering_budget = 1024;
  set_callbacks();
  thread = std::thread([this] { viewer_loop(); });
}

RemoteViewer::~RemoteViewer() {
  kill_switch = true;
  if (thread.joinable()) {
    thread.join();
  }
}

// bool RemoteViewer::ok() const { return !request_to_terminate; }

void RemoteViewer::invoke(const std::function<void()> &task) {
  if (kill_switch) {
    return;
  }
  std::lock_guard<std::mutex> lock(invoke_queue_mutex);
  invoke_queue.push_back(task);
}

void RemoteViewer::set_callbacks() {

  glim::DDSCallbacks::on_raw_pointcloud.add(
      [this](glim::RawPoints::ConstPtr gtsam_pointcloud) {
        invoke([this, gtsam_pointcloud] {
          auto viewer = guik::LightViewer::instance();
          auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(
              gtsam_pointcloud->points);
          Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
          guik::ShaderSetting shader_setting =
              guik::FlatColor(1.0f, 0.5f, 0.0f, 1.0f, pose);
          guik::ShaderSetting shader_setting_rainbow = guik::Rainbow(pose);

          switch (current_color_mode) {
          case 0: // FLAT
            break;
          case 1: // INTENSITY
            if (!gtsam_pointcloud->intensities.empty()) {
              const double max_intensity =
                  *std::max_element(gtsam_pointcloud->intensities.begin(),
                                    gtsam_pointcloud->intensities.end());
              cloud_buffer->add_intensity(glk::COLORMAP::TURBO,
                                          gtsam_pointcloud->intensities,
                                          1.0 / max_intensity);
            }
            shader_setting.add("color_mode", guik::ColorMode::VERTEX_COLOR);
            shader_setting_rainbow.add("color_mode",
                                       guik::ColorMode::VERTEX_COLOR);
            break;
          }

          viewer->update_drawable("current_frame", cloud_buffer,
                                  shader_setting.add("point_scale", 2.0f));
          viewer->update_drawable(
              "current_coord", glk::Primitives::coordinate_system(),
              guik::VertexColor(pose * Eigen::UniformScaling<float>(1.5f)));
        });
      });
}

bool RemoteViewer::drawable_filter(const std::string &name) { return true; }

void RemoteViewer::drawable_selection() {
  auto viewer = guik::LightViewer::instance();

  ImGui::SetWindowPos("images", {1800, 60}, ImGuiCond_FirstUseEver);
  ImGui::SetWindowPos("logging", {1800, 950}, ImGuiCond_FirstUseEver);

  ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.5));
  ImGui::Begin("selection", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
  ImGui::PopStyleColor();

  //if (ImGui::Checkbox("track", &track)) {
  //  if (track) {
  //    guik::LightViewer::instance()->reset_center();
  //  }
  //}
  //ImGui::SameLine();
  //bool show_current = show_current_coord || show_current_points;
  //if (ImGui::Checkbox("current", &show_current)) {
  //  show_current_coord = show_current_points = show_current;
  //}
  //ImGui::SameLine();
  //ImGui::Checkbox("coord", &show_current_coord);
  //ImGui::SameLine();
  //ImGui::Checkbox("points", &show_current_points);

  std::vector<const char *> current_color_modes = {"FLAT", "INTENSITY",
                                                   "NORMAL"};
  ImGui::SetNextItemWidth(92);
  ImGui::Combo("color_mode", &current_color_mode, current_color_modes.data(),
               current_color_modes.size());

  ImGui::SameLine();
  if (ImGui::Button("Log")) {
    viewer->register_ui_callback(
        "logging", guik::create_logger_ui(glim::get_ringbuffer_sink(), 0.5));
  }

  // ImGui::Separator();
  // bool show_odometry =
  //     show_odometry_scans || show_odometry_keyframes || show_odometry_factors;
  // if (ImGui::Checkbox("odometry", &show_odometry)) {
  //   show_odometry_scans = show_odometry_keyframes = show_odometry;
  //   show_odometry_factors &= show_odometry;
  // }
  //
  // ImGui::SameLine();
  // if (ImGui::Button("Status")) {
  //   show_odometry_status = true;
  // }
  //
  // ImGui::Checkbox("scans##odom", &show_odometry_scans);
  // ImGui::SameLine();
  // ImGui::Checkbox("keyframes##odom", &show_odometry_keyframes);
  // ImGui::SameLine();
  // ImGui::Checkbox("factors##odom", &show_odometry_factors);
  //
  // ImGui::Separator();
  // bool show_mapping = show_submaps || show_factors;
  // if (ImGui::Checkbox("mapping", &show_mapping)) {
  //   show_submaps = show_factors = show_mapping;
  // }
  //
  // ImGui::Checkbox("submaps", &show_submaps);
  // ImGui::SameLine();
  // ImGui::Checkbox("factors", &show_factors);
  //
  // ImGui::Separator();
  // if (ImGui::DragFloatRange2("z_range", &z_range[0], &z_range[1], 0.1f, -100.0f,
  //                            100.0f)) {
  //   viewer->shader_setting().add<Eigen::Vector2f>("z_range",
  //                                                 auto_z_range + z_range);
  // }
  //
  // ImGui::End();
  //
  // if (show_odometry_status) {
  //   ImGui::Begin("odometry status", &show_odometry_status,
  //                ImGuiWindowFlags_AlwaysAutoResize);
  //   ImGui::Text("frame ID:%d", last_id);
  //   ImGui::Text("points:%d", last_num_points);
  //   ImGui::Text("stamp:%.3f ~ %.3f", last_point_stamps.first,
  //               last_point_stamps.second);
  //   ImGui::Text("vel:%.3f %.3f %.3f", last_imu_vel[0], last_imu_vel[1],
  //               last_imu_vel[2]);
  //   ImGui::Text("bias:%.3f %.3f %.3f %.3f %.3f %.3f", last_imu_bias[0],
  //               last_imu_bias[1], last_imu_bias[2], last_imu_bias[3],
  //               last_imu_bias[4], last_imu_bias[5]);
  //   ImGui::End();
  // }
}

void RemoteViewer::viewer_loop() {
  // glim::Config config(glim::GlobalConfig::get_config_path("config_viewer"));

  // auto viewer = guik::LightViewer::instance(
  //     Eigen::Vector2i(config.param("standard_viewer", "viewer_width", 2560),
  //                     config.param("standard_viewer", "viewer_height", 1440)));

  auto viewer = guik::LightViewer::instance(Eigen::Vector2i(640, 480));
  viewer->enable_vsync();
  viewer->shader_setting().add("z_range", z_range);

  if (enable_partial_rendering) {
    viewer->enable_partial_rendering(1e-1);
    viewer->shader_setting().add("dynamic_object", 1);
  }

  viewer->register_drawable_filter(
      "selection",
      [this](const std::string &name) { return drawable_filter(name); });
  viewer->register_ui_callback("selection", [this] { drawable_selection(); });
  // viewer->register_ui_callback(
  //     "logging", guik::create_logger_ui(glim::get_ringbuffer_sink(), 0.5));

  while (!kill_switch) {
    if (!viewer->spin_once()) {
      request_to_terminate = true;
    }

    std::vector<std::function<void()>> tasks;
    {
      std::lock_guard<std::mutex> lock(invoke_queue_mutex);
      tasks.swap(invoke_queue);
    }

    for (const auto &task : tasks) {
      task();
    }
  }

  guik::LightViewer::destroy();
}
