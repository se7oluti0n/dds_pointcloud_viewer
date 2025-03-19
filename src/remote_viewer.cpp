#include "remote_viewer.hpp"

#include "callbacks.hpp"
#include "idl_data_conversion.hpp"

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

const int max_keyframe_num = 15;

RemoteViewer::RemoteViewer(): CallbackWorker() {

  kill_switch = false;
  request_to_terminate = false;
  track = true;

  current_color_mode = 0;
  z_range = Eigen::Vector2d(-2.0, 4.0).cast<float>();
  auto_z_range << 0.0f, 0.0f;

  enable_partial_rendering = false;
  partial_rendering_budget = 1024;
  set_callbacks();
  start();
}

RemoteViewer::~RemoteViewer() {
}

// bool RemoteViewer::ok() const { return !request_to_terminate; }


void RemoteViewer::set_callbacks() {

  glim::DDSCallbacks::on_raw_pointcloud.add(
      [this](glim::RawPoints::ConstPtr gtsam_pointcloud) {
        std::cout << "Start handle raw pointcloud" << std::endl;
        invoke([this, gtsam_pointcloud] {
          auto viewer = guik::LightViewer::instance();
          viewer->use_topdown_camera_control();
        

          auto cloud_buffer =
              std::make_shared<glk::PointCloudBuffer>(gtsam_pointcloud->points);
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

  glim::DDSCallbacks::on_submap_list.add(
      [this](std::shared_ptr<const Slam3D::SubmapList> submaps) {
        std::cout << "start handle submap list: "
                  << submaps->submap_list().size() << std::endl;
        if (submaps->submap_list().size() == 0)
          return;

        invoke([this, submaps] {
          std::vector<int> submap_ids(submaps->submap_list().size());
          std::vector<Eigen::Isometry3f> submap_poses(
              submaps->submap_list().size());
          for (int i = 0; i < submaps->submap_list().size(); i++) {
            submap_ids[i] = submaps->submap_list()[i].submap_id();
            submap_poses[i] =
                idl::to_eigen(submaps->submap_list()[i].submap_pose())
                    .cast<float>();
          }

          auto viewer = guik::LightViewer::instance();

          std::vector<Eigen::Vector3f> submap_positions(submap_ids.size());

          for (int i = 0; i < submap_poses.size(); i++) {
            submap_positions[i] = submap_poses[i].translation();

            auto_z_range[0] = std::min<float>(
                auto_z_range[0], submap_poses[i].translation().z());
            auto_z_range[1] = std::max<float>(
                auto_z_range[1], submap_poses[i].translation().z());

            auto drawable = viewer->find_drawable(
                "submap_" + std::to_string(submap_ids[i]));
            if (drawable.first) {
              drawable.first->add("model_matrix", submap_poses[i].matrix());
            }
            viewer->update_drawable("submap_coord_" +
                                        std::to_string(submap_ids[i]),
                                    glk::Primitives::coordinate_system(),
                                    guik::VertexColor(submap_poses[i]));
          }

          viewer->shader_setting().add<Eigen::Vector2f>("z_range",
                                                        auto_z_range + z_range);
        });
      });

  glim::DDSCallbacks::on_submap_data.add(
      [this](uint32_t submap_id, const Eigen::Isometry3f &submap_pose,
             glim::RawPoints::ConstPtr submap_points) {
        // const double stamp_endpoint_R = submap->odom_frames.back()->stamp;
        // const Eigen::Isometry3d T_world_endpoint_R = (*T_world_origin) *
        // submap->T_origin_endpoint_R;
        // trajectory->update_anchor(stamp_endpoint_R, T_world_endpoint_R);

        invoke([this, submap_id, submap_pose, submap_points] {
          std::cout << "[Viewer] render submap data" << submap_id << std::endl;
          auto viewer = guik::LightViewer::instance();
          auto cloud_buffer =
              std::make_shared<glk::PointCloudBuffer>(submap_points->points);
          auto shader_setting = guik::Rainbow(submap_pose.matrix());
          shader_setting.add("point_scale", 1.0f);

          if (enable_partial_rendering) {
            cloud_buffer->enable_partial_rendering(partial_rendering_budget);
            shader_setting.add("dynamic_object", 0).make_transparent();
          }

          viewer->update_drawable("submap_" + std::to_string(submap_id),
                                  cloud_buffer, shader_setting);
        });
      });

  glim::DDSCallbacks::on_keyframe.add(
      [this](uint32_t keyframe_Id, const Eigen::Isometry3f &keyframe_pose,
             glim::RawPoints::ConstPtr keyframe_points) {
        invoke([this, keyframe_Id, keyframe_pose, keyframe_points] {
          auto keyframe_id = keyframe_Id % max_keyframe_num;

          std::cout << " [Viewer] render keyframe" << keyframe_id << std::endl;
          auto viewer = guik::LightViewer::instance();
          auto cloud_buffer =
              std::make_shared<glk::PointCloudBuffer>(keyframe_points->points);
          // TODO: update keyframe pose from odom frame to world frame
          guik::ShaderSetting shader_setting =
              guik::FlatColor(1.0f, 0.5f, 0.0f, 1.0f, keyframe_pose.matrix());
          // auto shader_setting = guik::Rainbow(keyframe_pose.matrix());
          shader_setting.add("point_scale", 0.8f);

          viewer->update_drawable("keyframe_" + std::to_string(keyframe_id),
                                  cloud_buffer, shader_setting);
        });
      });

  glim::DDSCallbacks::on_lidar_pose.add(
      [this](double stamp, const Eigen::Isometry3f &pose) {
        invoke([this, stamp, pose] {
          auto viewer = guik::LightViewer::instance();
          if (track) {
            viewer->lookat(pose.translation());
          }
          viewer->update_drawable("lidar_pose",
                                  glk::Primitives::coordinate_system(),
                                  guik::VertexColor(pose));
        });
      });
}

bool RemoteViewer::drawable_filter(const std::string &name) { return true; }


void RemoteViewer::drawable_session_list() {
// fill imu gui UI here
  auto viewer = guik::LightViewer::instance();
  ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.5));
  ImGui::Begin("session_list", nullptr, ImGuiWindowFlags_ChildMenu);
  ImGui::PopStyleColor();

  std::vector<const char *> current_color_modes = {"FLAT", "INTENSITY",
                                                   "NORMAL"};

  for (auto i = 0; i < current_color_modes.size(); i++) {
    if (ImGui::Button(current_color_modes[i])) {
      current_color_mode = i;
    }
  }
  ImGui::End();
}

void RemoteViewer::drawable_selection() {
  auto viewer = guik::LightViewer::instance();

  ImGui::SetWindowPos("images", {1800, 60}, ImGuiCond_FirstUseEver);
  ImGui::SetWindowPos("logging", {1800, 950}, ImGuiCond_FirstUseEver);

  ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.5));
  ImGui::Begin("selection", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
  ImGui::PopStyleColor();

  if (ImGui::Checkbox("track", &track)) {
    if (track) {
      guik::LightViewer::instance()->reset_center();
    }
  }
  // ImGui::SameLine();
  // bool show_current = show_current_coord || show_current_points;
  // if (ImGui::Checkbox("current", &show_current)) {
  //   show_current_coord = show_current_points = show_current;
  // }
  // ImGui::SameLine();
  // ImGui::Checkbox("coord", &show_current_coord);
  // ImGui::SameLine();
  // ImGui::Checkbox("points", &show_current_points);

  // std::vector<const char *> current_color_modes = {"FLAT", "INTENSITY",
  //                                                  "NORMAL"};
  // ImGui::SetNextItemWidth(92);
  // ImGui::Combo("color_mode", &current_color_mode, current_color_modes.data(),
  //              current_color_modes.size());

  // ImGui::SameLine();
  // if (ImGui::Button("Log")) {
  //   viewer->register_ui_callback(
  //       "logging", guik::create_logger_ui(glim::get_ringbuffer_sink(), 0.5));
  // }
  if (ImGui::Button("Session list")) {
    viewer->register_ui_callback("session_list", [this]{ drawable_session_list();});  
  }

  // ImGui::Separator();
  // bool show_odometry =
  //     show_odometry_scans || show_odometry_keyframes ||
  //     show_odometry_factors;
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
  // if (ImGui::DragFloatRange2("z_range", &z_range[0], &z_range[1], 0.1f,
  // -100.0f,
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

void RemoteViewer::run() {
  // glim::Config config(glim::GlobalConfig::get_config_path("config_viewer"));

  // auto viewer = guik::LightViewer::instance(
  //     Eigen::Vector2i(config.param("standard_viewer", "viewer_width", 2560),
  //                     config.param("standard_viewer", "viewer_height",
  //                     1440)));

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
