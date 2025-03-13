#include <data_writer.hpp>
#include <callbacks.hpp>
#include "pointcloud_converter.hpp"
#include "helpers/json_converter.hpp"
#include "idl_data_conversion.hpp"


DataWriter::DataWriter(const std::string& file_name) {
  file_.open(file_name, std::ios::app | std::ios::out);
  if (!file_.is_open()) {
    std::cout << "Cannot open file for writing: " << file_name << "\n";
  }
  start();
}

DataWriter::~DataWriter() {
  if (file_.is_open()) {
    file_.close();
  }
}

void DataWriter::run() {

  while (!kill_switch) {

    std::vector<std::function<void()>> tasks;
    {
      std::lock_guard<std::mutex> lock(invoke_queue_mutex);
      tasks.swap(invoke_queue);
    }

    for (const auto &task : tasks) {
      task();
    }
  }
}

void DataWriter::set_callbacks() {


}

void DataWriter::write(const EstimationFrame::ConstPtr& keyframe) {
  invoke([this, keyframe]() {

    if (!file_.is_open()) return;

    Slam3D::Keyframe dds_keyframe;
    dds_keyframe.keyframe_id() = keyframe->id;
    dds_keyframe.pose() = idl::from_eigen(keyframe->T_world_sensor());
    dds_keyframe.pointcloud() = *frame_to_pointcloud2("odom", 0.0, *keyframe->frame);

    nlohmann::json json_keyframe;

    to_json(json_keyframe, dds_keyframe);

    file_ << json_keyframe << "\n";
  });
}
