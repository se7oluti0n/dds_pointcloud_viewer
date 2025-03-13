#pragma once

// local include
#include "dds/DDSPublisher.hpp"
#include "Slam3DIDL.hpp"

#include <mutex>
#include <functional>
#include <thread>

#include <glim/util/extension_module.hpp>
namespace spdlog {
class logger;
}
class DataWriter;
namespace glim {

class TrajectoryManager;

class GlimDDSClient: public ExtensionModule 
{
public:
  GlimDDSClient();
  ~GlimDDSClient();

  virtual bool ok() const override;

private:
  void create_dds_publishers();
  void set_callbacks();
  void run_loop();
  void invoke(const std::function<void()> &task);
  // Private variables

  std::atomic_bool request_to_terminate;
  std::atomic_bool kill_switch;
  std::thread thread;

  std::mutex invoke_queue_mutex;
  std::vector<std::function<void()>> invoke_queue;
  DDSPublisher<Slam3D::SubmapList>::Ptr submap_list_publisher_;
  DDSPublisher<Slam3D::Keyframe>::Ptr keyframe_publisher_;
  DDSPublisher<Slam3D::SubmapData>::Ptr submap_data_publisher_;
  DDSPublisher<Common::Pose3DTimestamped>::Ptr pose_publisher_;
  std::shared_ptr<spdlog::logger> logger_;

  std::unique_ptr<TrajectoryManager> trajectory_manager_;
  std::unique_ptr<DataWriter> data_writer_;

};

}
