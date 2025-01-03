#pragma once

// local include
#include "DDSPublisher.hpp"
#include "Slam3DIDL.hpp"

#include <mutex>
#include <functional>
#include <thread>

#include <glim/util/extension_module.hpp>
namespace spdlog {
class logger;
}
namespace glim {

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
  std::shared_ptr<spdlog::logger> logger_;

};

}
