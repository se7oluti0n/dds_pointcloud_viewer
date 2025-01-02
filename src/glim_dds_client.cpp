#include "glim_dds_client.hpp"
#include "idl_data_conversion.hpp"
#include "pointcloud_converter.hpp"
#include <glim/mapping/callbacks.hpp>

#include <glim/util/logging.hpp>

namespace glim {
GlimDDSClient::GlimDDSClient():
logger_(create_module_logger("dds_client")){
  kill_switch = false;
  request_to_terminate = false;

  create_dds_publishers();
  set_callbacks();
  thread = std::thread([this] { run_loop(); });
}

bool GlimDDSClient::ok() const { return true; }

void GlimDDSClient::create_dds_publishers()
{
  auto domain_id = 
    org::eclipse::cyclonedds::domain::default_id();                                                                           

  submap_list_publisher_ = std::make_shared<DDSPublisher<Slam3D::SubmapList>>(
    domain_id, "submap_list"
  );
  submap_data_publisher_ = std::make_shared<DDSPublisher<Slam3D::SubmapData>>(
    domain_id, "submap_data"
  );
  keyframe_publisher_ = std::make_shared<DDSPublisher<Slam3D::Keyframe>>(
    domain_id, "keyframe"
  );

}

void GlimDDSClient::set_callbacks() {
  GlobalMappingCallbacks::on_insert_submap.add([this](const SubMap::ConstPtr& submap){
    invoke([this, submap]{
      Slam3D::SubmapData dds_submap_data;
      dds_submap_data.submap_id() = submap->id;
      dds_submap_data.pose() = idl::from_eigen(submap->T_world_origin);
      dds_submap_data.pointcloud() = *frame_to_pointcloud2("odom", 0.0, *submap->frame);
      submap_data_publisher_->get_writer()->write(dds_submap_data);
      logger_->info("dds insert submap: {}", submap->id);
      
    });

  });

  GlobalMappingCallbacks::on_update_submaps.add([this](const std::vector<SubMap::Ptr>& submaps){
    // update submap list (pose)
    invoke([this, submaps]{
      Slam3D::SubmapList dds_submap_list;
      for (const auto& submap : submaps){
        Slam3D::Submap dds_submap;
        dds_submap.submap_id() = submap->id;
        dds_submap.submap_pose() = idl::from_eigen(submap->T_world_origin);
        dds_submap_list.submap_list().push_back(dds_submap);
      }
      submap_list_publisher_->get_writer()->write(dds_submap_list);
      logger_->info("dds published submap list: {}", dds_submap_list.submap_list().size());
    });

  });

  SubMappingCallbacks::on_new_keyframe.add([this](int id, const EstimationFrame::ConstPtr& keyframe){
    // publish key frame

    invoke([this, keyframe]{
      Slam3D::Keyframe dds_keyframe;
      dds_keyframe.keyframe_id() = keyframe->id;
      dds_keyframe.pose() = idl::from_eigen(keyframe->T_world_sensor());
      dds_keyframe.pointcloud() = *frame_to_pointcloud2("odom", 0.0, *keyframe->frame);
      keyframe_publisher_->get_writer()->write(dds_keyframe);
    });


  });
}

void GlimDDSClient::invoke(const std::function<void()> &task) {
  if (kill_switch) {
    return;
  }
  std::lock_guard<std::mutex> lock(invoke_queue_mutex);
  invoke_queue.push_back(task);
}

void GlimDDSClient::run_loop()
{

  while (!kill_switch) {
    std::vector<std::function<void()>> tasks;
    {
      std::lock_guard<std::mutex> lock(invoke_queue_mutex);
      tasks.swap(invoke_queue);
    }

    for (const auto &task : tasks) {
      task();
    }

    usleep(10000);
  }
}

} // namespace glim


extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::GlimDDSClient();
}
