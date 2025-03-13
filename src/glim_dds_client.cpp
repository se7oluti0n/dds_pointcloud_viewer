#include "glim_dds_client.hpp"
#include "idl_data_conversion.hpp"
#include "pointcloud_converter.hpp"
#include "data_writer.hpp"
#include <glim/mapping/callbacks.hpp>
#include <glim/odometry/callbacks.hpp>

#include <glim/util/logging.hpp>
#include <glim/util/trajectory_manager.hpp>

namespace glim {
GlimDDSClient::GlimDDSClient():
logger_(create_module_logger("dds_client")){
  kill_switch = false;
  request_to_terminate = false;
  trajectory_manager_ = std::make_unique<TrajectoryManager>();
  data_writer_ = std::make_unique<DataWriter>("/home/murphy/pcl.json");

  create_dds_publishers();
  set_callbacks();
  thread = std::thread([this] { run_loop(); });
}

GlimDDSClient::~GlimDDSClient() 
{ 
  kill_switch = true; 
  if (thread.joinable()) {
    thread.join();
  }
} //~Glim

bool GlimDDSClient::ok() const { return true; }

void GlimDDSClient::create_dds_publishers()
{
  // auto domain_id = 
  //   org::eclipse::cyclonedds::domain::default_id();                                                                           
  dds::topic::qos::TopicQos tqos = dds::topic::qos::TopicQos();
  dds::pub::qos::PublisherQos pqos = dds::pub::qos::PublisherQos();

  dds::pub::qos::DataWriterQos wqos_besteffort = dds::pub::qos::DataWriterQos();
  wqos_besteffort << dds::core::policy::Reliability::BestEffort(); 
  wqos_besteffort << dds::core::policy::History::KeepLast(5);
  uint32_t domain_id = 0;

  submap_list_publisher_ = std::make_shared<DDSPublisher<Slam3D::SubmapList>>(
    domain_id, "submap_list", tqos, pqos, wqos_besteffort
  );
  submap_data_publisher_ = std::make_shared<DDSPublisher<Slam3D::SubmapData>>(
    domain_id, "submap_data", tqos, pqos, wqos_besteffort

  );
  keyframe_publisher_ = std::make_shared<DDSPublisher<Slam3D::Keyframe>>(
    domain_id, "keyframe", tqos, pqos, wqos_besteffort

  );
  pose_publisher_ = std::make_shared<DDSPublisher<Common::Pose3DTimestamped>>(
    domain_id, "lidar_pose", tqos, pqos, wqos_besteffort

  );

}

void GlimDDSClient::set_callbacks() {
  // New frame callback
  OdometryEstimationCallbacks::on_new_frame.add([this](const EstimationFrame::ConstPtr& new_frame) {
    invoke([this, new_frame] {
      // auto viewer = guik::LightViewer::instance();
      // auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(new_frame->frame->points, new_frame->frame->size());
      //
      // last_id = new_frame->id;
      // last_num_points = new_frame->frame->size();
      // if (new_frame->raw_frame && new_frame->raw_frame->size()) {
      //   last_point_stamps.first = new_frame->raw_frame->times.front();
      //   last_point_stamps.second = new_frame->raw_frame->times.back();
      // }
      // last_imu_vel = new_frame->v_world_imu;
      // last_imu_bias = new_frame->imu_bias;

      trajectory_manager_->add_odom(new_frame->stamp, new_frame->T_world_sensor(), 1);
      // const Eigen::Isometry3f pose = resolve_pose(new_frame);
      const Eigen::Isometry3f pose = trajectory_manager_->odom2world(new_frame->T_world_sensor()).cast<float>();

      Common::Pose3DTimestamped dds_pose;
      dds_pose.timestamp() = new_frame->stamp;
      dds_pose.pose() = idl::from_eigen(pose.cast<double>());
      pose_publisher_->get_writer()->write(dds_pose);
      // logger_->info("[dds client] Publish pose: {}", new_frame->stamp);

      // if (track) {
      //   viewer->lookat(pose.translation());
      // }


      // viewer->update_drawable("current_coord", glk::Primitives::coordinate_system(), guik::VertexColor(pose * Eigen::UniformScaling<float>(1.5f)));
    });
  });
  GlobalMappingCallbacks::on_insert_submap.add([this](const SubMap::ConstPtr& submap){
    invoke([this, submap]{
      Slam3D::SubmapData dds_submap_data;
      dds_submap_data.submap_id() = submap->id;
      dds_submap_data.pose() = idl::from_eigen(submap->T_world_origin);
      dds_submap_data.pointcloud() = *frame_to_pointcloud2("odom", 0.0, *submap->frame);
      submap_data_publisher_->get_writer()->write(dds_submap_data);
      logger_->info("[dds client] Publish submap data: {}", submap->id);
      
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
      logger_->info("[dds client] published submap list: {}", dds_submap_list.submap_list().size());
    });

  });

  SubMappingCallbacks::on_new_keyframe.add([this](int id, const EstimationFrame::ConstPtr& keyframe){
    // publish key frame
    const EstimationFrame::ConstPtr kf = keyframe;
    data_writer_->write(kf);

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
