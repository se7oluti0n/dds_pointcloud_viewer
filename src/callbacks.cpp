#include "callbacks.hpp"

namespace glim { 
  CallbackSlot<void(gtsam_points::PointCloud::ConstPtr)> DDSCallbacks::on_pointcloud;
  CallbackSlot<void(glim::RawPoints::ConstPtr)> DDSCallbacks::on_raw_pointcloud;
  CallbackSlot<void(std::shared_ptr<const Slam3D::SubmapList>)> DDSCallbacks::on_submap_list;
  CallbackSlot<void(uint32_t, const Eigen::Isometry3f&, glim::RawPoints::ConstPtr)> DDSCallbacks::on_submap_data;
  CallbackSlot<void(uint32_t, const Eigen::Isometry3f&, glim::RawPoints::ConstPtr)> DDSCallbacks::on_keyframe;
  CallbackSlot<void(double, const Eigen::Isometry3f&)> DDSCallbacks::on_lidar_pose;
}
