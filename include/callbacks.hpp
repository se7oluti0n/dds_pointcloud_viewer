#pragma once

#include <glim/util/callback_slot.hpp>
#include <gtsam_points/types/point_cloud.hpp>
#include <glim/util/raw_points.hpp>
#include <Slam3DIDL.hpp>

namespace glim {

struct DDSCallbacks {
  static CallbackSlot<void(gtsam_points::PointCloud::ConstPtr)> on_pointcloud;
  static CallbackSlot<void(glim::RawPoints::ConstPtr)> on_raw_pointcloud;
  static CallbackSlot<void(std::shared_ptr<const Slam3D::SubmapList>)> on_submap_list;
  static CallbackSlot<void(uint32_t, const Eigen::Isometry3f&, glim::RawPoints::ConstPtr)> on_submap_data;
  static CallbackSlot<void(uint32_t, const Eigen::Isometry3f&, glim::RawPoints::ConstPtr)> on_keyframe;
  static CallbackSlot<void(double, const Eigen::Isometry3f&)> on_lidar_pose;
};
}

