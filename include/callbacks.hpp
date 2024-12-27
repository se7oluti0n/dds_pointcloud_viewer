#pragma once

#include <glim/util/callback_slot.hpp>
#include <gtsam_points/types/point_cloud.hpp>
#include <glim/util/raw_points.hpp>

namespace glim {

struct DDSCallbacks {
  static CallbackSlot<void(gtsam_points::PointCloud::ConstPtr)> on_pointcloud;
  static CallbackSlot<void(glim::RawPoints::ConstPtr)> on_raw_pointcloud;
};
}

