#pragma once

#include <glim/util/callback_slot.hpp>
#include <gtsam_points/types/point_cloud.hpp>

namespace glim {

struct DDSCallbacks {
  static CallbackSlot<void(gtsam_points::PointCloud::ConstPtr)> on_pointcloud;
};
}

