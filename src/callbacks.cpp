#include "callbacks.hpp"

namespace glim { 
  CallbackSlot<void(gtsam_points::PointCloud::ConstPtr)> DDSCallbacks::on_pointcloud;
  CallbackSlot<void(glim::RawPoints::ConstPtr)> DDSCallbacks::on_raw_pointcloud;
}
