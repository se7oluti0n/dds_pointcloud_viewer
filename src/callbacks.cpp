#include "callbacks.hpp"

namespace glim { 
  CallbackSlot<void(gtsam_points::PointCloud::ConstPtr)> DDSCallbacks::on_pointcloud;
}
