#pragma once

#include "CommonIDL.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace idl {

Common::Vector3D from_eigen(const Eigen::Vector3d &v) {
  Common::Vector3D res;
  res.x() = v.x();
  res.y() = v.y();
  res.z() = v.z();
  return res;
}

Common::Quaternion from_eigen(const Eigen::Quaterniond &q) {
  Common::Quaternion res;
  res.x() = q.x();
  res.y() = q.y();
  res.z() = q.z();
  res.w() = q.w();
  return res;
}

Common::Pose3D from_eigen(const Eigen::Isometry3d &pose) {
  Common::Pose3D res;
  res.translation() = from_eigen(pose.translation());
  res.rotation() = from_eigen(Eigen::Quaterniond(pose.linear()));
  return res;
}

Eigen::Vector3d to_eigen(const Common::Vector3D &v) {
  Eigen::Vector3d res;
  res.x() = v.x();
  res.y() = v.y();
  res.z() = v.z();
  return res;
}

Eigen::Quaterniond to_eigen(const Common::Quaternion &q) {
  Eigen::Quaterniond res;
  res.x() = q.x();
  res.y() = q.y();
  res.z() = q.z();
  res.w() = q.w();
  return res;
}

Eigen::Isometry3d to_eigen(const Common::Pose3D &pose) {
  Eigen::Isometry3d res;
  res.translation() = to_eigen(pose.translation());
  res.linear() = to_eigen(pose.rotation()).toRotationMatrix();
  return res;
}
} // namespace idl
