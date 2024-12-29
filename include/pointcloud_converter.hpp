#pragma once

#include <Eigen/Core>
#include <gtsam_points/types/point_cloud.hpp>
#include <glim/util/raw_points.hpp>
#include <PointcloudIDL.hpp>


enum PointFieldType {
    INT8    = 1,
    UINT8   = 2,
    INT16   = 3,
    UINT16  = 4,
    INT32   = 5,
    UINT32  = 6,
    FLOAT32 = 7,
    FLOAT64 = 8
};


namespace glim {
using PointCloud2 = PointCloudData::PointCloud2;
using PointCloud2Ptr = std::shared_ptr<PointCloud2>;
using PointCloud2ConstPtr = const std::shared_ptr<PointCloud2>; 
using PointField = PointCloudData::PointField;


template <typename T>
Eigen::Vector4d get_vec4(const void* x, const void* y, const void* z) {
  return Eigen::Vector4d(*reinterpret_cast<const T*>(x), *reinterpret_cast<const T*>(y), *reinterpret_cast<const T*>(z), 1.0);
}

static PointCloud2ConstPtr frame_to_pointcloud2(const std::string& frame_id, const double stamp, const gtsam_points::PointCloud& frame) {
  PointCloud2Ptr msg(new PointCloud2);

  msg->width() = frame.size();
  msg->height() = 1;
  msg->frame_id() = frame_id;
  msg->timestamp() = stamp;

  std::vector<std::string> field_names = {"x", "y", "z", "t"};
  int num_fields = frame.times ? 4 : 3;
  msg->fields().resize(num_fields);

  for (int i = 0; i < num_fields; i++) {
    msg->fields()[i].name() = field_names[i];
    msg->fields()[i].offset() = sizeof(float) * i;
    msg->fields()[i].datatype() = PointFieldType::FLOAT32;
    msg->fields()[i].count() = 1;
  }

  msg->is_bigendian() = false;
  msg->point_step() = sizeof(float) * num_fields;
  msg->row_step() = sizeof(float) * num_fields * frame.size();

  msg->data().resize(sizeof(float) * num_fields * frame.size());
  for (int i = 0; i < frame.size(); i++) {
    float* point = reinterpret_cast<float*>(msg->data().data() + msg->point_step() * i);
    for (int j = 0; j < 3; j++) {
      point[j] = frame.points[i][j];
    }

    if (frame.times) {
      point[3] = frame.times[i];
    }
  }

  return msg;
}

static RawPoints::Ptr extract_raw_points(const PointCloud2& points_msg, const std::string& intensity_channel = "intensity") {
  int num_points = points_msg.width() * points_msg.height();

  int x_type = 0;
  int y_type = 0;
  int z_type = 0;
  int time_type = 0;  // ouster and livox
  int intensity_type = 0;
  int color_type = 0;

  int x_offset = -1;
  int y_offset = -1;
  int z_offset = -1;
  int time_offset = -1;
  int intensity_offset = -1;
  int color_offset = -1;

  std::unordered_map<std::string, std::pair<int*, int*>> fields;
  fields["x"] = std::make_pair(&x_type, &x_offset);
  fields["y"] = std::make_pair(&y_type, &y_offset);
  fields["z"] = std::make_pair(&z_type, &z_offset);
  fields["t"] = std::make_pair(&time_type, &time_offset);
  fields["time"] = std::make_pair(&time_type, &time_offset);
  fields["time_stamp"] = std::make_pair(&time_type, &time_offset);
  fields["timestamp"] = std::make_pair(&time_type, &time_offset);
  fields[intensity_channel] = std::make_pair(&intensity_type, &intensity_offset);
  fields["rgba"] = std::make_pair(&color_type, &color_offset);

  for (const auto& field : points_msg.fields()) {
    auto found = fields.find(field.name());
    if (found == fields.end()) {
      continue;
    }

    *found->second.first = field.datatype();
    *found->second.second = field.offset();
  }

  if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
    //spdlog::warn("missing point coordinate fields");
    return nullptr;
  }

  if ((x_type != PointFieldType::FLOAT32 && x_type != PointFieldType::FLOAT64) || x_type != y_type || x_type != y_type) {
    //spdlog::warn("unsupported points type");
    return nullptr;
  }

  auto raw_points = std::make_shared<RawPoints>();

  raw_points->points.resize(num_points);

  if (x_type == PointFieldType::FLOAT32 && y_offset == x_offset + sizeof(float) && z_offset == y_offset + sizeof(float)) {
    // Special case: contiguous 3 floats
    for (int i = 0; i < num_points; i++) {
      const auto* x_ptr = &points_msg.data()[points_msg.point_step() * i + x_offset];
      raw_points->points[i] << Eigen::Map<const Eigen::Vector3f>(reinterpret_cast<const float*>(x_ptr)).cast<double>(), 1.0;
    }
  } else if (x_type == PointFieldType::FLOAT64 && y_offset == x_offset + sizeof(double) && z_offset == y_offset + sizeof(double)) {
    // Special case: contiguous 3 doubles
    for (int i = 0; i < num_points; i++) {
      const auto* x_ptr = &points_msg.data()[points_msg.point_step() * i + x_offset];
      raw_points->points[i] << Eigen::Map<const Eigen::Vector3d>(reinterpret_cast<const double*>(x_ptr)), 1.0;
    }
  } else {
    for (int i = 0; i < num_points; i++) {
      const auto* x_ptr = &points_msg.data()[points_msg.point_step() * i + x_offset];
      const auto* y_ptr = &points_msg.data()[points_msg.point_step() * i + y_offset];
      const auto* z_ptr = &points_msg.data()[points_msg.point_step() * i + z_offset];

      if (x_type == PointFieldType::FLOAT32) {
        raw_points->points[i] = get_vec4<float>(x_ptr, y_ptr, z_ptr);
      } else {
        raw_points->points[i] = get_vec4<double>(x_ptr, y_ptr, z_ptr);
      }
    }
  }

  if (time_offset >= 0) {
    raw_points->times.resize(num_points);

    for (int i = 0; i < num_points; i++) {
      const auto* time_ptr = &points_msg.data()[points_msg.point_step() * i + time_offset];
      switch (time_type) {
        case PointFieldType::UINT32:
          raw_points->times[i] = *reinterpret_cast<const uint32_t*>(time_ptr) / 1e9;
          break;
        case PointFieldType::FLOAT32:
          raw_points->times[i] = *reinterpret_cast<const float*>(time_ptr);
          break;
        case PointFieldType::FLOAT64:
          raw_points->times[i] = *reinterpret_cast<const double*>(time_ptr);
          break;
        default:
          //spdlog::warn("unsupported time type {}", time_type);
          return nullptr;
      }
    }
  }

  if (intensity_offset >= 0) {
    raw_points->intensities.resize(num_points);

    for (int i = 0; i < num_points; i++) {
      const auto* intensity_ptr = &points_msg.data()[points_msg.point_step() * i + intensity_offset];
      switch (intensity_type) {
        case PointFieldType::UINT8:
          raw_points->intensities[i] = *reinterpret_cast<const std::uint8_t*>(intensity_ptr);
          break;
        case PointFieldType::UINT16:
          raw_points->intensities[i] = *reinterpret_cast<const std::uint16_t*>(intensity_ptr);
          break;
        case PointFieldType::UINT32:
          raw_points->intensities[i] = *reinterpret_cast<const std::uint32_t*>(intensity_ptr);
          break;
        case PointFieldType::FLOAT32:
          raw_points->intensities[i] = *reinterpret_cast<const float*>(intensity_ptr);
          break;
        case PointFieldType::FLOAT64:
          raw_points->intensities[i] = *reinterpret_cast<const double*>(intensity_ptr);
          break;
        default:
          //spdlog::warn("unsupported intensity type {}", intensity_type);
          return nullptr;
      }
    }
  }

  if (color_offset >= 0) {
    if (color_type != PointFieldType::UINT32) {
      //spdlog::warn("unsupported color type {}", color_type);
    } else {
      raw_points->colors.resize(num_points);

      for (int i = 0; i < num_points; i++) {
        const auto* color_ptr = &points_msg.data()[points_msg.point_step() * i + color_offset];
        raw_points->colors[i] = Eigen::Matrix<unsigned char, 4, 1>(reinterpret_cast<const std::uint8_t*>(color_ptr)).cast<double>() / 255.0;
      }
    }
  }

  raw_points->stamp = points_msg.timestamp();
  return raw_points;
}
}
