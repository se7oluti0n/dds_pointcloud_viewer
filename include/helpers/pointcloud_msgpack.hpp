#ifndef HELPERS_POINT_CLOUD_HPP
#define HELPERS_POINT_CLOUD_HPP

#include <msgpack.hpp>
#include <vector>
#include <string>
#include <gtsam_points/types/point_cloud.hpp>
#include <rclcpp/rclcpp.hpp>



struct PointFieldMsgPack {
    std::string name;
    uint32_t offset;
    uint32_t datatype;
    uint32_t count;

    MSGPACK_DEFINE(name, offset, datatype, count);
};

struct PointCloud2MsgPack {
    double timestamp;
    std::string frame_id;
    uint32_t height;
    uint32_t width;
    std::vector<PointFieldMsgPack> fields;
    bool is_bigendian;
    uint32_t point_step;
    uint32_t row_step;
    std::vector<uint8_t> data;
    bool is_dense;

    MSGPACK_DEFINE(timestamp, frame_id, height, width, fields, 
                   is_bigendian, point_step, row_step, data, is_dense);
};


void to_pointcloud2(const std::string& frame_id, const gtsam_points::PointCloud& frame, PointCloud2MsgPack& point_cloud) {
  point_cloud.frame_id = frame_id;
  point_cloud.timestamp = rclcpp::Clock().now().seconds();
  point_cloud.width = frame.size();
  point_cloud.height = 1;
  std::vector<std::string> field_names = {"x", "y", "z", "t"};
  int num_fields = frame.times ? 4 : 3;
  point_cloud.fields.resize(num_fields);
  for (int i = 0; i < num_fields; i++) {
    point_cloud.fields[i].name = field_names[i];
    point_cloud.fields[i].offset = sizeof(float) * i;
    point_cloud.fields[i].datatype = PointFieldType::FLOAT32;
    point_cloud.fields[i].count = 1;
  }
  point_cloud.is_bigendian = false;
  point_cloud.point_step = sizeof(float) * num_fields;
  point_cloud.row_step = sizeof(float) * num_fields * frame.size();

  point_cloud.data.resize(sizeof(float) * num_fields * frame.size());
  for (int i = 0; i < frame.size(); i++) {
    float* point = reinterpret_cast<float*>(point_cloud.data.data() + point_cloud.point_step * i);
    for (int j = 0; j < 3; j++) {
      point[j] = frame.points[i][j];
    }

    if (frame.times) {
      point[3] = frame.times[i];
    }
  }
}


#endif // HELPERS_POINT_CLOUD_HPP
