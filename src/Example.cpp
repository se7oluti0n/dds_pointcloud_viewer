#include "Example.hpp"


using namespace org::eclipse::cyclonedds;

PointCloudData::PointField from_ros2_field(const sensor_msgs::msg::PointField &field) {
  PointCloudData::PointField f;
  f.name() = field.name;
  f.offset() = field.offset;
  f.datatype() = field.datatype;
  f.count() = field.count;
  return f;
}

PointCloudData::PointCloud2 from_ros2_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  PointCloudData::PointCloud2 cloud;
  cloud.timestamp() = msg->header.stamp.sec;
  cloud.frame_id() = msg->header.frame_id;
  cloud.height() = msg->height;
  cloud.width() = msg->width;
  cloud.is_bigendian() = msg->is_bigendian;
  cloud.point_step() = msg->point_step;
  cloud.row_step() = msg->row_step;
  cloud.is_dense() = msg->is_dense;

  for (const auto &field : msg->fields) {
    cloud.fields().push_back(from_ros2_field(field));
  }

  cloud.data() = msg->data;

  return cloud;
}


DDSPublisherExample::DDSPublisherExample(rclcpp::Node::SharedPtr node):
  node_(node)
{
  create_publisher();
  pointcloud2_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud2", 10, std::bind(&DDSPublisherExample::ros2_pointcloud_callback, this, std::placeholders::_1));
}

void DDSPublisherExample::create_publisher() {
  // Create participant
  dds::domain::DomainParticipant dp(0);

  // Create topic
  dds::topic::Topic<PointCloudData::PointCloud2> topic(dp, "PointCloudTopic");

  // Configure QoS for large data
  dds::pub::qos::PublisherQos pub_qos;
  // pub_qos << dds::core::policy::Reliability::Reliable() <<
  // dds::core::policy::History::KeepLast(5) <<
  // dds::core::policy::ResourceLimits(20, 100, 10);
  //  Create publisher and writer
  publisher_ = std::make_shared<dds::pub::Publisher>(dp, pub_qos);
  writer_ =
      std::make_shared<dds::pub::DataWriter<PointCloudData::PointCloud2>>(*publisher_, topic);

}

void DDSPublisherExample::ros2_pointcloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  std::cout << "Received ROS2 PointCloud message" << std::endl;

  PointCloudData::PointCloud2 cloud = from_ros2_pointcloud(msg);
  writer_->write(cloud);
}

