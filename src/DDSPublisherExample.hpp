#pragma once

#include "dds/dds.hpp"
#include "PointCloud.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/rclcpp.hpp>
class DDSPublisherExample {
public:
  DDSPublisherExample(rclcpp::Node::SharedPtr node);

  void create_publisher();
  void ros2_pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  std::shared_ptr<dds::pub::Publisher> publisher_;
  std::shared_ptr<dds::pub::DataWriter<PointCloudData::PointCloud2>> writer_;

  std::shared_ptr<dds::domain::DomainParticipant> participant_;
  std::shared_ptr<dds::topic::Topic<PointCloudData::PointCloud2>> topic_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_sub_;
};
