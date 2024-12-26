#pragma once

#include "dds/dds.hpp"
#include "PointCloud.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <thread>
#include <deque>

class DDSPublisherExample {
public:
  DDSPublisherExample(rclcpp::Node::SharedPtr node);
  ~DDSPublisherExample();

  void create_publisher();
  void ros2_pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void sending_loop();

private:
  std::shared_ptr<dds::pub::Publisher> publisher_;
  std::shared_ptr<dds::pub::DataWriter<PointCloudData::PointCloud2>> writer_;

  std::shared_ptr<dds::domain::DomainParticipant> participant_;
  std::shared_ptr<dds::topic::Topic<PointCloudData::PointCloud2>> topic_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_sub_;
  
  std::mutex pointcloud2_mutex_;
  std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> pointcloud2_queue_;
  std::shared_ptr<std::thread> sending_thread_;
  bool running_{false};
};