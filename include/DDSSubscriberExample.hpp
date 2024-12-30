#pragma once

#include "DDSSubscriber.hpp"
#include "DDSListener.hpp"
#include <PointcloudIDL.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <thread>


class DDSSubscriberExample {

friend class PointCloudListener;
public:
  DDSSubscriberExample(rclcpp::Node::SharedPtr node);
  ~DDSSubscriberExample();

  void create_client();
  void receiving_loop();

private:
  std::unique_ptr<DDSSubscriber<PointCloudData::PointCloud2>> dds_subscriber_;
  std::shared_ptr<dds::sub::DataReader<PointCloudData::PointCloud2>> reader_;

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_pub_;

  std::shared_ptr<DDSListener<PointCloudData::PointCloud2>> listener_;

  // std::mutex pointcloud2_mutex_;
  // std::deque<std::shared_ptr<PointCloudData::PointCloud2>> pointcloud2_queue_;
  std::shared_ptr<std::thread> receiving_thread_;
  bool running_{false};

  // worker thread
  void invoke(const std::function<void()> &task);
  std::mutex invoke_queue_mutex;
  std::vector<std::function<void()>> invoke_queue;
};
