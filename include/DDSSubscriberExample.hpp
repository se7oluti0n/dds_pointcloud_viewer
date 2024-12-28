#pragma once

#include "dds/dds.hpp"
#include "DDSSubscriber.hpp"
#include <PointCloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <thread>

class PointCloudListener;

class DDSSubscriberExample {

friend class PointCloudListener;
public:
  DDSSubscriberExample(rclcpp::Node::SharedPtr node);
  ~DDSSubscriberExample();

  void create_client();
  void receiving_loop();

private:
  // std::shared_ptr<dds::domain::DomainParticipant> participant_;
  // std::shared_ptr<dds::topic::Topic<PointCloudData::PointCloud2>> topic_;
  /* A reader also needs a subscriber. */
  // std::shared_ptr<dds::sub::Subscriber> subscriber_;

  /* Now, the reader can be created to subscribe to a HelloWorld message. */
  std::unique_ptr<DDSSubscriber<PointCloudData::PointCloud2>> dds_subscriber_;
  std::shared_ptr<dds::sub::DataReader<PointCloudData::PointCloud2>> reader_;

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_pub_;

  std::shared_ptr<PointCloudListener> listener_;

  std::mutex pointcloud2_mutex_;
  std::deque<PointCloudData::PointCloud2> pointcloud2_queue_;
  std::shared_ptr<std::thread> receiving_thread_;
  bool running_{false};
};
