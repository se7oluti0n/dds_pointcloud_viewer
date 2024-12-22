#pragma once

#include "dds/dds.hpp"
#include <PointCloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/rclcpp.hpp>
class DDSSubscriberExample {

public:
  DDSSubscriberExample(rclcpp::Node::SharedPtr node);

  void create_client();

private:
  /* A reader also needs a subscriber. */
  std::shared_ptr<dds::sub::Subscriber> subscriber_;

  /* Now, the reader can be created to subscribe to a HelloWorld message. */
  std::shared_ptr<dds::sub::DataReader<PointCloudData::PointCloud2>> reader_;

  rclcpp::Node::SharedPtr node_;
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_pub_;
  
  
};
