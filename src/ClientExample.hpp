#pragma once

#include "dds/dds.hpp"
#include <PointCloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/rclcpp.hpp>

class PointCloudListener : public dds::sub::NoOpDataReaderListener<PointCloudData::PointCloud2> {
public:
  PointCloudListener(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_pub)
      : pointcloud2_pub_(pointcloud2_pub) {}
  void on_data_available(dds::sub::DataReader<PointCloudData::PointCloud2> &reader) override;

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_pub_;
};

class DDSSubscriberExample {

public:
  DDSSubscriberExample(rclcpp::Node::SharedPtr node);

  void create_client();

private:
  std::shared_ptr<dds::domain::DomainParticipant> participant_;
  std::shared_ptr<dds::topic::Topic<PointCloudData::PointCloud2>> topic_;
  /* A reader also needs a subscriber. */
  std::shared_ptr<dds::sub::Subscriber> subscriber_;

  /* Now, the reader can be created to subscribe to a HelloWorld message. */
  std::shared_ptr<dds::sub::DataReader<PointCloudData::PointCloud2>> reader_;

  rclcpp::Node::SharedPtr node_;
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_pub_;

  std::shared_ptr<PointCloudListener> listener_;
  
  
};
