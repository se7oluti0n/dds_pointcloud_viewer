#pragma once

#include "DDSSubscriber.hpp"
#include "DDSListener.hpp"
#include <PointcloudIDL.hpp>
#include <Slam3DIDL.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <thread>


class DDSSubscriberExample {

public:
  DDSSubscriberExample(rclcpp::Node::SharedPtr node);
  ~DDSSubscriberExample();

  void create_client();
  void create_submap_list_subscriber();
  void create_submap_data_subscriber();
  void create_keyframe_subscriber();

  void receiving_loop();

private:
  std::unique_ptr<DDSSubscriber<PointCloudData::PointCloud2>> pointcloud_subscriber_;
  std::unique_ptr<DDSSubscriber<Slam3D::SubmapList>> submap_list_subscriber_;
  std::unique_ptr<DDSSubscriber<Slam3D::SubmapData>> submap_data_subscriber_;
  std::unique_ptr<DDSSubscriber<Slam3D::Keyframe>> keyframe_subscriber_;

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_pub_;

  std::shared_ptr<DDSListener<PointCloudData::PointCloud2>> listener_;
  std::shared_ptr<DDSListener<Slam3D::SubmapList>> submap_list_listener_;
  std::shared_ptr<DDSListener<Slam3D::SubmapData>> submap_data_listener_;
  std::shared_ptr<DDSListener<Slam3D::Keyframe>> keyframe_listener_;

  std::shared_ptr<std::thread> receiving_thread_;
  bool running_{false};

  // worker thread
  void invoke(const std::function<void()> &task);
  std::mutex invoke_queue_mutex;
  std::vector<std::function<void()>> invoke_queue;
};
