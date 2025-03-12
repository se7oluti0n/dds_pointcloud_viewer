
#include "remote_viewer.hpp"
#include "LidarMapSubscriber.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("dds_subscriber");
  LidarMapSubscriber example(node);
  RemoteViewer viewer;

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
