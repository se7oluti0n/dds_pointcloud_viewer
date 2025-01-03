
#include "remote_viewer.hpp"
#include "DDSSubscriberExample.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("dds_subscriber");
  DDSSubscriberExample example(node);
  RemoteViewer viewer;

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
