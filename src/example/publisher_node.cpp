#include "example/DDSPublisherExample.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("dds_publisher");
  DDSPublisherExample example(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
