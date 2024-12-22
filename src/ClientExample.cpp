#include "ClientExample.hpp"
DDSSubscriberExample::DDSSubscriberExample(rclcpp::Node::SharedPtr node): 
  node_(node) {

 pointcloud2_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud2", 10);
}


void DDSSubscriberExample::create_client() {


  std::cout << "=== [Subscriber] Create reader." << std::endl;

  /* First, a domain participant is needed.
   * Create one on the default domain. */
  dds::domain::DomainParticipant participant(0);

  /* To subscribe to something, a topic is needed. */
  dds::topic::Topic<PointCloudData::PointCloud2> topic(participant,
                                               "HelloWorldData_Msg");

  subscriber_ = std::make_shared<dds::sub::Subscriber>(participant);
  reader_ = std::make_shared<dds::sub::DataReader<PointCloudData::PointCloud2>>(
      *subscriber_, topic);
}
