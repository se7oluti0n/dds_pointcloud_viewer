#include "Example.hpp"

#include "dds/dds.hpp"
#include "PointCloud.hpp"

Example::Example() {
  // Create participant
  dds::domain::DomainParticipant dp(0);

  // Create topic
  dds::topic::Topic<PointCloudData::PointCloud> topic(dp, "PointCloudTopic");

  // Configure QoS for large data
  dds::pub::qos::PublisherQos pub_qos;
  pub_qos << dds::core::policy::Reliability::Reliable() << dds::core::policy::History::KeepLast(5) << dds::core::policy::ResourceLimits(20, 100, 10);

  // Create publisher and writer
  auto publisher = dds::pub::Publisher(dp, pub_qos);
  auto writer = dds::pub::DataWriter<PointCloudData::PointCloud>(publisher, topic);

  // Create and publish point cloud data
  PointCloudData::PointCloud cloud;
  cloud.timestamp = getCurrentTime();
  cloud.frame_id = "sensor_frame";
}
