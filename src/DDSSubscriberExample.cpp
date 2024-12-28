#include "DDSSubscriberExample.hpp"

#include "callbacks.hpp"
#include "pointcloud_converter.hpp"

using dds::domain::DomainParticipant;
using dds::topic::Topic;

sensor_msgs::msg::PointField
from_dds_field(const PointCloudData::PointField &field) {
  sensor_msgs::msg::PointField f;
  f.name = field.name();
  f.offset = field.offset();
  f.datatype = field.datatype();
  f.count = field.count();
  return f;
}

sensor_msgs::msg::PointCloud2
from_dds_pointcloud(const PointCloudData::PointCloud2 &cloud) {
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp.sec = cloud.timestamp();
  msg.header.frame_id = cloud.frame_id();
  msg.height = cloud.height();
  msg.width = cloud.width();
  msg.is_bigendian = cloud.is_bigendian();
  msg.point_step = cloud.point_step();
  msg.row_step = cloud.row_step();
  msg.is_dense = cloud.is_dense();
  msg.fields = std::vector<sensor_msgs::msg::PointField>();
  for (const auto &field : cloud.fields()) {
    sensor_msgs::msg::PointField f = from_dds_field(field);
    msg.fields.push_back(f);
  }
  msg.data = cloud.data();
  return msg;
}

class PointCloudListener
    : public dds::sub::NoOpDataReaderListener<PointCloudData::PointCloud2> {
public:
  PointCloudListener(DDSSubscriberExample *parent) : parent_(parent) {}
  void on_data_available(
      dds::sub::DataReader<PointCloudData::PointCloud2> &reader) override {
    /* Take all samples. */
    try {

      auto preTakeTime = dds_time();
      dds::sub::LoanedSamples<PointCloudData::PointCloud2> samples =
          reader.take();
      std::lock_guard<std::mutex> lock(parent_->pointcloud2_mutex_);
      for (dds::sub::LoanedSamples<PointCloudData::PointCloud2>::const_iterator
               sample_it = samples.begin();
           sample_it != samples.end(); sample_it++) {
        if (sample_it->info().valid()) {
          parent_->pointcloud2_queue_.push_back(sample_it->data());
        }
      }
      auto postTakeTime = dds_time();

      auto difference = (postTakeTime - preTakeTime) / DDS_NSECS_IN_USEC;

      // std::cout << "=== [Publisher] Sent data: "
      //           << ", take : " << difference << std::endl;
    } catch (const dds::core::TimeoutError &) {
      std::cout << "# Timeout encountered.\n" << std::flush;
      return;
    } catch (const dds::core::Exception &e) {
      std::cout << "# Error: \"" << e.what() << "\".\n" << std::flush;
      return;
    } catch (...) {
      std::cout << "# Generic error.\n" << std::flush;
      return;
    }
  }

private:
  DDSSubscriberExample *parent_;
};

DDSSubscriberExample::DDSSubscriberExample(rclcpp::Node::SharedPtr node)
    : node_(node) {

  pointcloud2_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "dds_pointcloud", 10);
  create_client();
  running_ = true;
  receiving_thread_ = std::make_shared<std::thread>(
      &DDSSubscriberExample::receiving_loop, this);
}

DDSSubscriberExample::~DDSSubscriberExample() {
  running_ = false;
  receiving_thread_->join();
}

void DDSSubscriberExample::receiving_loop() {
  while (running_) {
    if (!pointcloud2_queue_.empty()) {
      std::lock_guard<std::mutex> lock(pointcloud2_mutex_);
      auto start_time = dds_time();
      auto msg = pointcloud2_queue_.front();
      pointcloud2_queue_.pop_front();

      sensor_msgs::msg::PointCloud2 msg_ros2;
      msg_ros2 = from_dds_pointcloud(msg);
      pointcloud2_pub_->publish(msg_ros2);

      // emit gtsam pointcloud 
      auto gtsam_pointcloud = glim::extract_raw_points(msg);
      glim::DDSCallbacks::on_raw_pointcloud(gtsam_pointcloud); // emit raw(gtsam_pointcloud);


      auto end_time = dds_time();
      // std::cout << "=== [Subscriber] Received data: "
      //           << ", take : " << (end_time - start_time) / DDS_NSECS_IN_USEC
      //           << std::endl;
    }

    usleep(10000);
  }
}

void DDSSubscriberExample::create_client() {

  std::cout << "=== [Subscriber] Create reader." << std::endl;

  dds::topic::qos::TopicQos tqos;
  tqos << dds::core::policy::Reliability::Reliable(
      dds::core::Duration::from_secs(10));

  dds::sub::qos::SubscriberQos sqos;
  sqos << dds::core::policy::Partition("pong");
  // create listener
  std::cout << "=== [Subscriber] Create listener." << std::endl;

  listener_ = std::make_shared<PointCloudListener>(this);

  dds_subscriber_ = std::make_unique<DDSSubscriber<PointCloudData::PointCloud2>>(
    org::eclipse::cyclonedds::domain::default_id(),                                                                           
    "ManhTopic", listener_.get(), tqos, sqos);
  reader_ = dds_subscriber_->get_reader();
}
