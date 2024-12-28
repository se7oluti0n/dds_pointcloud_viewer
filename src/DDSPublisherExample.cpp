#include "DDSPublisherExample.hpp"

#include <chrono>

using namespace org::eclipse::cyclonedds;

PointCloudData::PointField
from_ros2_field(const sensor_msgs::msg::PointField &field) {
  PointCloudData::PointField f;
  f.name() = field.name;
  f.offset() = field.offset;
  f.datatype() = field.datatype;
  f.count() = field.count;
  return f;
}

PointCloudData::PointCloud2
from_ros2_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  PointCloudData::PointCloud2 cloud;
  cloud.timestamp() = msg->header.stamp.sec;
  cloud.frame_id() = msg->header.frame_id;
  cloud.height() = msg->height;
  cloud.width() = msg->width;
  cloud.is_bigendian() = msg->is_bigendian;
  cloud.point_step() = msg->point_step;
  cloud.row_step() = msg->row_step;
  cloud.is_dense() = msg->is_dense;

  for (const auto &field : msg->fields) {
    cloud.fields().push_back(from_ros2_field(field));
  }

  cloud.data() = msg->data;

  return cloud;
}

static bool
match_readers_and_writers(dds::pub::DataWriter<PointCloudData::PointCloud2> &wr,
                          dds::core::Duration timeout) {
  dds::core::cond::WaitSet waitset;

  dds::core::cond::StatusCondition wsc(wr);
  wsc.enabled_statuses(dds::core::status::StatusMask::publication_matched());

  waitset.attach_condition(wsc);

  std::cout << "# Waiting for readers and writers to match up\n" << std::flush;
  try {

    while (0 == wr.publication_matched_status().current_count()) {
      auto conds = waitset.wait(timeout);
      for (const auto &c : conds) {
        if (wsc == c)
          waitset.detach_condition(wsc);
      }
    }

    std::cout << "# Reader and writer have matched.\n" << std::flush;
    return true;
  } catch (const dds::core::TimeoutError &) {
    std::cout << "\nTimeout occurred during matching readers and writers.\n"
              << std::flush;
  } catch (const dds::core::Exception &e) {
    std::cout << "\nThe following error: \"" << e.what()
              << "\" was encountered during matching of readers and writers.\n"
              << std::flush;
  } catch (...) {
    std::cout << "\nA generic error was encountered during matching of readers "
                 "and writers.\n"
              << std::flush;
  }
  return false;
}

DDSPublisherExample::DDSPublisherExample(rclcpp::Node::SharedPtr node)
    : node_(node) {
  pointcloud2_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud2", 10,
      std::bind(&DDSPublisherExample::ros2_pointcloud_callback, this,
                std::placeholders::_1));

  create_publisher();
  running_ = true;
  sending_thread_ = std::make_shared<std::thread>(
      &DDSPublisherExample::sending_loop, this);
}

DDSPublisherExample::~DDSPublisherExample() {
  running_ = false;
  sending_thread_->join();
}

void DDSPublisherExample::create_publisher() {
  dds::topic::qos::TopicQos tqos;
  tqos << dds::core::policy::Reliability::Reliable(
      dds::core::Duration::from_secs(10));
  dds::pub::qos::PublisherQos pqos;
  pqos << dds::core::policy::Partition("pong");
  
  dds_publisher_ = std::make_unique<DDSPublisher<PointCloudData::PointCloud2>>( 
    domain::default_id(), "ManhTopic", tqos, pqos);

  writer_ = dds_publisher_->get_writer();

  std::cout << "=== [Publisher] Waiting for sample to be accepted."
            << std::endl;

  match_readers_and_writers(*writer_, dds::core::Duration::from_secs(10));
}

void DDSPublisherExample::sending_loop()
{
  while(running_) {

    if (!pointcloud2_queue_.empty() && writer_) {
      std::lock_guard<std::mutex> lock(pointcloud2_mutex_);
      auto msg = pointcloud2_queue_.front();
      pointcloud2_queue_.pop_front();


      auto preTakeTime = dds_time();
      PointCloudData::PointCloud2 cloud = from_ros2_pointcloud(msg);
      writer_->write(cloud);
      auto postTakeTime = dds_time();

      auto difference = (postTakeTime - preTakeTime) / DDS_NSECS_IN_USEC;

      // std::cout << "=== [Publisher] Sent data: " << cloud.data().size()
      //           << ", take : " << difference << std::endl;
    }

    usleep(10000);
  }

}

void DDSPublisherExample::ros2_pointcloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // std::cout << "=== [Publisher] Received data: " << msg->data.size() << std::endl;
  std::lock_guard<std::mutex> lock(pointcloud2_mutex_);
  
  pointcloud2_queue_.push_back(msg);
}
