#pragma once

#include "dds/dds.hpp"

template <typename T> class DDSPublisher {
public:
  using Ptr = std::shared_ptr<DDSPublisher<T>>;
  DDSPublisher(uint32_t domain_id, std::string topic_name, 
               const dds::topic::qos::TopicQos& tqos = dds::topic::qos::TopicQos(),
               const dds::pub::qos::PublisherQos& pqos = dds::pub::qos::PublisherQos() ,
               const dds::pub::qos::DataWriterQos &wqos = dds::pub::qos::DataWriterQos()){
    participant_ =
        std::make_shared<dds::domain::DomainParticipant>(domain_id);
    // Create topic
    topic_ = std::make_shared<typename dds::topic::Topic<T>>(
        *participant_, topic_name, tqos);

    //  Create publisher and writer
    publisher_ = std::make_shared<dds::pub::Publisher>(*participant_, pqos);

    writer_ =
        std::make_shared<typename dds::pub::DataWriter<T>>(
            *publisher_, *topic_, wqos);
  }

  std::shared_ptr<dds::pub::DataWriter<T>> get_writer(){
    return writer_;
  }

private:
  std::shared_ptr<dds::pub::Publisher> publisher_;
  std::shared_ptr<dds::pub::DataWriter<T>> writer_;

  std::shared_ptr<dds::domain::DomainParticipant> participant_;
  std::shared_ptr<dds::topic::Topic<T>> topic_;
};
