#pragma once

#include "DDSListener.hpp"

template <typename T> class DDSSubscriber {
public:
  DDSSubscriber(
      uint32_t domain_id, std::string topic_name,
      std::shared_ptr<DDSListener<T>> listener,
      const dds::topic::qos::TopicQos &tqos = dds::topic::qos::TopicQos(),
      const dds::sub::qos::SubscriberQos &pqos = dds::sub::qos::SubscriberQos(),
      const dds::sub::qos::DataReaderQos &wqos =
          dds::sub::qos::DataReaderQos()): listener_(listener) {
    participant_ = std::make_shared<dds::domain::DomainParticipant>(domain_id);
    // Create topic
    topic_ = std::make_shared<typename dds::topic::Topic<T>>(*participant_,
                                                             topic_name, tqos);

    //  Create publisher and writer
    publisher_ = std::make_shared<dds::sub::Subscriber>(*participant_, pqos);

    reader_ = std::make_shared<typename dds::sub::DataReader<T>>(*publisher_,
                                                                 *topic_, wqos);

    if (listener_ != nullptr) {
      reader_->listener(listener_.get(),
                        dds::core::status::StatusMask::data_available());
    }
  }

  DDSSubscriber(
      uint32_t domain_id, std::string topic_name,
      const dds::topic::qos::TopicQos &tqos = dds::topic::qos::TopicQos(),
      const dds::sub::qos::SubscriberQos &pqos = dds::sub::qos::SubscriberQos(),
      const dds::sub::qos::DataReaderQos &wqos =
          dds::sub::qos::DataReaderQos(),
      std::function<void(const std::shared_ptr<T> &)> callback = {}
  ) {
    auto listener = std::make_shared<DDSListener<T>>(
        [this, callback](const std::vector<std::shared_ptr<T>> &data) {
          for (auto &msg : data) {
            callback(msg);
          }
        });

    DDSSubscriber(domain_id, topic_name, listener, tqos, pqos, wqos);
  }

  std::shared_ptr<dds::sub::DataReader<T>> get_reader() { return reader_; }

private:
  std::shared_ptr<dds::sub::Subscriber> publisher_;
  std::shared_ptr<dds::sub::DataReader<T>> reader_;

  std::shared_ptr<dds::domain::DomainParticipant> participant_;
  std::shared_ptr<dds::topic::Topic<T>> topic_;
  std::shared_ptr<DDSListener<T>> listener_;
};
