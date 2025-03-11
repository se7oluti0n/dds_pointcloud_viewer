#pragma once

#include <dds/dds.hpp>
#include <functional>

template <typename T>
class DDSListener
    : public dds::sub::NoOpDataReaderListener<T> {
public:
  using Handler = std::function<void(const std::vector<std::shared_ptr<T>> &)>;
  DDSListener(Handler handler) : handler_(handler) {}
  void on_data_available(
      dds::sub::DataReader<T> &reader) override {
    try {

      auto preTakeTime = dds_time();
      dds::sub::LoanedSamples<T> samples =
          reader.take();
      std::vector<std::shared_ptr<T>> data;
      for (typename dds::sub::LoanedSamples<T>::const_iterator
               sample_it = samples.begin();
           sample_it != samples.end(); sample_it++) {
        if (sample_it->info().valid()) {
          data.emplace_back(new T);
          *data.back() = sample_it->data();
        }
      }

      handler_(data);
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
  std::function<void(const std::vector<std::shared_ptr<T>> &)> handler_;
};
