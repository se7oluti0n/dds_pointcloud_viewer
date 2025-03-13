#pragma once

#include "callback_worker.hpp"
#include "Slam3DIDL.hpp"

#include "glim/odometry/estimation_frame.hpp"

#include <fstream>

using namespace glim;

class DataWriter: public CallbackWorker
{
public:
  DataWriter(const std::string& file_name);

  ~DataWriter();

  virtual void run() override;

  void write(const EstimationFrame::ConstPtr& keyframe);


private:
  void set_callbacks();

  std::fstream file_;

};
