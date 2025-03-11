#pragma once

#include "callback_worker.hpp"

class DataWriter: public CallbackWorker
{
public:
  DataWriter();

  virtual void run() override;

private:
  void set_callbacks();



};
