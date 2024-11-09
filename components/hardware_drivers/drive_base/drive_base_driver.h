#pragma once

#include "hardware_driver.h"
#include "motor_driver.h"

class DriveBaseDriver : public HardwareDriver
{
  public:
    DriveBaseDriver();
    void init();
    void set_enabled(bool enabled);

  private:
    Motor left_motor_;
    Motor right_motor_;

    bool is_enabled_;
};
