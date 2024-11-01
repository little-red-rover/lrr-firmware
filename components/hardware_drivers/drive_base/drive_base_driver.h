#pragma once

#include "hardware_driver.h"
#include "motor_driver.h"

class DriveBaseDriver : public HardwareDriver
{
  public:
    DriveBaseDriver();
    void init();

  private:
    Motor left_motor_;
    Motor right_motor_;
};
