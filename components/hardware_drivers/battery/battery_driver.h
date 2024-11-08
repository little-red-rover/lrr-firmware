#pragma once

#include "hardware_driver.h"

class BatteryDriver : public HardwareDriver
{
  public:
    BatteryDriver();
    void init();
};
