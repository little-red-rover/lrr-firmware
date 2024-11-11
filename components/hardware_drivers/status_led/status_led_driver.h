#pragma once

#include "hardware_driver.h"
#include "neopixel.h"

class StatusLedDriver : public HardwareDriver
{
  public:
    StatusLedDriver();
    void init();

    enum eStatus
    {
        eWifiDisconnected,
        eWifiProvisioning,
        eWifiConnected,
        eAgentDisconnected,
        eAgentConnected,
        eSystemError,
        eImuInitFailed,
        eLidarInitFailed,
        eDriveBaseInitFailed,
        eSystemGood
    };

    static void set_status(enum eStatus status);

  private:
    static tNeopixelContext neopixels_;
};
