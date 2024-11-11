#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "battery_driver.h"
#include "drive_base_driver.h"
#include "imu_driver.h"
#include "lidar_driver.h"

#include "socket_manager.h"
#include "status_led_driver.h"
#include "wifi_manager.h"

extern "C" void app_main(void)
{
    HardwareDriver *drivers[] = { new DriveBaseDriver(),
                                  new LidarDriver(),
                                  new IMUDriver(),
                                  new BatteryDriver(),
                                  new StatusLedDriver() };

    StatusLedDriver::set_status(StatusLedDriver::eSystemGood);

    for (HardwareDriver *&driver : drivers) {
        driver->init();
    }

    WifiManager::init();

    SocketManager::init();

    // status_led_driver_init();
    //
    // set_status(eSystemGood);
    //
    // LSM6DS3_imu_driver_init();
    //
    // drive_base_driver_init();
    //
    // lidar_driver_init();
    //
    // wifi_mgr_init();
    //
    // socket_mgr_init();
}
