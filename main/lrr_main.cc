#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "drive_base_driver.h"

#include "wifi_manager.h"

extern "C" void app_main(void)
{
    HardwareDriver *drivers[] = { new DriveBaseDriver() };

    for (HardwareDriver *&driver : drivers) {
        driver->init();
    }

    WifiManager::init();

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
