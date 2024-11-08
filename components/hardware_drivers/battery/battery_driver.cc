#include "battery_driver.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "hal/gpio_types.h"

#include "messages.pb.h"
#include "socket_manager.h"

#define TAG "battery_driver"

#define BATTERY_SENSE_PIN GPIO_NUM_19

BatteryDriver::BatteryDriver() {}

void BatteryDriver::init()
{
    // Highest scanning frequency
    gpio_set_direction(BATTERY_SENSE_PIN, GPIO_MODE_INPUT);

    ESP_LOGI(TAG, "Battery monitor initialized");
}
