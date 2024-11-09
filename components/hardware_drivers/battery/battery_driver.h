#pragma once

#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include "hardware_driver.h"

class BatteryDriver : public HardwareDriver
{
  public:
    BatteryDriver();
    void init();

  private:
    adc_oneshot_unit_handle_t adc_handle_;
    adc_cali_handle_t adc_cali_chan_handle_;

    QueueHandle_t battery_data_publish_queue_;
    static void publish_timer_callback_(void *arg);
    esp_timer_handle_t publish_timer_;
};
