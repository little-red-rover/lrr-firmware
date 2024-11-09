#pragma once

#include "driver/i2c_types.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"

#include "hardware_driver.h"

class IMUDriver : public HardwareDriver
{
  public:
    IMUDriver();
    void init();

  private:
    void read_registers_(uint8_t address, uint8_t *data, size_t length);
    uint8_t read_register_(uint8_t address);

    void write_register_(uint8_t address, uint8_t value);
    void write_registers_(uint8_t address, uint8_t *values, size_t length);

    int read_acceleration_(float *x, float *y, float *z);
    int acceleration_available_();
    float acceleration_sample_rate_();

    int read_gyroscope_(float *x, float *y, float *z);
    int gyroscope_available_();
    float gyroscope_sample_rate_();

    QueueHandle_t imu_data_publish_queue_;
    static void publish_timer_callback_(void *arg);
    esp_timer_handle_t publish_timer_;

    i2c_master_dev_handle_t imu_i2c_handle_;

    static void task_main_(void *arg);
};
