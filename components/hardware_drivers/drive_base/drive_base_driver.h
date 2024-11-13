#pragma once

#include "freertos/idf_additions.h"
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

    QueueHandle_t joint_state_publish_queue_;
    static void publish_timer_callback_(void *arg);
    esp_timer_handle_t publish_timer_;

    QueueHandle_t joint_cmd_recv_queue_;

    static void task_main_(void *arg);
};
