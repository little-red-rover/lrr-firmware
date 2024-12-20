#pragma once

#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "hal/ledc_types.h"
#include "pid_ctrl.h"
#include "soc/gpio_num.h"

class Encoder
{
  public:
    Encoder(gpio_num_t a, gpio_num_t b);

    void update(float dt);

  private:
    pcnt_channel_handle_t chan_a_;
    pcnt_channel_handle_t chan_b_;
    pcnt_unit_handle_t unit_;
    int count_;

    float velocity_;
    float position_;

    friend class Motor;
    friend class DriveBaseDriver;
};

class Motor
{
  public:
    Motor(gpio_num_t pwm_a,
          ledc_channel_t chan_a,
          gpio_num_t pwm_b,
          ledc_channel_t chan_b,
          gpio_num_t encoder_a,
          gpio_num_t encoder_b,
          bool reversed);

    void set_velocity(float speed);

  private:
    Encoder encoder_;

    void set_effort_(float power);

    static void pid_timer_callback_(void *arg);
    pid_ctrl_block_handle_t pid_controller_;
    esp_timer_handle_t pid_timer_;

    ledc_channel_t chan_a_;
    ledc_channel_t chan_b_;

    bool reversed_;

    float cmd_velocity_; // rad / s
    float cmd_position_; // rad
    float cmd_effort_;

    float applied_effort_;

    friend class DriveBaseDriver;
};
