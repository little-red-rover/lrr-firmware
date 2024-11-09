/*
 * Motor control rountines for LRR.
 * Closed loop velocity control for TT Motors with an encoder.
 *
 * References
 *
 * PWM:
 * https://github.com/espressif/esp-idf/blob/master/examples/peripherals/ledc/ledc_basic/main/ledc_basic_example_main.c
 *
 * PID CONTROL:
 * https://github.com/espressif/esp-idf/blob/master/examples/peripherals/mcpwm/mcpwm_bdc_speed_control/main/mcpwm_bdc_control_example_main.c
 *
 * PULSE COUNTER:
 * https://github.com/espressif/esp-idf/blob/master/examples/peripherals/pcnt/rotary_encoder/main/rotary_encoder_example_main.c
 */

#include "motor_driver.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"

#include "freertos/idf_additions.h"
#include "freertos/task.h"

#include "hal/gpio_types.h"
#include "hal/pcnt_types.h"

#include "pid_ctrl.h"
#include <assert.h>
#include <cstdio>
#include <ctime>
#include <math.h>

#include "messages.pb.h"
#include "portmacro.h"
#include "socket_manager.h"
#include "xtensa_context.h"

#define PWM_TIMER_RESOLUTION LEDC_TIMER_10_BIT

// Anything above audible is fine
#define PWM_FREQ_HZ 25000

// Max change to motor power per pid cycle
// Reduces current surges
#define MAX_JERK 0.15

// Minimum % duty that must be applied to affect any motion
#define HYSTERESIS 0.45

#define PID_LOOP_PERIOD_MS 20.0

#define PULSES_PER_ROTATION 2340.0
#define PULSES_TO_RAD(pulses)                                                  \
    (((float)pulses / PULSES_PER_ROTATION) * (2 * M_PI))

#define PUBLISH_LOOP_PERIOD_MS 100.0

double clamp(float d, float min, float max)
{
    const float t = d < min ? min : d;
    return t > max ? max : t;
}

Encoder::Encoder(gpio_num_t a, gpio_num_t b)
  : count_(0)
  , velocity_(0.0)
  , position_(0.0)
{
    pcnt_unit_config_t unit_config = {
        .low_limit = INT16_MIN,
        .high_limit = INT16_MAX,
        .intr_priority = 0,
        .flags = { .accum_count = 1 },
    };

    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &unit_));

    pcnt_glitch_filter_config_t filter_config = { .max_glitch_ns = 1000 };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit_, &filter_config));

    pcnt_chan_config_t chan_config_a = { .edge_gpio_num = a,
                                         .level_gpio_num = b,
                                         .flags = {} };
    ESP_ERROR_CHECK(pcnt_new_channel(unit_, &chan_config_a, &chan_a_));

    pcnt_chan_config_t chan_config_b = { .edge_gpio_num = b,
                                         .level_gpio_num = a,
                                         .flags = {} };
    ESP_ERROR_CHECK(pcnt_new_channel(unit_, &chan_config_b, &chan_b_));

    ESP_ERROR_CHECK(
      pcnt_channel_set_edge_action(chan_a_,
                                   PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                   PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(
      pcnt_channel_set_level_action(chan_a_,
                                    PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                    PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(
      pcnt_channel_set_edge_action(chan_b_,
                                   PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                   PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(
      pcnt_channel_set_level_action(chan_b_,
                                    PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                    PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit_, INT16_MAX));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit_, INT16_MIN));

    ESP_ERROR_CHECK(pcnt_unit_enable(unit_));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit_));
    ESP_ERROR_CHECK(pcnt_unit_start(unit_));
}

void Encoder::update(float dt)
{
    int new_count;
    pcnt_unit_get_count(unit_, &new_count);

    int pulses_elapsed = new_count - count_;
    count_ = new_count;
    velocity_ = PULSES_TO_RAD(pulses_elapsed) * dt;
    position_ = PULSES_TO_RAD(count_);
}

void Motor::set_effort_(float power)
{
    power = clamp(power, -1.0, 1.0);
    power *= 1.0 - HYSTERESIS;
    applied_effort_ =
      clamp(power, applied_effort_ - MAX_JERK, applied_effort_ + MAX_JERK);
    if (applied_effort_ > 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE,
                      chan_b_,
                      (uint32_t)((applied_effort_ + HYSTERESIS) *
                                 (float)(1 << PWM_TIMER_RESOLUTION)));
        ledc_set_duty(LEDC_LOW_SPEED_MODE, chan_a_, 0);
    } else if (applied_effort_ < 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, chan_b_, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE,
                      chan_a_,
                      (uint32_t)((-applied_effort_ + HYSTERESIS) *
                                 (float)(1 << PWM_TIMER_RESOLUTION)));
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, chan_b_, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, chan_a_, 0);
    }

    ledc_update_duty(LEDC_LOW_SPEED_MODE, chan_a_);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, chan_b_);
}

void Motor::set_velocity(float velocity)
{
    if (reversed_) {
        velocity *= -1;
    }
    cmd_velocity_ = velocity;
}

void Motor::pid_timer_callback_(void *arg)
{
    Motor *motor = (Motor *)arg;

    motor->encoder_.update((1000.0 / PID_LOOP_PERIOD_MS));
    float error = motor->cmd_velocity_ - motor->encoder_.velocity_;

    ESP_ERROR_CHECK(
      pid_compute(motor->pid_controller_, error, &motor->cmd_effort_));

    motor->set_effort_(motor->cmd_effort_);
};

static void configure_pwm(ledc_channel_t channel, int gpio)
{
    ledc_channel_config_t pwm_channel = { .gpio_num = gpio,
                                          .speed_mode = LEDC_LOW_SPEED_MODE,
                                          .channel = channel,
                                          .intr_type = LEDC_INTR_DISABLE,
                                          .timer_sel = LEDC_TIMER_0,
                                          .duty = 0,
                                          .hpoint = 0,
                                          .flags = {} };

    ESP_ERROR_CHECK(ledc_channel_config(&pwm_channel));
}

void Motor::publish_timer_callback_(void *arg)
{
    Motor *motor = (Motor *)arg;
    if (!xQueueIsQueueFullFromISR(motor->joint_state_publish_queue_)) {
        OutgoingData msg = OutgoingData_init_default;
        msg.has_joint_state = true;
        msg.joint_state.has_time = true;
        msg.msg_id = OutgoingMessageID_JOINT_STATES_DATA;

        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        msg.joint_state.time.sec = (int32_t)ts.tv_sec;
        msg.joint_state.time.nanosec = (uint32_t)ts.tv_nsec;

        msg.joint_state.joint = motor->joint_name_;
        msg.joint_state.effort = motor->applied_effort_;
        msg.joint_state.velocity = motor->encoder_.velocity_;
        msg.joint_state.position = motor->encoder_.position_;

        BaseType_t xHigherPriorityTaskWoken;
        xHigherPriorityTaskWoken = pdFALSE;

        xQueueSendToBackFromISR(motor->joint_state_publish_queue_,
                                static_cast<void *>(&msg),
                                &xHigherPriorityTaskWoken);

        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

void Motor::task_main_(void *arg)
{
    Motor *motor = (Motor *)arg;
    IncomingCommand cmd = IncomingCommand_init_default;
    while (xQueueReceive(motor->joint_cmd_recv_queue_, &cmd, portMAX_DELAY)) {
        assert(cmd.has_joint_cmd);
        if (cmd.joint_cmd.joint != motor->joint_name_) {
            continue;
        }
        motor->set_velocity(cmd.joint_cmd.vel);
    }
}

Motor::Motor(Joint joint_name,
             gpio_num_t pwm_a,
             ledc_channel_t chan_a,
             gpio_num_t pwm_b,
             ledc_channel_t chan_b,
             gpio_num_t encoder_a,
             gpio_num_t encoder_b,
             bool reversed)
  : joint_name_(joint_name)
  , encoder_{ Encoder(encoder_a, encoder_b) }
  , chan_a_{ chan_a }
  , chan_b_{ chan_b }
  , reversed_{ reversed }
{
    ledc_timer_config_t pwm_timer = { .speed_mode = LEDC_LOW_SPEED_MODE,
                                      .duty_resolution = PWM_TIMER_RESOLUTION,
                                      .timer_num = LEDC_TIMER_0,
                                      .freq_hz = PWM_FREQ_HZ,
                                      .clk_cfg = LEDC_AUTO_CLK,
                                      .deconfigure = false };

    ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));

    configure_pwm(chan_a, pwm_a);
    configure_pwm(chan_b, pwm_b);

    pid_ctrl_parameter_t pid_runtime_param = {
        .kp = 0.05,
        .ki = 0.01,
        .kd = 0.0,
        .max_output = 1.0,
        .min_output = -1.0,
        .max_integral = 0.05,
        .min_integral = -0.05,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
    };

    pid_ctrl_block_handle_t pid_ctrl = NULL;
    pid_ctrl_config_t pid_config = {
        .init_param = pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid_ctrl));
    pid_controller_ = pid_ctrl;

    esp_timer_create_args_t pid_cb_args_ = { .callback = pid_timer_callback_,
                                             .arg = this,
                                             .dispatch_method = ESP_TIMER_TASK,
                                             .name = "PID_timer",
                                             .skip_unhandled_events = true };

    pid_timer_ = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&pid_cb_args_, &pid_timer_));
    esp_timer_start_periodic(pid_timer_, PID_LOOP_PERIOD_MS * 1000);

    joint_state_publish_queue_ = SocketManager::register_data_producer(
      OutgoingMessageID_JOINT_STATES_DATA);

    joint_cmd_recv_queue_ =
      SocketManager::register_data_consumer(IncomingMessageID_JOINT_CMD);

    esp_timer_create_args_t publish_cb_args_ = {
        .callback = publish_timer_callback_,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "publish_timer",
        .skip_unhandled_events = true
    };

    publish_timer_ = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&publish_cb_args_, &publish_timer_));
    esp_timer_start_periodic(publish_timer_, PUBLISH_LOOP_PERIOD_MS * 1000);

    xTaskCreatePinnedToCore(task_main_,
                            "motor_task",
                            4096,
                            static_cast<void *>(this),
                            5,
                            NULL,
                            tskNO_AFFINITY);
}
