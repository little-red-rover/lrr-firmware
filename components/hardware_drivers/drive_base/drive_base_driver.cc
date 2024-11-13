#include "drive_base_driver.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "hal/ledc_types.h"
#include "messages.pb.h"
#include "socket_manager.h"

#include <cstdio>
#include <ctime>

#define MOTOR_ENABLE_PIN GPIO_NUM_5

#define LEFT_MOTOR_PWM_A_PIN GPIO_NUM_7
#define LEFT_MOTOR_PWM_A_CHANNEL LEDC_CHANNEL_0
#define LEFT_MOTOR_PWM_B_PIN GPIO_NUM_6
#define LEFT_MOTOR_PWM_B_CHANNEL LEDC_CHANNEL_1
#define RIGHT_MOTOR_PWM_A_PIN GPIO_NUM_9
#define RIGHT_MOTOR_PWM_A_CHANNEL LEDC_CHANNEL_2
#define RIGHT_MOTOR_PWM_B_PIN GPIO_NUM_8
#define RIGHT_MOTOR_PWM_B_CHANNEL LEDC_CHANNEL_3

#define LEFT_ENCODER_PIN_A GPIO_NUM_13
#define LEFT_ENCODER_PIN_B GPIO_NUM_14
#define RIGHT_ENCODER_PIN_A GPIO_NUM_3
#define RIGHT_ENCODER_PIN_B GPIO_NUM_4

#define WHEEL_DIAMETER 0.060960 // m
#define WHEEL_TRACK 0.13948     // m

#define PUBLISH_LOOP_PERIOD_MS 50.0

#define TAG "DriveBaseDriver"

void DriveBaseDriver::publish_timer_callback_(void *arg)
{
    DriveBaseDriver *driver = (DriveBaseDriver *)arg;
    if (!xQueueIsQueueFullFromISR(driver->joint_state_publish_queue_)) {
        OutgoingData msg = OutgoingData_init_default;

        msg.has_joint_state = true;
        msg.joint_state.has_time = true;
        msg.msg_id = OutgoingMessageID_JOINT_STATES_DATA;

        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        msg.joint_state.time.sec = (int32_t)ts.tv_sec;
        msg.joint_state.time.nanosec = (uint32_t)ts.tv_nsec;

        msg.joint_state.right_effort = driver->right_motor_.applied_effort_;
        msg.joint_state.right_velocity =
          driver->right_motor_.encoder_.velocity_;
        msg.joint_state.right_position =
          driver->right_motor_.encoder_.position_;

        msg.joint_state.left_effort = driver->left_motor_.applied_effort_;
        msg.joint_state.left_velocity = driver->left_motor_.encoder_.velocity_;
        msg.joint_state.left_position = driver->left_motor_.encoder_.position_;

        BaseType_t xHigherPriorityTaskWoken;
        xHigherPriorityTaskWoken = pdFALSE;

        xQueueSendToBackFromISR(driver->joint_state_publish_queue_,
                                static_cast<void *>(&msg),
                                &xHigherPriorityTaskWoken);

        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

void DriveBaseDriver::task_main_(void *arg)
{
    DriveBaseDriver *driver = (DriveBaseDriver *)arg;
    IncomingCommand cmd = IncomingCommand_init_default;
    while (xQueueReceive(driver->joint_cmd_recv_queue_, &cmd, portMAX_DELAY)) {
        assert(cmd.has_joint_cmd);
        driver->left_motor_.set_velocity(cmd.joint_cmd.left_vel);
        driver->right_motor_.set_velocity(cmd.joint_cmd.right_vel);
    }
}

DriveBaseDriver::DriveBaseDriver()
  : left_motor_(Motor(LEFT_MOTOR_PWM_A_PIN,
                      LEFT_MOTOR_PWM_A_CHANNEL,
                      LEFT_MOTOR_PWM_B_PIN,
                      LEFT_MOTOR_PWM_B_CHANNEL,
                      LEFT_ENCODER_PIN_A,
                      LEFT_ENCODER_PIN_B,
                      false))
  , right_motor_(Motor(RIGHT_MOTOR_PWM_A_PIN,
                       RIGHT_MOTOR_PWM_A_CHANNEL,
                       RIGHT_MOTOR_PWM_B_PIN,
                       RIGHT_MOTOR_PWM_B_CHANNEL,
                       RIGHT_ENCODER_PIN_A,
                       RIGHT_ENCODER_PIN_B,
                       true)) {

  };

void DriveBaseDriver::init()
{
    right_motor_.set_velocity(0.0);

    left_motor_.set_velocity(0.0);

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
                            "drive_base_driver_task",
                            4096,
                            static_cast<void *>(this),
                            5,
                            NULL,
                            tskNO_AFFINITY);

    set_enabled(true);

    ESP_LOGI(TAG, "Drive base initialized");
}

void DriveBaseDriver::set_enabled(bool enabled)
{
    is_enabled_ = enabled;

    gpio_set_direction(MOTOR_ENABLE_PIN, GPIO_MODE_OUTPUT);

    if (enabled) {
        gpio_set_level(MOTOR_ENABLE_PIN, 1);
    } else {
        gpio_set_level(MOTOR_ENABLE_PIN, 0);
    }
}
