#include "drive_base_driver.h"
#include "esp_log.h"
#include "hal/ledc_types.h"
#include "messages.pb.h"
#include "socket_manager.h"

#include <cstdio>

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

#define TAG "DriveBaseDriver"

DriveBaseDriver::DriveBaseDriver()
  : left_motor_(Motor(Joint_LEFT_WHEEL,
                      LEFT_MOTOR_PWM_A_PIN,
                      LEFT_MOTOR_PWM_A_CHANNEL,
                      LEFT_MOTOR_PWM_B_PIN,
                      LEFT_MOTOR_PWM_B_CHANNEL,
                      LEFT_ENCODER_PIN_A,
                      LEFT_ENCODER_PIN_B,
                      MOTOR_ENABLE_PIN,
                      false))
  , right_motor_(Motor(Joint_RIGHT_WHEEL,
                       RIGHT_MOTOR_PWM_A_PIN,
                       RIGHT_MOTOR_PWM_A_CHANNEL,
                       RIGHT_MOTOR_PWM_B_PIN,
                       RIGHT_MOTOR_PWM_B_CHANNEL,
                       RIGHT_ENCODER_PIN_A,
                       RIGHT_ENCODER_PIN_B,
                       MOTOR_ENABLE_PIN,
                       true)) {};

void DriveBaseDriver::init()
{
    right_motor_.set_velocity(0.0);
    right_motor_.set_enabled(true);

    left_motor_.set_velocity(0.0);
    left_motor_.set_enabled(true);

    SocketManager::register_data_consumer(IncomingMessageID_JOINT_CMD);

    ESP_LOGI(TAG, "Drive base initialized");
}
