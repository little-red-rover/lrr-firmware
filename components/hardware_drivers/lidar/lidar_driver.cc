#include "lidar_driver.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "hal/uart_types.h"
#include "messages.pb.h"
#include "socket_manager.h"

#define LIDAR_PWM GPIO_NUM_47

#define LIDAR_TXD GPIO_NUM_17
#define LIDAR_RXD GPIO_NUM_18
#define LIDAR_RTS (UART_PIN_NO_CHANGE)
#define LIDAR_CTS (UART_PIN_NO_CHANGE)

#define LIDAR_UART_PORT_NUM UART_NUM_1
#define LIDAR_UART_BAUD_RATE (230400)
#define LIDAR_TASK_STACK_SIZE (4098)
#define BUF_SIZE 2048

#define TAG = "lidar driver";

#define POINT_PER_UART_PACKET 12
#define UART_PACKETS_PER_UDP_PACKET 10
#define HEADER 0x54
#define VERLEN 0x2C

LidarDriver::LidarDriver() {}

void LidarDriver::init()
{
    // Highest scanning frequency
    gpio_set_direction(LIDAR_PWM, GPIO_MODE_OUTPUT);
    gpio_set_level(LIDAR_PWM, 1);

    // Init UART
    uart_config_t uart_config = {
        .baud_rate = LIDAR_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

    ESP_ERROR_CHECK(uart_driver_install(
      LIDAR_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));

    ESP_ERROR_CHECK(uart_param_config(LIDAR_UART_PORT_NUM, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(
      LIDAR_UART_PORT_NUM, LIDAR_TXD, LIDAR_RXD, LIDAR_RTS, LIDAR_CTS));
}
