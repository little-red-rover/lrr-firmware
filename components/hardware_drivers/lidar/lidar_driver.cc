#include "lidar_driver.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "hal/uart_types.h"
#include "messages.pb.h"
#include "portmacro.h"
#include "soc/soc.h"
#include "socket_manager.h"
#include "status_led_driver.h"
#include <cmath>
#include <ctime>

#define LIDAR_PWM GPIO_NUM_47

#define LIDAR_TXD GPIO_NUM_17
#define LIDAR_RXD GPIO_NUM_18
#define LIDAR_RTS (UART_PIN_NO_CHANGE)
#define LIDAR_CTS (UART_PIN_NO_CHANGE)

#define LIDAR_UART_PORT_NUM UART_NUM_1
#define LIDAR_UART_BAUD_RATE (230400)
#define LIDAR_TASK_STACK_SIZE (4098)
#define BUF_SIZE 2048

#define TAG "lidar_driver"

#define POINTS_PER_UART_PACKET 12
#define UART_PACKETS_PER_MESSAGE 1
#define HEADER 0x54
#define VERLEN 0x2C

#define deg_2_rad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)

static const uint8_t CrcTable[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
    0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
    0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
    0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
    0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
    0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
    0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
    0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
    0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
    0x7f, 0x32, 0xe5, 0xa8
};

uint8_t CalCRC8(const uint8_t *data, uint16_t data_len)
{
    uint8_t crc = 0;
    while (data_len--) {
        crc = CrcTable[(crc ^ *data) & 0xff];
        data++;
    }
    return crc;
}

void LidarDriver::add_scan_to_message_(const LidarDriver::LiDARFrame &frame,
                                       OutgoingData &msg)
{
    LaserScan scan_msg = LaserScan_init_default;

    // Set time
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    scan_msg.has_time = true;
    scan_msg.time.sec = (int32_t)ts.tv_sec;
    scan_msg.time.nanosec = (uint32_t)ts.tv_nsec;

    // Set message array size bounds (nanoPB specific)
    scan_msg.ranges_count = POINTS_PER_UART_PACKET;
    scan_msg.intensities_count = POINTS_PER_UART_PACKET;
    for (size_t i = 0; i < POINTS_PER_UART_PACKET; i++) {
        scan_msg.ranges[i] = frame.points[i].distance;
        scan_msg.intensities[i] = frame.points[i].intensity;
    }

    // Set angle limits
    scan_msg.start_angle = frame.start_angle;
    scan_msg.end_angle = frame.end_angle;

    scan_msg.speed = frame.speed;

    // Add scan to message
    msg.laser[msg.laser_count] = scan_msg;

    // Increment laser message count
    msg.laser_count++;
}

void LidarDriver::publish_message_(OutgoingData msg)
{
    if (xQueueSendToFront(
          lidar_data_publish_queue_, static_cast<void *>(&msg), 0) != pdTRUE) {
    }
}

void LidarDriver::task_main_(void *arg)
{
    LidarDriver *lidar_driver = (LidarDriver *)arg;

    // Init message structure
    OutgoingData msg = OutgoingData_init_default;
    msg.msg_id = OutgoingMessageID_LIDAR_DATA;
    msg.laser_count = 0;

    uint8_t start_chars[2] = { 0x00, 0x00 };
    LiDARFrame scan_data;

    while (uart_read_bytes(
             LIDAR_UART_PORT_NUM, &start_chars, 1, portMAX_DELAY) > 0) {
        // Look for start bytes, 0x54 0x2C
        if (start_chars[1] != HEADER || start_chars[0] != VERLEN) {
            start_chars[1] = start_chars[0];
            continue;
        }

        uart_read_bytes(LIDAR_UART_PORT_NUM,
                        ((uint8_t *)&scan_data) + 2,
                        sizeof(scan_data) - 2,
                        portMAX_DELAY);

        scan_data.header = HEADER;
        scan_data.ver_len = VERLEN;

        uint8_t checksum = CalCRC8(reinterpret_cast<uint8_t *>(&scan_data),
                                   sizeof(scan_data) - 1);
        lidar_driver->add_scan_to_message_(scan_data, msg);

        // If message is full, send it
        if (msg.laser_count >= UART_PACKETS_PER_MESSAGE) {
            lidar_driver->publish_message_(msg);
            // And reset the count
            msg.laser_count = 0;
        }
    }
    vTaskDelete(NULL);
}

LidarDriver::LidarDriver() {}

void LidarDriver::init()
{
    // Highest scanning frequency
    gpio_set_direction(LIDAR_PWM, GPIO_MODE_OUTPUT);
    gpio_set_level(LIDAR_PWM, 0);

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

    if (uart_driver_install(
          LIDAR_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags) !=
          ESP_OK ||
        uart_param_config(LIDAR_UART_PORT_NUM, &uart_config) != ESP_OK ||
        uart_set_pin(
          LIDAR_UART_PORT_NUM, LIDAR_TXD, LIDAR_RXD, LIDAR_RTS, LIDAR_CTS) !=
          ESP_OK ||
        uart_set_pin(
          LIDAR_UART_PORT_NUM, LIDAR_TXD, LIDAR_RXD, LIDAR_RTS, LIDAR_CTS) !=
          ESP_OK) {
        StatusLedDriver::set_status(StatusLedDriver::eLidarInitFailed);
        ESP_LOGE(TAG, "Failed to initialize LiDAR");
    }

    lidar_data_publish_queue_ =
      SocketManager::register_data_producer(OutgoingMessageID_LIDAR_DATA);

    // Reading from the LiDAR is very resource intensive.
    // Pin it to core 1 with priority 19 so it won't get interrupted
    xTaskCreatePinnedToCore(task_main_,
                            "lidar_task",
                            8192,
                            static_cast<void *>(this),
                            19,
                            NULL,
                            APP_CPU_NUM);

    ESP_LOGI(TAG, "LiDAR driver initialized");
}
