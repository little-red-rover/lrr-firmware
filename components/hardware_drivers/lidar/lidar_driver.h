#pragma once

#include "freertos/idf_additions.h"
#include "hardware_driver.h"
#include <cstdint>

#include "messages.pb.h"

#define POINT_PER_UART_PACKET 12

class LidarDriver : public HardwareDriver
{
  public:
    LidarDriver();
    void init();

  private:
    typedef struct
    {
        uint16_t distance;
        uint8_t intensity;
    } __attribute__((packed)) LidarPoint;

    typedef struct
    {
        uint8_t header;
        uint8_t ver_len;
        uint16_t speed;
        uint16_t start_angle;
        LidarPoint points[POINT_PER_UART_PACKET];
        uint16_t end_angle;
        uint16_t timestamp;
        uint8_t crc8;
    } __attribute__((packed)) LiDARFrame;

    QueueHandle_t lidar_data_publish_queue_;
    void publish_message_(OutgoingData msg);

    static void add_scan_to_message_(const LidarDriver::LiDARFrame &frame,
                                     OutgoingData &msg);

    static void task_main_(void *arg);
};
