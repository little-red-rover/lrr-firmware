#pragma once

#include "hardware_driver.h"
#include <cstdint>

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
};
