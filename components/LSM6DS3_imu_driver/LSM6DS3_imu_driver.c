// https://content.arduino.cc/assets/st_imu_lsm6ds3_datasheet.pdf
// https://github.com/nopnop2002/esp-idf-lsm6ds3/blob/main/components/LSM6DS3/LSM6DS3.cpp

#include "LSM6DS3_imu_driver.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"

#include "messages.pb.h"
#include "socket_mgr.h"

#include "status_led_driver.h"

#define IMU_TASK_STACK_SIZE (4096)

#define IMU_PUBLISH_RATE_HZ 100

#define SCL_PIN 1
#define SDA_PIN 2

#define GRAV_ACCEL 9.80665
#define deg_2_rad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)

static const char *TAG = "imu driver";

i2c_master_dev_handle_t imu_i2c_handle;

UdpPacket imu_msg;

void read_registers(uint8_t address, uint8_t *data, size_t length)
{
    ESP_ERROR_CHECK(i2c_master_transmit_receive(
      imu_i2c_handle, &address, 1, data, length, -1));
}

uint8_t read_register(uint8_t address)
{
    uint8_t read;
    read_registers(address, &read, 1);
    return read;
}

void write_register(uint8_t address, uint8_t value)
{
    uint8_t buff[] = { address, value };

    ESP_ERROR_CHECK(i2c_master_transmit(imu_i2c_handle, buff, 2, -1));
}

void write_registers(uint8_t address, uint8_t *values, size_t length)
{
    uint8_t buff[] = { address };

    ESP_ERROR_CHECK(i2c_master_transmit(imu_i2c_handle, buff, 2, -1));
    ESP_ERROR_CHECK(i2c_master_transmit(imu_i2c_handle, values, length, -1));
}

int read_acceleration(float *x, float *y, float *z)
{
    int16_t data[3];

    read_registers(LSM6DS3_OUTX_L_XL, (uint8_t *)data, sizeof(data));

    *x = -(data[1] * 4.0 / 32768.0) * GRAV_ACCEL;
    *y = -(data[0] * 4.0 / 32768.0) * GRAV_ACCEL;
    *z = -(data[2] * 4.0 / 32768.0) * GRAV_ACCEL;

    return 1;
}

int acceleration_available()
{
    if (read_register(LSM6DS3_STATUS_REG) & 0x01) {
        return 1;
    }

    return 0;
}

float acceleration_sample_rate()
{
    return 104.0F;
}

int read_gyroscope(float *x, float *y, float *z)
{
    int16_t data[3];

    read_registers(LSM6DS3_OUTX_L_G, (uint8_t *)data, sizeof(data));

    *x = -deg_2_rad(data[1] * 500.0 / 32768.0);
    *y = -deg_2_rad(data[0] * 500.0 / 32768.0);
    *z = -deg_2_rad(data[2] * 500.0 / 32768.0);

    return 1;
}

int gyroscope_available()
{
    if (read_register(LSM6DS3_STATUS_REG) & 0x02) {
        return 1;
    }

    return 0;
}

float gyroscope_sample_rate()
{
    return 104.0F;
}

static void imu_driver_task(void *arg)
{
    imu_msg.has_imu = true;
    imu_msg.imu.has_time = true;

    while (1) {
        vTaskDelay((1000 / IMU_PUBLISH_RATE_HZ) / portTICK_PERIOD_MS);
        if (gyroscope_available()) {
            read_gyroscope(
              &imu_msg.imu.gyro_x, &imu_msg.imu.gyro_y, &imu_msg.imu.gyro_z);
            // ESP_LOGI(TAG, "Gyro reading: \n %f \n %f \n %f", x, y, z);
        }
        if (acceleration_available()) {
            read_acceleration(
              &imu_msg.imu.accel_x, &imu_msg.imu.accel_y, &imu_msg.imu.accel_z);
            // ESP_LOGI(TAG, "Accelerometer reading: \n %f \n %f \n %f", x, y,
            // z);
        }

        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        imu_msg.imu.time.sec = (int32_t)ts.tv_sec;
        imu_msg.imu.time.nanosec = (uint32_t)ts.tv_nsec;

        if (tx_queue != NULL &&
            xQueueSend(tx_queue, (void *)&imu_msg, 10) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to push scan onto queue");
        }
    }
    vTaskDelete(NULL);

    write_register(LSM6DS3_CTRL2_G, 0x00);
    write_register(LSM6DS3_CTRL1_XL, 0x00);
}

void LSM6DS3_imu_driver_init()
{
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = SCL_PIN,
        .sda_io_num = SDA_PIN,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    i2c_device_config_t imu_i2c_conf = { .scl_speed_hz = 400000,
                                         .device_address = LSM6DS3_ADDRESS };

    ESP_ERROR_CHECK(
      i2c_master_bus_add_device(bus_handle, &imu_i2c_conf, &imu_i2c_handle));

    if (read_register(LSM6DS3_WHO_AM_I_REG) != 0x69 &&
        read_register(LSM6DS3_WHO_AM_I_REG) != 0x6C) {
        set_status(eImuInitFailed);
    }

    ESP_LOGI(TAG, "IMU WHO_AM_I: %d", (int)read_register(LSM6DS3_WHO_AM_I_REG));

    // Set the Accelerometer control register to work at 104 Hz, 2 g, and in
    // bypass mode and enable ODR / 4 low pass filter (check figure 9 of
    // LSM6DS3's datasheet)
    write_register(LSM6DS3_CTRL1_XL, 0x42);

    // set the gyroscope control register to work at 104 Hz, 500 dps and in
    // bypass mode
    write_register(LSM6DS3_CTRL2_G, 0x44);

    // set gyroscope power mode to high performance and bandwidth to 16 MHz
    write_register(LSM6DS3_CTRL7_G, 0x00);

    // Set the ODR config register to ODR/4
    write_register(LSM6DS3_CTRL8_XL, 0x09);

    xTaskCreate(
      imu_driver_task, "imu_driver_task", IMU_TASK_STACK_SIZE, NULL, 10, NULL);
}
