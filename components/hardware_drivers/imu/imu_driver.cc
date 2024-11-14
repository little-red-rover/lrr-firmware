#include "imu_driver.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"

#include "messages.pb.h"
#include "socket_manager.h"
#include "status_led_driver.h"
#include <cmath>
#include <ctime>

#define TAG "imu_driver"

#define PUBLISH_LOOP_PERIOD_MS 25

#define SCL_PIN GPIO_NUM_1
#define SDA_PIN GPIO_NUM_2

#define LSM6DS3_ADDRESS 0b1101011

#define LSM6DS3_WHO_AM_I_REG 0X0F
#define LSM6DS3_CTRL1_XL 0X10
#define LSM6DS3_CTRL2_G 0X11

#define LSM6DS3_STATUS_REG 0X1E

#define LSM6DS3_CTRL6_C 0X15
#define LSM6DS3_CTRL7_G 0X16
#define LSM6DS3_CTRL8_XL 0X17

#define LSM6DS3_OUTX_L_G 0X22
#define LSM6DS3_OUTX_H_G 0X23
#define LSM6DS3_OUTY_L_G 0X24
#define LSM6DS3_OUTY_H_G 0X25
#define LSM6DS3_OUTZ_L_G 0X26
#define LSM6DS3_OUTZ_H_G 0X27

#define LSM6DS3_OUTX_L_XL 0X28
#define LSM6DS3_OUTX_H_XL 0X29
#define LSM6DS3_OUTY_L_XL 0X2A
#define LSM6DS3_OUTY_H_XL 0X2B
#define LSM6DS3_OUTZ_L_XL 0X2C
#define LSM6DS3_OUTZ_H_XL 0X2D

#define GRAV_ACCEL 9.80665
#define deg_2_rad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)

void IMUDriver::read_registers_(uint8_t address, uint8_t *data, size_t length)
{
    ESP_ERROR_CHECK(i2c_master_transmit_receive(
      imu_i2c_handle_, &address, 1, data, length, -1));
}

uint8_t IMUDriver::read_register_(uint8_t address)
{
    uint8_t read;
    read_registers_(address, &read, 1);
    return read;
}

void IMUDriver::write_register_(uint8_t address, uint8_t value)
{
    uint8_t buff[] = { address, value };

    ESP_ERROR_CHECK(i2c_master_transmit(imu_i2c_handle_, buff, 2, -1));
}

void IMUDriver::write_registers_(uint8_t address,
                                 uint8_t *values,
                                 size_t length)
{
    uint8_t buff[] = { address };

    ESP_ERROR_CHECK(i2c_master_transmit(imu_i2c_handle_, buff, 2, -1));
    ESP_ERROR_CHECK(i2c_master_transmit(imu_i2c_handle_, values, length, -1));
}

int IMUDriver::read_acceleration_(float *x, float *y, float *z)
{
    int16_t data[3];

    read_registers_(LSM6DS3_OUTX_L_XL, (uint8_t *)data, sizeof(data));

    *x = (data[1] * 2.0 / 32768.0) * GRAV_ACCEL;
    *y = -(data[0] * 2.0 / 32768.0) * GRAV_ACCEL;
    *z = -(data[2] * 2.0 / 32768.0) * GRAV_ACCEL;

    return 1;
}

int IMUDriver::acceleration_available_()
{
    if (read_register_(LSM6DS3_STATUS_REG) & 0x01) {
        return 1;
    }

    return 0;
}

float IMUDriver::acceleration_sample_rate_()
{
    return 104.0F;
}

int IMUDriver::read_gyroscope_(float *x, float *y, float *z)
{
    int16_t data[3];

    read_registers_(LSM6DS3_OUTX_L_G, (uint8_t *)data, sizeof(data));

    *x = -deg_2_rad(data[1] * 500.0 / 32768.0);
    *y = -deg_2_rad(data[0] * 500.0 / 32768.0);
    *z = -deg_2_rad(data[2] * 500.0 / 32768.0);

    return 1;
}

int IMUDriver::gyroscope_available_()
{
    if (read_register_(LSM6DS3_STATUS_REG) & 0x02) {
        return 1;
    }

    return 0;
}

float IMUDriver::gyroscope_sample_rate_()
{
    return 104.0F;
}

void IMUDriver::publish_timer_callback_(void *arg)
{
    IMUDriver *imu_driver = (IMUDriver *)arg;

    if (!xQueueIsQueueFullFromISR(imu_driver->imu_data_publish_queue_)) {
        OutgoingData msg = OutgoingData_init_default;

        msg.has_imu = true;
        msg.imu.has_time = true;

        BaseType_t xHigherPriorityTaskWoken;
        xHigherPriorityTaskWoken = pdFALSE;

        if (imu_driver->gyroscope_available_()) {
            imu_driver->read_gyroscope_(
              &msg.imu.gyro_x, &msg.imu.gyro_y, &msg.imu.gyro_z);
        }
        if (imu_driver->acceleration_available_()) {
            imu_driver->read_acceleration_(
              &msg.imu.accel_x, &msg.imu.accel_y, &msg.imu.accel_z);
        }

        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        msg.imu.time.sec = (int32_t)ts.tv_sec;
        msg.imu.time.nanosec = (uint32_t)ts.tv_nsec;

        xQueueSendToBackFromISR(imu_driver->imu_data_publish_queue_,
                                static_cast<void *>(&msg),
                                &xHigherPriorityTaskWoken);

        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

IMUDriver::IMUDriver() {}

void IMUDriver::init()
{
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };

    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    i2c_device_config_t imu_i2c_conf = { .device_address = LSM6DS3_ADDRESS,
                                         .scl_speed_hz = 400000 };

    ESP_ERROR_CHECK(
      i2c_master_bus_add_device(bus_handle, &imu_i2c_conf, &imu_i2c_handle_));

    if (read_register_(LSM6DS3_WHO_AM_I_REG) != 0x69 &&
        read_register_(LSM6DS3_WHO_AM_I_REG) != 0x6C) {
        StatusLedDriver::set_status(StatusLedDriver::eImuInitFailed);
        ESP_LOGE(TAG, "Failed to initialize IMU");
    }

    // Set the Accelerometer control register to work at 104 Hz, 2 g, and in
    // bypass mode and enable ODR / 4 low pass filter (check figure 9 of
    // LSM6DS3's datasheet)
    write_register_(LSM6DS3_CTRL1_XL, 0x42);

    // set the gyroscope control register to work at 104 Hz, 500 dps and in
    // bypass mode
    write_register_(LSM6DS3_CTRL2_G, 0x44);

    // set gyroscope power mode to high performance and bandwidth to 16 MHz
    write_register_(LSM6DS3_CTRL7_G, 0x00);

    // Set the ODR config register to ODR/4
    write_register_(LSM6DS3_CTRL8_XL, 0x09);

    imu_data_publish_queue_ =
      SocketManager::register_data_producer(OutgoingMessageID_IMU_DATA);

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

    ESP_LOGI(TAG, "IMU driver initialized");
}
