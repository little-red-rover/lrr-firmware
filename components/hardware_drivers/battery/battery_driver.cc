#include "battery_driver.h"

#include "driver/adc_types_legacy.h"
#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "hal/adc_types.h"
#include "hal/gpio_types.h"
#include <ctime>

#include "messages.pb.h"
#include "socket_manager.h"

#define TAG "battery_driver"

#define PUBLISH_LOOP_PERIOD_MS 100

// https://docs.espressif.com/projects/esp-idf/en/v5.3.1/esp32s3/api-reference/peripherals/gpio.html
#define BATTERY_SENSE_PIN GPIO_NUM_15
#define BATTERY_SENSE_ADC_UNIT ADC_UNIT_2
#define BATTERY_SENSE_ADC_CHAN ADC_CHANNEL_4

// https://github.com/espressif/esp-idf/blob/master/examples/peripherals/adc/oneshot_read/main/oneshot_read_main.c
static bool example_adc_calibration_init(adc_unit_t unit,
                                         adc_channel_t channel,
                                         adc_atten_t atten,
                                         adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

void BatteryDriver::publish_timer_callback_(void *arg)
{
    BatteryDriver *battery_driver = (BatteryDriver *)arg;
    if (!xQueueIsQueueFullFromISR(
          battery_driver->battery_data_publish_queue_)) {
        OutgoingData msg = OutgoingData_init_default;

        msg.has_battery = true;
        msg.battery.has_time = true;
        msg.msg_id = OutgoingMessageID_BATTERY_DATA;

        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        msg.battery.time.sec = (int32_t)ts.tv_sec;
        msg.battery.time.nanosec = (uint32_t)ts.tv_nsec;

        int voltage;
        adc_oneshot_get_calibrated_result(battery_driver->adc_handle_,
                                          battery_driver->adc_cali_chan_handle_,
                                          BATTERY_SENSE_ADC_CHAN,
                                          &voltage);

        // Convert from mV to V, apply voltage divider
        msg.battery.voltage = ((float)voltage / 1000.0) * (43.0 / 33.0);

        BaseType_t xHigherPriorityTaskWoken;
        xHigherPriorityTaskWoken = pdFALSE;

        xQueueSendToBackFromISR(battery_driver->battery_data_publish_queue_,
                                static_cast<void *>(&msg),
                                &xHigherPriorityTaskWoken);

        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

BatteryDriver::BatteryDriver() {}

void BatteryDriver::init()
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = BATTERY_SENSE_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle_));

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(
      adc_oneshot_config_channel(adc_handle_, BATTERY_SENSE_ADC_CHAN, &config));

    if (!example_adc_calibration_init(BATTERY_SENSE_ADC_UNIT,
                                      BATTERY_SENSE_ADC_CHAN,
                                      ADC_ATTEN_DB_12,
                                      &adc_cali_chan_handle_)) {
        ESP_LOGE(TAG, "ADC calibration failed.");
    }

    esp_timer_create_args_t publish_cb_args_ = {
        .callback = publish_timer_callback_,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "publish_timer",
        .skip_unhandled_events = true
    };

    battery_data_publish_queue_ =
      SocketManager::register_data_producer(OutgoingMessageID_BATTERY_DATA);

    publish_timer_ = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&publish_cb_args_, &publish_timer_));
    esp_timer_start_periodic(publish_timer_, PUBLISH_LOOP_PERIOD_MS * 1000);

    ESP_LOGI(TAG, "Battery monitor initialized");
}
