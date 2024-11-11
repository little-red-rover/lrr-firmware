#include "status_led_driver.h"

#include "esp_log.h"
#include "neopixel.h"

#define TAG "status_led_driver"

#define BRIGHTNESS 10

void StatusLedDriver::set_status(enum eStatus status)
{
    tNeopixel set;
    switch (status) {
        case eWifiDisconnected:
            set = { 2, NP_RGB(BRIGHTNESS, 0, 0) };
            break;
        case eWifiProvisioning:
            set = { 2, NP_RGB(BRIGHTNESS, BRIGHTNESS, 0) };
            break;
        case eWifiConnected:
            set = { 2, NP_RGB(0, BRIGHTNESS, 0) };
            break;
        case eAgentDisconnected:
            set = { 1, NP_RGB(0, BRIGHTNESS, BRIGHTNESS) };
            break;
        case eAgentConnected:
            set = { 1, NP_RGB(0, BRIGHTNESS, 0) };
            break;
        case eSystemError:
            set = { 0, NP_RGB(BRIGHTNESS, 0, 0) };
            break;
        case eImuInitFailed:
            set = { 0, NP_RGB(BRIGHTNESS, 0, BRIGHTNESS) };
            break;
        case eLidarInitFailed:
            set = { 0, NP_RGB(0, BRIGHTNESS, BRIGHTNESS) };
            break;
        case eDriveBaseInitFailed:
            set = { 0, NP_RGB(BRIGHTNESS, BRIGHTNESS, 0) };
            break;
        case eSystemGood:
            set = { 0, NP_RGB(0, BRIGHTNESS, 0) };
            break;
    }
    neopixel_SetPixel(neopixels_, &set, 1);
}

tNeopixelContext StatusLedDriver::neopixels_ = neopixel_Init(3, 10);

StatusLedDriver::StatusLedDriver() {}

void StatusLedDriver::init()
{
    tNeopixel init_color[3] = {
        { 0, NP_RGB(0, 0, 0) },
        { 1, NP_RGB(0, 0, 0) },
        { 2, NP_RGB(0, 0, 0) },
    };
    ESP_LOGI(TAG, "Status LEDs initialized");
}
