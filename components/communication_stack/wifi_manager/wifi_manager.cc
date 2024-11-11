#include "wifi_manager.h"
#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>

#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_wifi_types.h"
#include "nvs_flash.h"
#include "portmacro.h"

#include <esp_wifi.h>
#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_softap.h>

#include "dhcpserver/dhcpserver.h"

#include "esp_netif.h"
#include "lwip/lwip_napt.h"
#include <lwip/netdb.h>

#include "esp_mac.h"

#include "status_led_driver.h"
#include "wifi_manager.h"

namespace WifiManager {
#define REPROVISION_PIN GPIO_NUM_11

#define WIFI_CONNECTION_TIMEOUT_MS 10000

#define DEFAULT_AP_IP "192.168.4.1"
#define DEFAULT_DNS "8.8.8.8"

#define AP_PASSWORD "littleredrover"

#define TAG "WifiManager"

esp_ip4_addr_t STA_IP;
esp_netif_t *esp_netif_sta;
esp_netif_t *esp_netif_ap;

void wifi_prov_event_handler(void *arg,
                             esp_event_base_t event_base,
                             long int event_id,
                             void *event_data)
{
    if (event_base == WIFI_PROV_EVENT) {
        switch (event_id) {
            case WIFI_PROV_START:
                ESP_LOGI(TAG, "Provisioning started");
                break;
            case WIFI_PROV_CRED_RECV: {
                wifi_sta_config_t *wifi_sta_cfg =
                  (wifi_sta_config_t *)event_data;
                ESP_LOGI(TAG,
                         "Received Wi-Fi credentials"
                         "\n\tSSID     : %s\n\tPassword : %s",
                         (const char *)wifi_sta_cfg->ssid,
                         (const char *)wifi_sta_cfg->password);
                break;
            }
            case WIFI_PROV_CRED_FAIL: {
                wifi_prov_sta_fail_reason_t *reason =
                  (wifi_prov_sta_fail_reason_t *)event_data;
                ESP_LOGE(TAG,
                         "Provisioning failed!\n\tReason : %s"
                         "\n\tPlease reset to factory and retry provisioning",
                         (*reason == WIFI_PROV_STA_AUTH_ERROR)
                           ? "Wi-Fi station authentication failed"
                           : "Wi-Fi access-point not found");
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                wifi_prov_mgr_reset_provisioning();
                // Delay so esp_prov.py has time to read reason for failure.
                esp_restart();
                break;
            }
            case WIFI_PROV_CRED_SUCCESS:
                ESP_LOGI(TAG, "Provisioning successful");
                break;
            case WIFI_PROV_END:
                /* De-initialize manager once provisioning is finished */
                // wifi_prov_mgr_deinit();
                break;
            default:
                break;
        }
    }
}

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static const char *TAG_AP = "WiFi SoftAP";
static const char *TAG_STA = "WiFi Sta";

int s_retry_num = 0;

EventGroupHandle_t s_wifi_event_group;

void wifi_event_handler(void *arg,
                        esp_event_base_t event_base,
                        int32_t event_id,
                        void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event =
          (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG_AP,
                 "Station " MACSTR " joined, AID=%d",
                 MAC2STR(event->mac),
                 event->aid);
    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event =
          (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG_AP, "Disconnect reason: %d", event->reason);
        ESP_LOGI(TAG_AP,
                 "Station " MACSTR " left, AID=%d",
                 MAC2STR(event->mac),
                 event->aid);

    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG_STA, "Station started");
    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_err_reason_t reason =
          (wifi_err_reason_t)(*(wifi_event_sta_disconnected_t *)event_data)
            .reason;
        if (reason == WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT ||
            reason == WIFI_REASON_NO_AP_FOUND ||
            reason == WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY ||
            reason == WIFI_REASON_HANDSHAKE_TIMEOUT ||
            reason == WIFI_REASON_AUTH_EXPIRE ||
            reason == WIFI_REASON_AUTH_FAIL) {
            wifi_prov_mgr_reset_provisioning();
            ESP_LOGI(
              TAG_STA,
              "Authentication failed. Resetting provisioning and restarting.");
            esp_restart();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        STA_IP = event->ip_info.ip;
        ESP_LOGI(TAG_STA, "Got IP:" IPSTR, IP2STR(&STA_IP));
        s_retry_num = 0;
        esp_netif_dns_info_t dns;
        if (esp_netif_get_dns_info(esp_netif_sta, ESP_NETIF_DNS_MAIN, &dns) ==
            ESP_OK) {
            esp_netif_set_dns_info(esp_netif_ap, ESP_NETIF_DNS_MAIN, &dns);
            ESP_LOGI(TAG, "set dns to:" IPSTR, IP2STR(&(dns.ip.u_addr.ip4)));
        }
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_err_t get_ip_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Got request for IP. Sending " IPSTR, IP2STR(&STA_IP));
    char *ip_str = (char *)malloc(50 * sizeof(char));
    snprintf(ip_str, 50, IPSTR, IP2STR(&STA_IP));
    httpd_resp_send(req, ip_str, HTTPD_RESP_USE_STRLEN);
    free(ip_str);
    return ESP_OK;
};

// https://github.com/espressif/esp-idf/issues/4863#issuecomment-594357432
esp_err_t get_remote_ip(httpd_req_t *req, struct sockaddr_in6 *addr_in)
{
    int s = httpd_req_to_sockfd(req);
    socklen_t addrlen = sizeof(*addr_in);
    if (lwip_getpeername(s, (struct sockaddr *)addr_in, &addrlen) != -1) {
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Error getting peer's IP/port");
        return ESP_FAIL;
    }
}

esp_err_t get_agent_ip_handler(httpd_req_t *req)
{
    char *agent_ip;
    struct sockaddr_in6 addr_in;
    if (get_remote_ip(req, &addr_in) == ESP_OK) {
        agent_ip = inet_ntoa(addr_in.sin6_addr.un.u32_addr[3]);
        ESP_LOGI(TAG, "Remote IP is %s", agent_ip);
    } else {
        return ESP_FAIL;
    }

    httpd_resp_sendstr(req, agent_ip);

    return ESP_OK;
}

/* Save the server handle here */
httpd_handle_t _server = NULL;

esp_err_t start_webserver()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    ESP_LOGI(TAG, "Starting HTTP Server");
    ESP_ERROR_CHECK(httpd_start(&_server, &config));

    /* URI handler for getting web server files */
    httpd_uri_t common_get_uri = { .uri = "/get-ip",
                                   .method = HTTP_GET,
                                   .handler = get_ip_handler,
                                   .user_ctx = (void *)"" };
    httpd_uri_t agent_ip_get_uri = { .uri = "/set-agent-ip",
                                     .method = HTTP_GET,
                                     .handler = get_agent_ip_handler,
                                     .user_ctx = (void *)"" };
    httpd_register_uri_handler(_server, &common_get_uri);
    httpd_register_uri_handler(_server, &agent_ip_get_uri);

    return ESP_OK;
}

void init()
{
    StatusLedDriver::set_status(StatusLedDriver::eWifiProvisioning);

    /* NVS INIT */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    /* RETRIEVE MAC */
    unsigned char mac[6] = { 0 };
    esp_efuse_mac_get_default(mac);
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    /* EVENT LOOP INIT */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* WIFI INIT */
    esp_wifi_disconnect();
    ESP_ERROR_CHECK(esp_netif_init());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

    char *ssid;
    asprintf(&ssid, "little_red_rover_%02X:%02X:%02X", mac[3], mac[4], mac[5]);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "",
            .password = AP_PASSWORD,
            .ssid_len = 0,
            .channel = 0,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .ssid_hidden = false,
            .max_connection = 14,
            .pmf_cfg = {
                .capable = true,
                .required = true,
            },
        },
    };
    strcpy((char *)wifi_config.ap.ssid, ssid);
    free(ssid);

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));

    ESP_LOGI("AP", "ESP_WIFI_MODE_AP");
    esp_netif_ap = esp_netif_create_default_wifi_ap();

    ESP_LOGI("STA", "ESP_WIFI_MODE_STA");
    esp_netif_sta = esp_netif_create_default_wifi_sta();

    ESP_ERROR_CHECK(start_webserver());

    // DHCP
    esp_netif_dhcps_start(esp_netif_ap);
    esp_netif_dhcps_start(esp_netif_sta);
    esp_netif_dns_info_t dnsserver;
    dhcps_offer_t dhcps_dns_value = OFFER_DNS;
    esp_netif_dhcps_option(esp_netif_ap,
                           ESP_NETIF_OP_SET,
                           ESP_NETIF_DOMAIN_NAME_SERVER,
                           &dhcps_dns_value,
                           sizeof(dhcps_dns_value));

    // // Set custom dns server address for dhcp server
    dnsserver.ip.u_addr.ip4.addr = esp_ip4addr_aton(DEFAULT_DNS);
    dnsserver.ip.type = ESP_IPADDR_TYPE_V4;
    esp_netif_set_dns_info(esp_netif_ap, ESP_NETIF_DNS_MAIN, &dnsserver);

    /* PROVISIONING INIT */
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_event_handler_register(
      WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &wifi_prov_event_handler, NULL));

    wifi_prov_mgr_config_t config = { .scheme = wifi_prov_scheme_softap,
                                      .scheme_event_handler =
                                        WIFI_PROV_EVENT_HANDLER_NONE };
    ESP_ERROR_CHECK(wifi_prov_mgr_init(config));
    wifi_prov_scheme_softap_set_httpd_handle((void *)(&_server));

    // Reprovision button
    gpio_set_direction(REPROVISION_PIN, GPIO_MODE_INPUT);
    if (!gpio_get_level(REPROVISION_PIN)) {
        wifi_prov_mgr_reset_provisioning();
    }

    ESP_ERROR_CHECK(wifi_prov_mgr_disable_auto_stop(0));

    bool provisioned = false;
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    if (!provisioned) {
        StatusLedDriver::set_status(StatusLedDriver::eWifiProvisioning);
        ESP_ERROR_CHECK(esp_event_handler_register(
          IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

        ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(
          WIFI_PROV_SECURITY_1, NULL, (char *)wifi_config.ap.ssid, NULL));
    } else {
        ESP_LOGI(TAG, "Already provisioned, starting Wi-Fi STA");

        ESP_ERROR_CHECK(esp_event_handler_register(
          WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(
          IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

        ESP_ERROR_CHECK(esp_wifi_start());
    }

    // Load bearing delay
    // This prevents the esp32s3 from boot looping (for some reason, I don't
    // know why)
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Wifi started successfully");
    ip_napt_enable(esp_ip4addr_aton(DEFAULT_AP_IP), 1);
    ESP_LOGI(TAG, "ip_napt enabled");

    // Set up time sync
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();

    /* Wait for Wi-Fi connection */
    TickType_t timeout = WIFI_CONNECTION_TIMEOUT_MS / portTICK_PERIOD_MS;
    if (!provisioned) {
        timeout = portMAX_DELAY;
    }
    EventBits_t wifi_ret = xEventGroupWaitBits(
      s_wifi_event_group, WIFI_CONNECTED_BIT, false, true, timeout);

    if (wifi_ret & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Device is provisioned and connected.");
        StatusLedDriver::set_status(StatusLedDriver::eWifiConnected);
    } else {
        // Error handling is for chumps
        ESP_LOGE(TAG, "Wifi connection timed out. Rebooting...");
        esp_restart();
    }
}
}
