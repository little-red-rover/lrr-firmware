#include "socket_mgr.h"

#include "freertos/idf_additions.h"
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include "esp_log.h"
#include "freertos/projdefs.h"
#include "lwip/sockets.h"
#include "nvs.h"

#include "messages.pb.h"
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "pb_utils.h"
#include "portmacro.h"
#include "status_led_driver.h"

#define SOCKET_TASK_STACK_SIZE 4096
#define MAX_RX_CALLBACKS 1

#define PORT 8001

static const char *TAG = "SOCKET_MGR";

static char AGENT_IP[16];

struct sockaddr_storage source_addr;

static int listen_socket;

QueueHandle_t tx_queue = NULL;

static unsigned char tx_buffer[2000];

static unsigned char rx_buffer[2000];

static void (*rx_callbacks[MAX_RX_CALLBACKS])(void *);

void register_callback(void (*callback)(void *), eRxMsgTypes type)
{
    rx_callbacks[type] = callback;
}

static uint8_t tx_packets(int sock)
{
    NetworkPacket msg;

    while (xQueueReceive(tx_queue, (void *)&msg, portMAX_DELAY) == pdTRUE) {
        pb_ostream_t stream =
          pb_ostream_from_buffer(tx_buffer, sizeof(tx_buffer));
        bool status = pb_encode(&stream, NetworkPacket_fields, &msg);
        if (!status) {
            ESP_LOGE(TAG, "Failed to serialize message.");
        }

        ssize_t to_write = stream.bytes_written;
        while (to_write > 0) {
            ssize_t sent = send(
              sock, tx_buffer + (stream.bytes_written - to_write), to_write, 0);

            if (sent < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                return -1;
            }
            to_write -= sent;
        }

        if (to_write < 0) {
            ESP_LOGE(TAG,
                     "Failed to write full packet data. Wrote %ld, "
                     "expected %zu.",
                     (long)stream.bytes_written - to_write,
                     stream.bytes_written);
        }
    }

    return 0;
}

static uint8_t rx_packets(int sock)
{
    int len = recv(sock, rx_buffer, sizeof(rx_buffer), 0);

    if (len < 0) {
        ESP_LOGE(TAG, "recv failed: errno %d", errno);
        return -1;
    } else if (len == 0) {
        ESP_LOGE(TAG, "Connection closed");
        return -1;
    } else {
        pb_istream_t stream = pb_istream_from_buffer(rx_buffer, len);

        // TODO: Get rid of the union stuff, just pass the NetworkPacket message
        const pb_msgdesc_t *type = decode_unionmessage_type(&stream);
        bool status = false;

        if (type == TwistCmd_fields) {
            TwistCmd msg = {};
            status =
              decode_unionmessage_contents(&stream, TwistCmd_fields, &msg);
            (*(rx_callbacks[eTwistCmd]))(&msg);
        }

        if (!status) {
            ESP_LOGE(TAG, "Decode failed: %s\n", PB_GET_ERROR(&stream));
        }
    }
    return 0;
}

static void socket_task(void *arg)
{
    char addr_str[128];
    int keepAlive = 1;
    int keepIdle = 5;
    int keepInterval = 5;
    int keepCount = 3;

    struct sockaddr_in src_addr;
    src_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    src_addr.sin_family = AF_INET;
    src_addr.sin_port = htons(PORT);

    listen_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

    if (listen_socket < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
    }

    int opt = 1;
    setsockopt(listen_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    int err =
      bind(listen_socket, (struct sockaddr *)&src_addr, sizeof(src_addr));

    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(listen_socket);
        vTaskDelete(NULL);
    }

    err = listen(listen_socket, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occured during listen: errno %d", errno);
        close(listen_socket);
        vTaskDelete(NULL);
    } else {
        set_status(eAgentConnected);
        ESP_LOGI(TAG, "Socket listening on port %d", PORT);
    }

    while (1) {
        socklen_t addr_len = sizeof(source_addr);
        int sock =
          accept(listen_socket, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(
          sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));

        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr,
                        addr_str,
                        sizeof(addr_str) - 1);
        }
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);
        while (1) {
            if (tx_packets(sock) != 0)
                break;
            if (rx_packets(sock) != 0)
                break;
        }

        shutdown(sock, 0);
        close(sock);
    }
    vTaskDelete(NULL);
}

void socket_mgr_init()
{
    set_status(eAgentDisconnected);

    tx_queue = xQueueCreate(5, sizeof(NetworkPacket));

    xTaskCreatePinnedToCore(socket_task,
                            "socket_task",
                            SOCKET_TASK_STACK_SIZE,
                            NULL,
                            10,
                            NULL,
                            APP_CPU_NUM);
}
