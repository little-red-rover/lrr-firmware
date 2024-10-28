#include "socket_mgr.h"

#include "freertos/idf_additions.h"
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include "esp_log.h"
#include "freertos/projdefs.h"
#include "lwip/sockets.h"

#include "messages.pb.h"
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "pb_utils.h"
#include "status_led_driver.h"

#define SOCKET_TASK_STACK_SIZE 4096
#define MAX_RX_CALLBACKS 1

#define PORT 8001

#define MAGIC_NUMBER "LRR"

static const char *TAG = "SOCKET_MGR";

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

static int try_receive(const int sock, void *data, size_t max_len)
{
    int len = recv(sock, data, max_len, 0);
    if (len < 0) {
        if (errno == EINPROGRESS || errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0; // Not an error
        }
        if (errno == ENOTCONN) {
            ESP_LOGW(TAG, "Socket connection closed");
            return -2; // Socket has been disconnected
        }
        ESP_LOGE(TAG, "Error occurred during recieving: errno %d", errno);
        return -1;
    }

    return len;
}

static int socket_send(const int sock, const void *data, const size_t len)
{
    int to_write = len;
    while (to_write > 0) {
        int written = send(sock, data + (len - to_write), to_write, 0);
        if (written < 0 && errno != EINPROGRESS && errno != EAGAIN &&
            errno != EWOULDBLOCK) {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            return -1;
        }
        to_write -= written;
    }
    return len;
}

static uint8_t tx_packets(int sock)
{
    NetworkPacket msg;

    while (xQueueReceive(tx_queue, (void *)&msg, 0) == pdTRUE) {
        pb_ostream_t stream =
          pb_ostream_from_buffer(tx_buffer, sizeof(tx_buffer));
        bool status = pb_encode(&stream, NetworkPacket_fields, &msg);
        if (!status) {
            ESP_LOGE(TAG, "Failed to serialize message.");
        }

        socket_send(sock, MAGIC_NUMBER, sizeof(MAGIC_NUMBER) - 1);
        uint16_t length = stream.bytes_written;
        if (stream.bytes_written > UINT16_MAX) {
            ESP_LOGE(TAG, "TX packet too large");
            return -1;
        }
        socket_send(sock, &length, sizeof(length));
        ssize_t sent = socket_send(sock, tx_buffer, stream.bytes_written);

        if (sent < 0) {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            return -1;
        }

        if (sent < stream.bytes_written) {
            ESP_LOGE(TAG,
                     "Failed to write full packet data. Wrote %ld, "
                     "expected %zu.",
                     (long)sent,
                     stream.bytes_written);
        }
    }

    return 0;
}

static char magic[3] = {};
static uint16_t length = 0;
static uint16_t message_recv_length = 0;

static uint8_t rx_packets(int sock)
{
    // wait to see the delimeter
    if (strcmp(magic, MAGIC_NUMBER) != 0) {
        magic[0] = magic[1];
        magic[1] = magic[2];
        try_receive(sock, magic + 2, 1);
        return 0;
    }

    // after the delimeter is two byte size
    if (length == 0) {
        try_receive(sock, &length, 2);
        return 0;
    }

    // finally the actual data
    int recieved = try_receive(sock, rx_buffer, length);
    message_recv_length += recieved;

    if (recieved == 0) {
        // socket was busy
        return 0;
    } else if (recieved == -2) {
        ESP_LOGE(TAG, "Connection closed");
        return -1;
    } else if (recieved < 0) {
        ESP_LOGE(TAG, "recv failed: errno %d", errno);
        return -1;
    } else if (message_recv_length < length) {
        // there's more of the message yet to be recieved
        return 0;
    } else {
        pb_istream_t stream = pb_istream_from_buffer(rx_buffer, length);

        // TODO: Get rid of the union stuff, just pass the NetworkPacket
        // message
        const pb_msgdesc_t *type = decode_unionmessage_type(&stream);
        bool status = false;

        if (type == TwistCmd_fields) {
            TwistCmd msg = {};
            status =
              decode_unionmessage_contents(&stream, TwistCmd_fields, &msg);
            (*(rx_callbacks[eTwistCmd]))(&msg);
        }

        magic[0] = 0;
        magic[1] = 0;
        magic[2] = 0;
        length = 0;
        message_recv_length = 0;

        if (!status) {
            ESP_LOGE(TAG, "Decode failed: %s\n", PB_GET_ERROR(&stream));
        }
        return 0;
    }
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
        return;
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

        int flags = fcntl(sock, F_GETFL);
        if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) == -1) {
            ESP_LOGE(
              TAG, "Unable to set socket non blocking : errno %d", errno);
            vTaskDelete(NULL);
            return;
        }
        ESP_LOGI(TAG, "Socket marked as non blocking");

        while (1) {
            if (rx_packets(sock) != 0)
                break;
            if (tx_packets(sock) != 0)
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

    tx_queue = xQueueCreate(25, sizeof(NetworkPacket));

    xTaskCreatePinnedToCore(socket_task,
                            "socket_task",
                            SOCKET_TASK_STACK_SIZE,
                            NULL,
                            10,
                            NULL,
                            PRO_CPU_NUM);
}
