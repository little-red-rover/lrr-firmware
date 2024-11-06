#include "socket_manager.h"

#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <sys/select.h>
#include <sys/types.h>
#include <time.h>
#include <vector>

#include "esp_log.h"
#include "lwip/sockets.h"

#include "common.h"
#include "messages.pb.h"
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"

namespace SocketManager {

#define TAG "SocketManager"

#define SOCKET_MAIN_TASK_STACK_SIZE 4096
#define SOCKET_THREAD_STACK_SIZE 4066

#define MAX_CLIENTS 10

#define QUEUE_LENGTH 5

#define PORT 8001

#define MAGIC_NUMBER "LRR"

std::vector<QueueHandle_t>
  data_consumer_queues[static_cast<int>(IncomingMessageID_incoming_msg_count)];

std::vector<int>
  send_sockets[static_cast<int>(OutgoingMessageID_outgoing_msg_count)];

typedef struct
{
    QueueHandle_t queue_handle;
    OutgoingMessageID msg_id;
} send_socket_thread_args;

void send_socket_thread(void *arg)
{
    send_socket_thread_args args = *static_cast<send_socket_thread_args *>(arg);
    // Read from the queue

    // Send message to all subscribed sockets
    for (auto socket : send_sockets) {
        // ssize_t written =
        //   send(socket, const void *dataptr, size_t size, int flags)
    }
}

void recv_full_length(int &socket, uint8_t *buf, size_t len)
{
    // loop until the full requested length is placed into the buffer
    ssize_t to_recv = len;
    while (to_recv > 0) {
        ssize_t received = recv(socket, buf + (len - to_recv), to_recv, 0);
        if (received < 0) {
            // TODO: handle this
            ESP_LOGE(TAG, "Socket recv failed with error code: %d", errno);
            break;
        }
        to_recv -= received;
    }
}

// This function blocks until a valid message is received (TODO: timeout)
IncomingCommand recv_msg(int &socket)
{
    // Look for the magic number
    char magic[3] = {};

    while (magic[0] != MAGIC_NUMBER[0] || magic[1] != MAGIC_NUMBER[1] ||
           magic[2] != MAGIC_NUMBER[2]) {
        magic[0] = magic[1];
        magic[1] = magic[2];
        recv_full_length(socket, ((uint8_t *)magic) + 2, 1);
    }

    // Read the packet length

    // Read the data

    // uint8_t *buff[sizeof(IncomingCommand)];
    // ssize_t len = recv(socket, buff, sizeof(IncomingCommand), 0);
    //
    // Unpack into data structure
    return IncomingCommand();
}

void recv_socket_thread(void *arg)
{
    int socket = *static_cast<int *>(arg);

    pb_istream_t recv_stream = pb_istream_from_socket(socket);

    while (true) {
        fd_set read_set;
        FD_ZERO(&read_set);
        FD_SET(socket, &read_set);

        int ret = select(socket + 1, &read_set, NULL, NULL, NULL);

        switch (ret) {
            case (0): {
                // timeout
                ESP_LOGI(TAG, "Select timout");
                break;
            }
            case (-1): {
                ESP_LOGW(TAG, "Select returned with an error: %d", ret);
                break;
            }
            default: {
                IncomingCommand cmd;
                if (!pb_decode_delimited(
                      &recv_stream, IncomingCommand_fields, &cmd)) {
                    ESP_LOGE(TAG,
                             "Failed to decode message: %s. Closing socket.",
                             PB_GET_ERROR(&recv_stream));
                    close(socket);
                    vTaskDelete(NULL);
                    return;
                }

                // if message is a command, push recv'd message onto
                // relevant queue
                if (cmd.has_joint_cmd) {
                    ESP_LOGI(TAG, "Got cmd_vel");
                }

                // if message is a subscribe request, add the socket to
                // the relevant list
                if (cmd.has_subscribe_request) {
                    ESP_LOGI(TAG, "Got subscribe request");
                    // TODO: This needs to be controlled by a mutex
                    send_sockets[static_cast<int>(cmd.subscribe_request.msg_id)]
                      .push_back(socket);
                }
            }
        }
    }
}

QueueHandle_t register_data_producer(OutgoingMessageID id)
{
    QueueHandle_t handle = xQueueCreate(QUEUE_LENGTH, sizeof(OutgoingData));

    xTaskCreatePinnedToCore(send_socket_thread,
                            "send_socket_thread",
                            SOCKET_THREAD_STACK_SIZE,
                            static_cast<void *const>(handle),
                            10,
                            NULL,
                            PRO_CPU_NUM);

    return handle;
};

QueueHandle_t register_data_consumer(IncomingMessageID id)
{
    QueueHandle_t handle = xQueueCreate(QUEUE_LENGTH, sizeof(IncomingCommand));
    data_consumer_queues[static_cast<int>(id)].push_back(handle);
    return handle;
};

inline char *get_clients_address(struct sockaddr_storage *source_addr)
{
    static char address_str[128];
    char *res = NULL;
    // Convert ip address to string
    if (source_addr->ss_family == PF_INET) {
        res = inet_ntoa_r(((struct sockaddr_in *)source_addr)->sin_addr,
                          address_str,
                          sizeof(address_str) - 1);
    }
    if (!res) {
        address_str[0] =
          '\0'; // Returns empty string if conversion didn't succeed
    }
    return address_str;
}

void set_keep_alive(int &socket)
{
    int keepAlive = 1;
    int keepIdle = 5;
    int keepInterval = 5;
    int keepCount = 3;
    setsockopt(socket, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
    setsockopt(socket, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
    setsockopt(socket, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
    setsockopt(socket, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
}

void main_task(void *arg)
{
    struct sockaddr_in src_addr;
    src_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    src_addr.sin_family = AF_INET;
    src_addr.sin_port = htons(PORT);

    int listen_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

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

    err = listen(listen_socket, 10);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occured during listen: errno %d", errno);
        close(listen_socket);
        vTaskDelete(NULL);
    } else {
        ESP_LOGI(TAG, "Socket listening on port %d", PORT);
    }

    struct sockaddr_storage source_addr;
    socklen_t addr_len = sizeof(source_addr);

    while (true) {
        int *sock = new int(
          accept(listen_socket, (struct sockaddr *)&source_addr, &addr_len));

        if (*sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            continue;
        }

        ESP_LOGI(TAG,
                 "Connection accepted from IP:%s",
                 get_clients_address(&source_addr));

        set_keep_alive(*sock);

        xTaskCreatePinnedToCore(recv_socket_thread,
                                "recv_socket_thread",
                                SOCKET_THREAD_STACK_SIZE,
                                sock,
                                10,
                                NULL,
                                PRO_CPU_NUM);
    }
    vTaskDelete(NULL);
}

void init()
{
    xTaskCreatePinnedToCore(main_task,
                            "socket_manager_main",
                            SOCKET_MAIN_TASK_STACK_SIZE,
                            NULL,
                            5,
                            NULL,
                            PRO_CPU_NUM);
}
}
