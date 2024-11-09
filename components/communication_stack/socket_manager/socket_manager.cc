#include "socket_manager.h"

#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <set>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <sys/select.h>
#include <sys/types.h>
#include <time.h>
#include <vector>

#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "lwip/sockets.h"

#include "common.h"
#include "messages.pb.h"
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "portmacro.h"
#include "soc/soc.h"

namespace SocketManager {

#define TAG "SocketManager"

#define SOCKET_MAIN_TASK_STACK_SIZE 4096
#define SOCKET_THREAD_STACK_SIZE 4096

#define MAX_CLIENTS 10

#define QUEUE_LENGTH 10

#define PORT 8001

#define MAGIC_NUMBER "LRR"

SemaphoreHandle_t send_sockets_mutex;
std::set<int>
  send_sockets[static_cast<int>(OutgoingMessageID_outgoing_msg_count)];

SemaphoreHandle_t consumer_queue_handle_mutex;
std::vector<QueueHandle_t>
  consumer_queues[static_cast<int>(IncomingMessageID_incoming_msg_count)];

typedef struct
{
    QueueHandle_t queue_handle;
    OutgoingMessageID msg_id;
} send_socket_thread_args;

void send_socket_thread(void *arg)
{
    send_socket_thread_args args = *static_cast<send_socket_thread_args *>(arg);

    OutgoingData data;

    while (send_sockets[args.msg_id].size() == 0) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    while (xQueueReceive(args.queue_handle, &data, portMAX_DELAY)) {
        if (send_sockets_mutex == NULL) {
            continue;
        }

        xSemaphoreTake(send_sockets_mutex, portMAX_DELAY);
        auto socket_set = send_sockets[args.msg_id];
        xSemaphoreGive(send_sockets_mutex);
        // Send message to all subscribed sockets
        for (auto socket : socket_set) {
            pb_ostream_t send_stream = pb_ostream_from_socket(socket);
            if (!pb_encode_delimited(
                  &send_stream, OutgoingData_fields, &data)) {
                ESP_LOGE(TAG,
                         "Encoding failed: %s on socket: %d",
                         PB_GET_ERROR(&send_stream),
                         socket);
            }
        }
    }
    ESP_LOGE(TAG, "Queue recieve failed.");
    vTaskDelete(NULL);
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
                ESP_LOGE(TAG, "Select returned with an error: %d", ret);
                break;
            }
            default: {
                IncomingCommand cmd;
                if (!pb_decode_delimited(
                      &recv_stream, IncomingCommand_fields, &cmd)) {
                    // Most likely hit EOF (connection closed)
                    ESP_LOGD(TAG,
                             "Failed to decode message: %s. Closing socket.",
                             PB_GET_ERROR(&recv_stream));

                    xSemaphoreTake(send_sockets_mutex, portMAX_DELAY);
                    for (auto &set : send_sockets) {
                        set.erase(socket);
                    }
                    xSemaphoreGive(send_sockets_mutex);

                    close(socket);
                    vTaskDelete(NULL);
                    return;
                }

                // if message is a command, push recv'd message onto
                // relevant queue
                if (cmd.has_joint_cmd) {
                    xSemaphoreTake(consumer_queue_handle_mutex, portMAX_DELAY);
                    for (auto queue : consumer_queues[cmd.msg_id]) {
                        xQueueSend(queue, &cmd, 0);
                    }
                    xSemaphoreGive(consumer_queue_handle_mutex);
                }

                // if message is a subscribe request, add the socket to
                // the relevant list
                if (cmd.has_subscribe_request) {
                    ESP_LOGI(TAG,
                             "Got subscribe request %d from %d",
                             cmd.subscribe_request.msg_id,
                             socket);
                    if (send_sockets_mutex == NULL) {
                        ESP_LOGE(TAG,
                                 "Subscription request created before mutex "
                                 "initialized.");
                        continue;
                    }
                    xSemaphoreTake(send_sockets_mutex, portMAX_DELAY);
                    send_sockets[static_cast<int>(cmd.subscribe_request.msg_id)]
                      .insert(socket);
                    xSemaphoreGive(send_sockets_mutex);
                }
            }
        }
    }
    vTaskDelete(NULL);
}

QueueHandle_t register_data_producer(OutgoingMessageID id)
{
    send_socket_thread_args *const args = new send_socket_thread_args(
      { .queue_handle = xQueueCreate(QUEUE_LENGTH, sizeof(OutgoingData)),
        .msg_id = id });

    xTaskCreatePinnedToCore(send_socket_thread,
                            "send_socket_thread",
                            SOCKET_THREAD_STACK_SIZE,
                            static_cast<void *const>(args),
                            5,
                            NULL,
                            tskNO_AFFINITY);

    return args->queue_handle;
};

QueueHandle_t register_data_consumer(IncomingMessageID id)
{
    QueueHandle_t handle = xQueueCreate(QUEUE_LENGTH, sizeof(IncomingCommand));

    if (consumer_queue_handle_mutex != NULL) {
        ESP_LOGE(TAG, "Attempting to add consumer after init.");
        return nullptr;
    }

    consumer_queues[static_cast<int>(id)].push_back(handle);

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
                                5,
                                NULL,
                                tskNO_AFFINITY);
    }
    vTaskDelete(NULL);
}

void init()
{
    send_sockets_mutex = xSemaphoreCreateMutex();
    consumer_queue_handle_mutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(main_task,
                            "socket_manager_main",
                            SOCKET_MAIN_TASK_STACK_SIZE,
                            NULL,
                            5,
                            NULL,
                            tskNO_AFFINITY);
}
}
