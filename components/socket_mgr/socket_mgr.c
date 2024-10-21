#include "socket_mgr.h"

#include "freertos/idf_additions.h"
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <time.h>

#include "esp_log.h"
#include "freertos/projdefs.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "nvs.h"

#include "messages.pb.h"
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "pb_utils.h"
#include "portmacro.h"
#include "status_led_driver.h"

#define SOCKET_TX_TASK_STACK_SIZE 4096
#define SOCKET_RX_TASK_STACK_SIZE 4096
#define MAX_RX_CALLBACKS 1

#define PORT 8001

#define MULTICAST_IPV4_ADDR "239.255.12.11"
#define MULTICAST_TTL 5

static const char *TAG = "SOCKET_MGR";

static char AGENT_IP[16];

struct addrinfo *dest_addr;

static int socket_id;

QueueHandle_t tx_queue = NULL;

static unsigned char tx_buffer[1500];

static void socket_tx_task(void *arg)
{
    UdpPacket msg;

    while (1) {
        if (xQueueReceive(tx_queue, (void *)&msg, portMAX_DELAY) == pdTRUE) {
            pb_ostream_t stream =
              pb_ostream_from_buffer(tx_buffer, sizeof(tx_buffer));
            bool status = pb_encode(&stream, UdpPacket_fields, &msg);
            if (!status) {
                ESP_LOGE(TAG, "Failed to serialize message.");
            }

            ssize_t sent = sendto(socket_id,
                                  tx_buffer,
                                  stream.bytes_written,
                                  0,
                                  dest_addr->ai_addr,
                                  dest_addr->ai_addrlen);

            if (sent != stream.bytes_written) {
                ESP_LOGE(TAG,
                         "Failed to write full packet data. Wrote %ld, "
                         "expected %zu.",
                         (long)sent,
                         stream.bytes_written);
            }
        }
    }
}

static unsigned char rx_buffer[1500];
static void (*rx_callbacks[MAX_RX_CALLBACKS])(void *);

static void socket_rx_task(void *arg)
{
    struct sockaddr_storage source_addr;
    socklen_t socklen = sizeof(source_addr);
    while (1) {
        int len = recvfrom(socket_id,
                           rx_buffer,
                           sizeof(rx_buffer),
                           0,
                           (struct sockaddr *)&source_addr,
                           &socklen);

        if (len < 0) {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            return;
        } else {
            pb_istream_t stream = pb_istream_from_buffer(rx_buffer, len);

            // TODO: Get rid of the union stuff, just pass the UdpPacket message
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
    }
}

void register_callback(void (*callback)(void *), eRxMsgTypes type)
{
    rx_callbacks[type] = callback;
}

static void add_socket_multicast_group(int id)
{
    struct ip_mreq imreq = { 0 };
    struct in_addr iaddr = { 0 };
    int err = 0;
    // Configure source interface
    imreq.imr_interface.s_addr = IPADDR_ANY;

    // esp_netif_ip_info_t ip_info = { 0 };
    // err = esp_netif_get_ip_info(get_example_netif(), &ip_info);
    // if (err != ESP_OK) {
    //     ESP_LOGE(V4TAG, "Failed to get IP address info. Error 0x%x", err);
    //     goto err;
    // }
    // inet_addr_from_ip4addr(&iaddr, &ip_info.ip);

    // Configure multicast address to listen to
    err = inet_aton(MULTICAST_IPV4_ADDR, &imreq.imr_multiaddr.s_addr);
    if (err != 1) {
        ESP_LOGE(TAG,
                 "Configured IPV4 multicast address '%s' is invalid.",
                 MULTICAST_IPV4_ADDR);
        // Errors in the return value have to be negative
        err = -1;
    }
    ESP_LOGI(TAG,
             "Configured IPV4 Multicast address %s",
             inet_ntoa(imreq.imr_multiaddr.s_addr));
    if (!IP_MULTICAST(ntohl(imreq.imr_multiaddr.s_addr))) {
        ESP_LOGW(TAG,
                 "Configured IPV4 multicast address '%s' is not a valid "
                 "multicast address. This will probably not work.",
                 MULTICAST_IPV4_ADDR);
    }

    // Assign the IPv4 multicast source interface, via its IP
    err = setsockopt(
      id, IPPROTO_IP, IP_MULTICAST_IF, &iaddr, sizeof(struct in_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Failed to set IP_MULTICAST_IF. Error %d", errno);
    }

    err = setsockopt(
      id, IPPROTO_IP, IP_ADD_MEMBERSHIP, &imreq, sizeof(struct ip_mreq));
    if (err < 0) {
        ESP_LOGE(TAG, "Failed to set IP_ADD_MEMBERSHIP. Error %d", errno);
    }
}

static void create_socket()
{
    struct sockaddr_in src_addr;
    src_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    src_addr.sin_family = AF_INET;
    src_addr.sin_port = htons(PORT);

    socket_id = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

    if (socket_id < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    }

    int err = bind(socket_id, (struct sockaddr *)&src_addr, sizeof(src_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
    } else {
        set_status(eAgentConnected);
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);
    }

    uint8_t ttl = MULTICAST_TTL;
    err = setsockopt(
      socket_id, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(uint8_t));
    if (err < 0) {
        ESP_LOGE(TAG, "Failed to set IP_MULTICAST_TTL. Error %d", errno);
    }

    add_socket_multicast_group(socket_id);

    struct addrinfo hints = {
        .ai_flags = AI_PASSIVE,
        .ai_socktype = SOCK_DGRAM,
    };

    hints.ai_family = AF_INET; // For an IPv4 socket

    err = getaddrinfo(MULTICAST_IPV4_ADDR, NULL, &hints, &dest_addr);
    if (err < 0) {
        ESP_LOGE(TAG,
                 "getaddrinfo() failed for IPV4 destination address. error: %d",
                 err);
    }
    if (dest_addr == 0) {
        ESP_LOGE(TAG, "getaddrinfo() did not return any addresses");
    }
    
    char addrbuf[32] = { 0 };
    ((struct sockaddr_in *)dest_addr->ai_addr)->sin_port = htons(PORT);
    inet_ntoa_r(((struct sockaddr_in *)dest_addr->ai_addr)->sin_addr,
                addrbuf,
                sizeof(addrbuf) - 1);
    ESP_LOGI(
      TAG, "Sending to IPV4 multicast address %s:%d...", addrbuf, PORT);

    ESP_LOGI(TAG, "Socket created, communicating with multicast address %s:%d", addrbuf, PORT);
}

void socket_mgr_init()
{
    set_status(eAgentDisconnected);

    create_socket();

    tx_queue = xQueueCreate(25, sizeof(UdpPacket));

    xTaskCreatePinnedToCore(socket_tx_task,
                            "socket_tx_task",
                            SOCKET_TX_TASK_STACK_SIZE,
                            NULL,
                            10,
                            NULL,
                            APP_CPU_NUM);

    xTaskCreatePinnedToCore(socket_rx_task,
                            "socket_rx_task",
                            SOCKET_TX_TASK_STACK_SIZE,
                            NULL,
                            10,
                            NULL,
                            APP_CPU_NUM);
}
