#pragma once

#include "freertos/idf_additions.h"

#include "messages.pb.h"

namespace SocketManager {
void init();

QueueHandle_t register_data_producer(OutgoingMessageID id);

QueueHandle_t register_data_consumer(IncomingMessageID id);
}
