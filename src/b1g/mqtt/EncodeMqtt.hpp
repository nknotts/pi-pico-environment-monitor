#pragma once

#include <cstdint>
#include <cstring>

namespace b1g {
namespace mqtt {

size_t EncodeMqttConnect(uint8_t* buf,
                         const char* client_id_buf,
                         uint8_t client_id_len);

size_t EncodeMqttPublish(uint8_t* buf,
                         const char* topic_buf,
                         uint8_t topic_len,
                         const char* payload_buf,
                         uint8_t payload_len);

} // namespace mqtt
} // namespace b1g
