#include "EncodeMqtt.hpp"

// http://www.steves-internet-guide.com/mqtt-protocol-messages-overview/

namespace b1g {
namespace mqtt {

size_t EncodeMqttConnect(uint8_t* buf,
                         const char* client_id_buf,
                         uint8_t client_id_len) {
	uint8_t len = 14 + client_id_len;

	// fixed header
	buf[0] = 0x10;    // connect msg id
	buf[1] = len - 2; // remaining length, does not include header

	// length of protocol name
	buf[2] = 0;
	buf[3] = 4;

	// protocol name
	buf[4] = 'M';
	buf[5] = 'Q';
	buf[6] = 'T';
	buf[7] = 'T';

	// protocol version
	buf[8] = 4;

	// connect flags
	buf[9] = 0x02; // clean session

	// keep alive - 60 seconds
	buf[10] = 0;
	buf[11] = 60;

	// client id len
	buf[12] = 0;
	buf[13] = client_id_len;

	// client id
	std::memcpy(&buf[14], client_id_buf, client_id_len);

	return len;
}

size_t EncodeMqttPublish(uint8_t* buf,
                         const char* topic_buf,
                         uint8_t topic_len,
                         const char* payload_buf,
                         uint8_t payload_len) {

	uint8_t remaining_len = 2 + topic_len + payload_len; // 2 byte topic_len + topic + payload

	// fixed header
	buf[0] = 0x30; // publish msg id

	// remaining len, can be 1 or 2 bytes based on length
	uint8_t len;
	int idx;
	if (remaining_len < 128) {
		len = remaining_len + 2;
		buf[1] = remaining_len;
		idx = 2;
	} else {
		len = remaining_len + 3;
		buf[1] = remaining_len;
		buf[2] = remaining_len >> 7;
		idx = 3;
	}

	// topic length
	buf[idx++] = 0;
	buf[idx++] = topic_len;

	// topic payload
	std::memcpy(&buf[idx], topic_buf, topic_len);
	idx += topic_len;

	// payload
	std::memcpy(&buf[idx], payload_buf, payload_len);

	return len;
}

} // namespace mqtt
} // namespace b1g
