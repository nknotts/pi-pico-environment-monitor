#include <secrets.hpp>

#include <b1g/log/Logger.hpp>
#include <b1g/sensor/BME680.hpp>

#include <libEnviroMqtt_version.h>

#include <hardware/rtc.h>
#include <hardware/watchdog.h>
#include <pico/binary_info.h>
#include <pico/cyw43_arch.h>
#include <pico/stdlib.h>
#include <pico/unique_id.h>
#include <pico/util/datetime.h>

#include <lwip/apps/mqtt.h>
#include <lwip/apps/sntp.h>

#include <FreeRTOS.h>
#include <message_buffer.h>
#include <task.h>

#include <tusb.h>

#include <ctime>

namespace {

using namespace b1g;

MessageBufferHandle_t sample_stream_buffer;
constexpr const size_t SAMPLE_BUF_SIZE = MQTT_OUTPUT_RINGBUF_SIZE;

constexpr const size_t CDC_CONNECTED_BUF_SIZE = 8;
MessageBufferHandle_t cdc_connected_msg_buffer;

constexpr const size_t SNTP_BUF_SIZE = 2 * sizeof(uint32_t);
MessageBufferHandle_t sntp_msg_buffer;

constexpr const size_t MQTT_CONNECTION_BUF_SIZE = 4 * sizeof(mqtt_connection_status_t);
MessageBufferHandle_t mqtt_connection_msg_buffer;

void blink_task(__unused void* params) {
	bool on = false;
	int counter = 0;
	auto last_wake_time = xTaskGetTickCount();
	while (true) {
		watchdog_update();
		cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
		on = !on;
		vTaskDelayUntil(&last_wake_time, 500);
	}
}

void sample_task(__unused void* params) {
	const uint sda_pin = 26;
	const uint scl_pin = 27;

	i2c_inst_t* i2c = i2c1;
	i2c_init(i2c, 400 * 1000);
	gpio_set_function(sda_pin, GPIO_FUNC_I2C);
	gpio_set_function(scl_pin, GPIO_FUNC_I2C);
	gpio_pull_up(sda_pin);
	gpio_pull_up(scl_pin);
	// Make the I2C pins available to picotool
	bi_decl(bi_2pins_with_func(sda_pin, scl_pin, GPIO_FUNC_I2C));

	sensor::BME680 bme{i2c, 0x77};

	sensor::BME680::Data data;
	int counter = 0;
	auto last_wake_time = xTaskGetTickCount();
	while (true) {
		if (bme.Sample(data)) {
			log::info("sample_task: {:.1f} C, {:.1f} %RH, {:.1f} Pa, {:.1f} ohm",
			          data.temperature_C,
			          data.humidity_rh,
			          data.pressure_Pa,
			          data.gas_ohm);
			xMessageBufferSend(sample_stream_buffer, &data, sizeof(data), 0);
		} else {
			log::error("sample_task: Failed to get sample");
		}
		vTaskDelayUntil(&last_wake_time, 1000);
	}
}

bool is_wifi_connected() {
	return cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP;
}

void connect_wifi() {
	while (!is_wifi_connected()) {
		log::info("Connecting to WiFi...");
		sntp_stop();
		auto connect_ret = cyw43_arch_wifi_connect_blocking(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK);

		auto link_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
		if (connect_ret == 0 && link_status == CYW43_LINK_UP) {
			auto local_IP = ip4_addr_get_u32(netif_ip4_addr(cyw43_state.netif));
			log::info("Successfully connected to WiFi: IP Address {}.{}.{}.{}",
			          (local_IP & 0xff),
			          ((local_IP >> 8) & 0xff),
			          ((local_IP >> 16) & 0xff),
			          (local_IP >> 24));

			sntp_init();
		} else {
			log::error("Connect failed, retrying");
			vTaskDelay(1000);
		}
	}
}

void mqtt_connection_cb(mqtt_client_t*, void*, mqtt_connection_status_t status) {
	BaseType_t taskWoken = pdFALSE;
	xMessageBufferSendFromISR(mqtt_connection_msg_buffer, &status, sizeof(status), &taskWoken);
	portYIELD_FROM_ISR(taskWoken);
}

void mqtt_publish_cb(void*, err_t) {
}

static char PICO_BOARD_ID[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1];
static char ENVIRO_MONITOR_MQTT_CLIENT_NAME[64];

void software_reset() {
	// https://forums.raspberrypi.com/viewtopic.php?t=318747#p1928868
#define AIRCR_Register (*((volatile uint32_t*)(PPB_BASE + 0x0ED0C)))
	AIRCR_Register = 0x5FA0004;
	// watchdog_reboot(0, 0, 0);
}

void connect_mqtt(mqtt_client_t* client) {
	log::info("MQTT Begin Connect");
	ip_addr_t mqtt_server_ip;
	ipaddr_aton(MQTT_SERVER_HOST, &mqtt_server_ip);

	static const struct mqtt_connect_client_info_t mqtt_client_info =
	    {
	        ENVIRO_MONITOR_MQTT_CLIENT_NAME,
	        NULL, /* user */
	        NULL, /* pass */
	        100,  /* keep alive */
	        NULL, /* will_topic */
	        NULL, /* will_msg */
	        0,    /* will_qos */
	        0     /* will_retain */
	    };

	auto err = mqtt_client_connect(client, &mqtt_server_ip, MQTT_SERVER_PORT, mqtt_connection_cb, nullptr, &mqtt_client_info);
	if (err != ERR_OK) {
		// Not sure why, but after disconnect, unable to properly reconnect
		// resetting the pico works robustly, if not heavy handed
		log::error("MQTT Client Connect Error: {}", static_cast<int>(err));
		vTaskDelay(200); // give time for log to
		software_reset();
	}

	mqtt_connection_status_t status;
	auto len = xMessageBufferReceive(mqtt_connection_msg_buffer,
	                                 &status,
	                                 sizeof(status),
	                                 120000);
	if (len == sizeof(status)) {
		log::info("MQTT Connection Result: {}", static_cast<int>(status));
	} else {
		log::error("MQTT Connection Timed out");
	}
}

constexpr const uint8_t MQTT_QOS = 0;    // lowest reliability - "best effort"
constexpr const uint8_t MQTT_RETAIN = 1; // store last value for late joiners -  "durability"
static char MQTT_TOPIC_NAME[128];

void network_task(__unused void* params) {
	if (cyw43_arch_init_with_country(CYW43_COUNTRY_USA)) {
		log::error("WiFi init failed");
		return;
	}

	cyw43_arch_enable_sta_mode();

	{
		sntp_setoperatingmode(SNTP_OPMODE_POLL);

		ip_addr_t ntp_ip;
		ipaddr_aton(NTP_HOST, &ntp_ip);
		sntp_setserver(0, &ntp_ip);
	}

	{
		auto task_return = xTaskCreate(blink_task, "blink_task", configMINIMAL_STACK_SIZE, nullptr, 1, nullptr);
		assert(task_return == pdPASS);
	}

	auto mqtt_client = mqtt_client_new();
	assert(mqtt_client != nullptr);

	sensor::BME680::Data sample_data{};
	char json_buf[256];
	uint32_t sequence_number{};
	while (true) {
		if (!is_wifi_connected()) {
			connect_wifi();
		} else if (!mqtt_client_is_connected(mqtt_client)) {
			connect_mqtt(mqtt_client);
		} else {
			auto len = xMessageBufferReceive(sample_stream_buffer,
			                                 &sample_data,
			                                 sizeof(sample_data),
			                                 2000);
			if (len == sizeof(sample_data)) {
				auto tick_now = xTaskGetTickCount();
				datetime_t date_now;
				if (!rtc_get_datetime(&date_now)) {
					continue;
				}

				struct tm tm_now {};
				tm_now.tm_year = date_now.year - 1900;
				tm_now.tm_mon = date_now.month - 1;
				tm_now.tm_mday = date_now.day;
				tm_now.tm_wday = date_now.dotw;
				tm_now.tm_hour = date_now.hour;
				tm_now.tm_min = date_now.min;
				tm_now.tm_sec = date_now.sec;
				tm_now.tm_isdst = false;
				auto time_now = mktime(&tm_now);

				auto dt_tick = tick_now - sample_data.ttag_ms;
				int64_t dt_sec = (dt_tick * 1000 + 500) / 1000000; // nearest 1000 ms / 1s
				int64_t unix_time = time_now - dt_sec;

				auto len = snprintf(
				    json_buf,
				    sizeof(json_buf),
				    "{\"id\": %u, \"client_id\":\"%s\", \"ttag_s\": %lld, \"temperature_C\":%.1f, \"pressure_Pa\": %.1f, \"humidity_rh\": %.1f, \"gas_ohm\": %.1f}",
				    ++sequence_number,
				    PICO_BOARD_ID,
				    unix_time,
				    sample_data.temperature_C,
				    sample_data.pressure_Pa,
				    sample_data.humidity_rh,
				    sample_data.gas_ohm);

				auto ret = mqtt_publish(mqtt_client, MQTT_TOPIC_NAME, json_buf, len, MQTT_QOS, MQTT_RETAIN, mqtt_publish_cb, nullptr);
				if (ret != ERR_OK) {
					// Not sure why, but after disconnect, unable to properly reconnect
					// resetting the pico works robustly, if not heavy handed
					log::error("MQTT Publish Failed: {}", ret);
					vTaskDelay(200); // give time for log to flush
					software_reset();
				}
				// TODO: in mqtt_publish_cb, publish to a message buffer and wait for result here
			} else {
				break;
			}
		}
	}
}

void tusb_task(__unused void* params) {
	while (true) {
		tud_task();
		vTaskDelay(1);
	}
}

void cdc_connected_task(__unused void* params) {
	uint8_t itf{};
	while (true) {
		auto len = xMessageBufferReceive(cdc_connected_msg_buffer, &itf, 1, 1000);
		if (len == 1) {
			log::info("Connected to b1g enviro monitor: {}", VERSION);
		}
	}
}

void sntp_task(__unused void* params) {
	uint32_t ntp_sec{};
	while (true) {
		auto len = xMessageBufferReceive(sntp_msg_buffer, &ntp_sec, sizeof(ntp_sec), 7200000);
		if (len == sizeof(ntp_sec)) {
			static uint32_t ntp_to_unix = (70 * 365 + 17) * 86400U;
			time_t unix_time = ntp_sec - ntp_to_unix;

			struct tm datetime;
			gmtime_r(&unix_time, &datetime);

			datetime_t t = {
			    .year = static_cast<int16_t>(datetime.tm_year + 1900),
			    .month = static_cast<int8_t>(datetime.tm_mon + 1),
			    .day = static_cast<int8_t>(datetime.tm_mday),
			    .dotw = static_cast<int8_t>(datetime.tm_wday), // 0 is Sunday, so 5 is Friday
			    .hour = static_cast<int8_t>(datetime.tm_hour),
			    .min = static_cast<int8_t>(datetime.tm_min),
			    .sec = static_cast<int8_t>(datetime.tm_sec)};

			if (rtc_set_datetime(&t)) {
				char buf[64];
				datetime_to_str(buf, sizeof(buf), &t);
				log::info("Got SNTP Time: {}", buf);
			} else {
				log::error("SNTP Time Failed");
			}
		}
	}
}

} // namespace

// Invoked when CDC interface connection status changes
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
	if (itf == CDC_ITF_LOG && dtr) {
		BaseType_t taskWoken = pdFALSE;
		xStreamBufferSendFromISR(cdc_connected_msg_buffer, &itf, 1, &taskWoken);
		portYIELD_FROM_ISR(taskWoken);
	}
}

void sntp_set_system_time_us(uint32_t sec, uint64_t us) {
	BaseType_t taskWoken = pdFALSE;
	xMessageBufferSendFromISR(sntp_msg_buffer, &sec, sizeof(sec), &taskWoken);
	portYIELD_FROM_ISR(taskWoken);
}

int main() {
	watchdog_enable(1000, 1);
	pico_get_unique_board_id_string(PICO_BOARD_ID, sizeof(PICO_BOARD_ID));
	snprintf(ENVIRO_MONITOR_MQTT_CLIENT_NAME,
	         sizeof(ENVIRO_MONITOR_MQTT_CLIENT_NAME),
	         "b1g-enviro-monitor-%s",
	         PICO_BOARD_ID);
	snprintf(MQTT_TOPIC_NAME,
	         sizeof(MQTT_TOPIC_NAME),
	         "data/environment-monitor/%s",
	         PICO_BOARD_ID);

	auto task_return = xTaskCreate(network_task, "network_task", 2048, nullptr, 1, nullptr);
	assert(task_return == pdPASS);

	task_return = xTaskCreate(sample_task, "sample_task", 2048, nullptr, 1, nullptr);
	assert(task_return == pdPASS);

	task_return = xTaskCreate(tusb_task, "tusb_task", 2048, nullptr, 1, nullptr);
	assert(task_return == pdPASS);

	task_return = xTaskCreate(cdc_connected_task, "cdc_connected_task", 2048, nullptr, 1, nullptr);
	assert(task_return == pdPASS);

	task_return = xTaskCreate(sntp_task, "sntp_task", 2048, nullptr, 1, nullptr);
	assert(task_return == pdPASS);

	sample_stream_buffer = xMessageBufferCreate(SAMPLE_BUF_SIZE);
	assert(sample_stream_buffer != nullptr);

	cdc_connected_msg_buffer = xMessageBufferCreate(CDC_CONNECTED_BUF_SIZE);
	assert(cdc_connected_msg_buffer != nullptr);

	sntp_msg_buffer = xMessageBufferCreate(SNTP_BUF_SIZE);
	assert(sntp_msg_buffer != nullptr);

	mqtt_connection_msg_buffer = xMessageBufferCreate(MQTT_CONNECTION_BUF_SIZE);
	assert(mqtt_connection_msg_buffer != nullptr);

	tusb_init(); // initialize tinyusb stack
	rtc_init();
	log::error("Starting FreeRTOS on core 0");
	vTaskStartScheduler();

	return 0;
}
