#include <secrets.hpp>

#include <b1g/log/Logger.hpp>
#include <b1g/sensor/BME680.hpp>

#include <pico/binary_info.h>
#include <pico/cyw43_arch.h>
#include <pico/stdlib.h>

#include <lwip/apps/mqtt.h>

#include <FreeRTOS.h>
#include <message_buffer.h>
#include <task.h>

namespace {

using namespace b1g;

MessageBufferHandle_t sample_stream_buffer;
constexpr const size_t SAMPLE_BUF_SIZE = 60 * sizeof(sensor::BME680::Data);

void blink_task(__unused void* params) {
	bool on = false;
	int counter = 0;
	auto last_wake_time = xTaskGetTickCount();
	while (true) {
		cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
		on = !on;
		vTaskDelayUntil(&last_wake_time, 500);
	}
}

void sample_task(__unused void* params) {
	const uint sda_pin = 26;
	const uint scl_pin = 27;

	vTaskDelay(2000); // give tio chance to reconnect

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

void start_blink_task() {
	auto task_return = xTaskCreate(blink_task,
	                               "blink_task",
	                               configMINIMAL_STACK_SIZE,
	                               nullptr,
	                               1,
	                               nullptr);

	if (task_return != pdPASS) {
		log::error("blink_task failed: {}", task_return);
	}
}

bool is_wifi_connected() {
	return cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP;
}

void connect_wifi() {
	while (!is_wifi_connected()) {
		log::info("Connecting to WiFi...");
		auto connect_ret = cyw43_arch_wifi_connect_blocking(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK);

		auto link_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
		if (connect_ret == 0 && link_status == CYW43_LINK_UP) {
			auto local_IP = ip4_addr_get_u32(netif_ip4_addr(cyw43_state.netif));
			log::info("Successfully connected to WiFi: IP Address {}.{}.{}.{}",
			          (local_IP & 0xff),
			          ((local_IP >> 8) & 0xff),
			          ((local_IP >> 16) & 0xff),
			          (local_IP >> 24));
		} else {
			log::error("Connect failed, retrying");
			vTaskDelay(1000);
		}
	}
}

void mqtt_connection_cb(mqtt_client_t*, void*, mqtt_connection_status_t) {
}

void mqtt_publish_cb(void*, err_t) {
}

void connect_mqtt(mqtt_client_t* client) {
	log::info("lets connect to mqtt");
	ip_addr_t mqtt_server_ip;
	ipaddr_aton(MQTT_SERVER_HOST, &mqtt_server_ip);

	static const struct mqtt_connect_client_info_t mqtt_client_info =
	    {
	        "pico-w-enviro",
	        NULL, /* user */
	        NULL, /* pass */
	        100,  /* keep alive */
	        NULL, /* will_topic */
	        NULL, /* will_msg */
	        0,    /* will_qos */
	        0     /* will_retain */
	    };

	mqtt_client_connect(client, &mqtt_server_ip, MQTT_SERVER_PORT, mqtt_connection_cb, nullptr, &mqtt_client_info);

	vTaskDelay(3000);
}

void network_task(__unused void* params) {
	vTaskDelay(2000); // give tio chance to reconnect

	if (cyw43_arch_init_with_country(CYW43_COUNTRY_USA)) {
		log::error("WiFi init failed");
		return;
	}

	auto mqtt_client = mqtt_client_new();
	if (mqtt_client == nullptr) {
		log::error("MQTT client create failed");
		return;
	}

	start_blink_task();
	cyw43_arch_enable_sta_mode();

	sensor::BME680::Data sample_data{};

	while (true) {
		log::info("WiFi loop");
		if (!is_wifi_connected()) {
			connect_wifi();
		} else if (!mqtt_client_is_connected(mqtt_client)) {
			connect_mqtt(mqtt_client);
		} else {
			log::info("connected to mqtt");
			while (true) {
				auto len = xMessageBufferReceive(sample_stream_buffer,
				                                 &sample_data,
				                                 sizeof(sample_data),
				                                 0);
				log::info("sample len: {}", len);
				if (len == sizeof(sample_data)) {
					mqtt_publish(mqtt_client, "temp", &sample_data, sizeof(sample_data), 0, 0, mqtt_publish_cb, nullptr);
				} else {
					break;
				}
			}
		}

		vTaskDelay(1000);
	}
}

} // namespace

int main() {
	auto task_return = xTaskCreate(network_task, "network_task", 2048, nullptr, 1, nullptr);
	assert(task_return == pdPASS);

	task_return = xTaskCreate(sample_task, "sample_task", 2048, nullptr, 1, nullptr);
	assert(task_return == pdPASS);

	sample_stream_buffer = xMessageBufferCreate(SAMPLE_BUF_SIZE);
	assert(sample_stream_buffer != nullptr);

	stdio_init_all();
	log::error("Starting FreeRTOS on core 0");
	vTaskStartScheduler();

	return 0;
}
