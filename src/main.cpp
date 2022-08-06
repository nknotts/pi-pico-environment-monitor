#include <sensor/BME680.hpp>

#include <hardware/clocks.h>
#include <hardware/i2c.h>
#include <hardware/vreg.h>
#include <pico/binary_info.h>
#include <pico/cyw43_arch.h>
#include <pico/stdlib.h>

#include <fmt/core.h>

#include <lwip/apps/mqtt_opts.h>
#include <lwip/ip4_addr.h>

#include <FreeRTOS.h>
#include <task.h>

#include <stdio.h>

static int scan_result(void* env, const cyw43_ev_scan_result_t* result) {
	if (result) {
		printf("ssid: %-32s rssi: %4d chan: %3d mac: %02x:%02x:%02x:%02x:%02x:%02x sec: %u\n",
		       result->ssid, result->rssi, result->channel,
		       result->bssid[0], result->bssid[1], result->bssid[2], result->bssid[3], result->bssid[4], result->bssid[5],
		       result->auth_mode);
	}
	return 0;
}

static constexpr const char* WIFI_SSID = "${WIFI_SSID}";
static constexpr const char* WIFI_PASSWORD = "${WIFI_PASSWORD}";
static constexpr const char* PING_ADDR = "10.0.0.1";

void main_task(__unused void* params) {
	if (cyw43_arch_init_with_country(CYW43_COUNTRY_USA)) {
		printf("failed to initialise\n");
		return;
	}
	cyw43_arch_enable_sta_mode();
	printf("Connecting to WiFi...\n");

	if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
		printf("failed to connect.\n");
		exit(1);
	} else {
		printf("Connected.\n");
	}

	ip_addr_t ping_addr;
	ip4_addr_set_u32(&ping_addr, ipaddr_addr(PING_ADDR));
	// ping_init(&ping_addr);

	while (true) {
		// not much to do as LED is in another task, and we're using RAW (callback) lwIP API
		vTaskDelay(100);
	}

	cyw43_arch_deinit();
}

void blink_task(__unused void* params) {
	bool on = false;
	fmt::print("blink_task starts");
	while (true) {
		cyw43_arch_gpio_put(0, on);
		on = !on;
		vTaskDelay(200);
	}
}

int main() {
	bi_decl(bi_program_description("Environment MQTT"));

	stdio_init_all();

	fmt::print("It works!");

	sensor::BME680 bme{};

	absolute_time_t scan_test = nil_time;
	bool scan_in_progress = false;
	bool led_state = false;

	TaskHandle_t blink_task_handle;
	xTaskCreate(blink_task,
	            "BlinkThread",
	            configMINIMAL_STACK_SIZE,
	            nullptr,
	            tskIDLE_PRIORITY + 1,
	            &blink_task_handle);

	fmt::print("Starting FreeRTOS on core 0");
	vTaskStartScheduler();

	while (true) {
		if (absolute_time_diff_us(get_absolute_time(), scan_test) < 0) {
			if (!scan_in_progress) {
				cyw43_wifi_scan_options_t scan_options = {0};
				int err = cyw43_wifi_scan(&cyw43_state, &scan_options, NULL, scan_result);
				if (err == 0) {
					fmt::print("\nPerforming wifi scan\n");
					scan_in_progress = true;
				} else {
					fmt::print("Failed to start scan: {}", err);
					scan_test = make_timeout_time_ms(10000); // wait 10s and scan again
				}
			} else if (!cyw43_wifi_scan_active(&cyw43_state)) {
				scan_test = make_timeout_time_ms(10000); // wait 10s and scan again
				scan_in_progress = false;
			}
		}

		cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
		led_state = !led_state;
		sleep_ms(500);
	}

	return 0;
}
