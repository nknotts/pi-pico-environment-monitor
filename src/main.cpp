
#include <hardware/clocks.h>
#include <hardware/i2c.h>
#include <hardware/vreg.h>
#include <pico/binary_info.h>
#include <pico/cyw43_arch.h>
#include <pico/stdlib.h>

#define FMT_HEADER_ONLY
#include <fmt/format.h>

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

constexpr const unsigned int LED_PIN = 25;

int main() {
	bi_decl(bi_program_description("First Blink"));
	bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));

	stdio_init_all();

	fmt::print("It works!");

	if (cyw43_arch_init_with_country(CYW43_COUNTRY_USA)) {
		fmt::print("WiFi init failed");
		return -1;
	}

	cyw43_arch_enable_sta_mode();

	absolute_time_t scan_test = nil_time;
	bool scan_in_progress = false;
	bool led_state = false;

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
}
