
#include <hardware/i2c.h>
#include <pico/binary_info.h>
#include <pico/cyw43_arch.h>
#include <pico/stdlib.h>

#include "hardware/clocks.h"
#include "hardware/vreg.h"

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

	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);
	while (1) {
		gpio_put(LED_PIN, 0);
		sleep_ms(250);
		gpio_put(LED_PIN, 1);
		sleep_ms(1000);
	}
}
