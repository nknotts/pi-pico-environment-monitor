
#include <sensor/BME680.hpp>
#include <sensor/I2cScan.hpp>

#include <pico/binary_info.h>
#include <pico/cyw43_arch.h>
#include <pico/stdlib.h>

#include <fmt/format.h>

#include <FreeRTOS.h>
#include <task.h>

void blink_task(__unused void* params) {
	bool on = false;
	// fmt::print("blink_task starts\n");
	int counter = 0;
	auto last_wake_time = xTaskGetTickCount();
	while (true) {
		cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
		on = !on;
		// fmt::print("blink_task: {}\n", counter++);
		vTaskDelayUntil(&last_wake_time, 500);
	}
}

void sample_task(__unused void* params) {
	const uint sda_pin = 26;
	const uint scl_pin = 27;

	vTaskDelay(2000);
	fmt::print("sample_task started\n");

	i2c_inst_t* i2c = i2c1;
	i2c_init(i2c, 400 * 1000);
	gpio_set_function(sda_pin, GPIO_FUNC_I2C);
	gpio_set_function(scl_pin, GPIO_FUNC_I2C);
	gpio_pull_up(sda_pin);
	gpio_pull_up(scl_pin);
	// Make the I2C pins available to picotool
	bi_decl(bi_2pins_with_func(sda_pin, scl_pin, GPIO_FUNC_I2C));

	sensor::i2c_scan(i2c);

	sensor::BME680 bme{i2c, 0x77};

	int counter = 0;
	auto last_wake_time = xTaskGetTickCount();
	while (true) {
		fmt::print("sample_task: {}\n", counter++);
		auto result = bme.Sample();
		(void)result;
		vTaskDelayUntil(&last_wake_time, 1000);
	}
}

int main() {
	stdio_init_all();
	if (cyw43_arch_init()) {
		fmt::print("WiFi init failed\n");
		return -1;
	}

	auto task_return = xTaskCreate(blink_task,
	                               "blink_task",
	                               2048,
	                               nullptr,
	                               1,
	                               nullptr);

	if (task_return != pdPASS) {
		fmt::print("blink_task failed\n");
		return -1;
	}

	task_return = xTaskCreate(sample_task,
	                          "sample_task",
	                          2048,
	                          nullptr,
	                          1,
	                          nullptr);

	if (task_return != pdPASS) {
		fmt::print("sample_task failed\n");
		return -1;
	}

	fmt::print("Starting FreeRTOS on core 0\n");
	vTaskStartScheduler();

	return 0;
}
