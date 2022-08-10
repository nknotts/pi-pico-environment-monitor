
#include <sensor/BME680.hpp>

#include <pico/binary_info.h>
#include <pico/cyw43_arch.h>
#include <pico/stdlib.h>

#include <log/Logger.hpp>

#include <FreeRTOS.h>
#include <task.h>

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
		} else {
			log::error("sample_task: Failed to get sample");
		}
		vTaskDelayUntil(&last_wake_time, 1000);
	}
}

int main() {
	stdio_init_all();
	if (cyw43_arch_init()) {
		log::error("WiFi init failed");
		return -1;
	}

	auto task_return = xTaskCreate(blink_task,
	                               "blink_task",
	                               2048,
	                               nullptr,
	                               1,
	                               nullptr);

	if (task_return != pdPASS) {
		log::error("blink_task failed");
		return -1;
	}

	task_return = xTaskCreate(sample_task,
	                          "sample_task",
	                          2048,
	                          nullptr,
	                          1,
	                          nullptr);

	if (task_return != pdPASS) {
		log::error("sample_task failed");
		return -1;
	}

	log::error("Starting FreeRTOS on core 0");
	vTaskStartScheduler();

	return 0;
}
