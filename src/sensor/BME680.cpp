#include "BME680.hpp"

#include <hardware/i2c.h>
#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <task.h>

#include <fmt/core.h>

namespace sensor {

namespace {

int8_t i2c_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf) {
	i2c_inst_t* i2c = (i2c_inst_t*)intf;
	int num_read = i2c_read_blocking(i2c, reg_addr, reg_data, len, true);
	return num_read == len ? 0 : -1;
}

int8_t i2c_write(uint8_t reg_addr,
                 const uint8_t* reg_data,
                 uint32_t len,
                 void* intf) {
	i2c_inst_t* i2c = (i2c_inst_t*)intf;
	int num_write = i2c_write_blocking(i2c, reg_addr, reg_data, len, true);
	return num_write == len ? 0 : -1;
}

void delay_usec(uint32_t us, void* intf_ptr) {
	(void)intf_ptr; // Unused parameter
	vTaskDelay(us / 1000);
}

// https://github.com/adafruit/Adafruit_BME680/blob/master/Adafruit_BME680.cpp

} // namespace

BME680::BME680() {

	const uint sda_pin = 16;
	const uint scl_pin = 17;

	// Ports
	i2c = i2c0;

	// Buffer to store raw reads
	uint8_t data[6];

	// Initialize I2C port at 400 kHz
	i2c_init(i2c, 400 * 1000);

	// Initialize I2C pins
	gpio_set_function(sda_pin, GPIO_FUNC_I2C);
	gpio_set_function(scl_pin, GPIO_FUNC_I2C);

	dev.chip_id = 0x77;
	dev.intf = BME68X_I2C_INTF;
	dev.intf_ptr = (void*)i2c;
	dev.read = &i2c_read;
	dev.write = &i2c_write;
	dev.amb_temp = 25; /* The ambient temperature in deg C is used for
	                         defining the heater temperature */
	dev.delay_us = delay_usec;

	int8_t rslt = bme68x_init(&dev);
	fmt::print("Init Result: {}", rslt);

	bme68x_conf gas_conf;
	gas_conf.filter = BME68X_FILTER_SIZE_3;
	gas_conf.odr = BME68X_ODR_NONE;
	gas_conf.os_hum = BME68X_OS_2X;
	gas_conf.os_pres = BME68X_OS_4X;
	gas_conf.os_temp = BME68X_OS_8X;
	rslt = bme68x_set_conf(&gas_conf, &dev);
	fmt::print("SetConf Result: {}", rslt);

	bme68x_heatr_conf gas_heatr_conf;
	gas_heatr_conf.enable = BME68X_ENABLE;
	gas_heatr_conf.heatr_temp = 320; // 320*C
	gas_heatr_conf.heatr_dur = 150;  //  150 ms

	rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &gas_heatr_conf, &dev);

	fmt::print("SetHeaterConf Result: {}", rslt);
}

BME680::Data BME680::Sample() {
	struct bme68x_data data;
	uint8_t n_fields;

	fmt::print("Getting sensor data");

	int8_t rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &dev);
	fmt::print("GetData Result: {}", rslt);

	BME680::Data out{};
	if (n_fields) {
		out.temperature_C = data.temperature;
		out.humidity_rh = data.humidity;
		out.pressure_Pa = data.pressure;

		fmt::print("data.status 0x{:02X}", data.status);

		if (data.status & (BME68X_HEAT_STAB_MSK | BME68X_GASM_VALID_MSK)) {
			// Serial.print("Gas resistance: "); Serial.println(data.gas_resistance);
			out.gas_ohm = data.gas_resistance;
		}
	}
	return out;
}

} // namespace sensor
