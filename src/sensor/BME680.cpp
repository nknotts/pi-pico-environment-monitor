#include "BME680.hpp"

#include <log/Logger.hpp>

#include <hardware/i2c.h>
#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <task.h>

namespace sensor {

namespace {

int8_t i2c_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf) {
	BME680::I2cDevice* dev = (BME680::I2cDevice*)intf;

	int num_write = i2c_write_blocking(dev->i2c, dev->device_address, &reg_addr, 1, true);
	if (num_write != 1) {
		return -1;
	}

	int num_read = i2c_read_blocking(dev->i2c, dev->device_address, reg_data, len, false);
	return num_read == len ? 0 : -1;
}

int8_t i2c_write(uint8_t reg_addr,
                 const uint8_t* reg_data,
                 uint32_t len,
                 void* intf) {
	BME680::I2cDevice* dev = (BME680::I2cDevice*)intf;
	uint8_t buf[64];
	uint32_t buf_len = len + 1;
	assert(buf_len <= sizeof(buf));

	// write reg_addr followed by reg_data in a single call to i2c_write_blocking
	buf[0] = reg_addr;
	std::memcpy(buf + 1, reg_data, len);
	int num_write = i2c_write_blocking(dev->i2c, dev->device_address, buf, buf_len, false);
	return num_write == buf_len ? 0 : -1;
}

void delay_usec(uint32_t us, void* intf_ptr) {
	(void)intf_ptr; // Unused parameter
	vTaskDelay(us / 1000);
}

// https://github.com/adafruit/Adafruit_BME680/blob/master/Adafruit_BME680.cpp

} // namespace

BME680::BME680(i2c_inst_t* i2c, uint8_t device_address) : i2c_dev{i2c, device_address} {
	dev.chip_id = i2c_dev.device_address;
	dev.intf = BME68X_I2C_INTF;
	dev.intf_ptr = (void*)&i2c_dev;
	dev.read = &i2c_read;
	dev.write = &i2c_write;
	dev.delay_us = &delay_usec;
	dev.amb_temp = 25; /* The ambient temperature in deg C is used for
	                         defining the heater temperature */

	Init();
}

bool BME680::Init() {
	int8_t ret = bme68x_init(&dev);
	is_init = (ret == BME68X_OK);
	if (!is_init) {
		log::error("BME680: bme68x_init failed ({})", ret);
		return is_init;
	}

	gas_conf.filter = BME68X_FILTER_SIZE_3;
	gas_conf.odr = BME68X_ODR_NONE;
	gas_conf.os_hum = BME68X_OS_2X;
	gas_conf.os_pres = BME68X_OS_4X;
	gas_conf.os_temp = BME68X_OS_8X;
	ret = bme68x_set_conf(&gas_conf, &dev);
	is_init = (ret == BME68X_OK);
	if (!is_init) {
		log::error("BME680: bme68x_set_conf failed ({})", ret);
		return is_init;
	}

	gas_heatr_conf.enable = BME68X_ENABLE;
	gas_heatr_conf.heatr_temp = 320;
	gas_heatr_conf.heatr_dur = 150;

	ret = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &gas_heatr_conf, &dev);
	is_init = (ret == BME68X_OK);
	if (!is_init) {
		log::error("BME680: bme68x_set_heatr_conf failed ({})", ret);
		return is_init;
	}

	log::info("BME680: Successfully initialized");
	return is_init;
}

bool BME680::Sample(BME680::Data& out) {
	if (!is_init && !Init()) {
		return false;
	}

	struct bme68x_data data;
	uint8_t n_fields;

	int8_t ret = bme68x_set_op_mode(BME68X_FORCED_MODE, &dev);
	if (ret != BME68X_OK) {
		log::info("BME680: bme68x_set_op_mode failed: {}", ret);
		return false;
	}

	uint32_t delay_us = bme68x_get_meas_dur(BME68X_FORCED_MODE, &gas_conf, &dev) + (gas_heatr_conf.heatr_dur * 1000);
	dev.delay_us(delay_us, dev.intf_ptr);

	ret = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &dev);
	if (ret != BME68X_OK) {
		log::info("BME680: bme68x_get_data failed: {}", ret);
		return false;
	}

	if (n_fields <= 0) {
		log::info("BME680: n_fields error: {}", n_fields);
		return false;
	}

	out.ttag_ms = xTaskGetTickCount();
	out.temperature_C = data.temperature;
	out.humidity_rh = data.humidity;
	out.pressure_Pa = data.pressure;
	if (data.status & (BME68X_HEAT_STAB_MSK | BME68X_GASM_VALID_MSK)) {
		out.gas_ohm = data.gas_resistance;
	} else {
		out.gas_ohm = 0.0;
	}

	return true;
}

} // namespace sensor
