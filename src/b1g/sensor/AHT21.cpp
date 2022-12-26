#include "AHT21.hpp"

#include <b1g/log/Logger.hpp>

#include <hardware/i2c.h>
#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <task.h>

namespace b1g {
namespace sensor {

namespace {

constexpr const uint8_t AHTX0_I2CADDR_DEFAULT = 0x38;   // Default I2C address
constexpr const uint8_t AHTX0_CMD_CALIBRATE = 0xE1;     // Calibration command
constexpr const uint8_t AHTX0_CMD_TRIGGER = 0xAC;       // Trigger reading command
constexpr const uint8_t AHTX0_CMD_SOFTRESET = 0xBA;     // Soft reset command
constexpr const uint8_t AHTX0_STATUS_BUSY = 0x80;       // Status bit for busy
constexpr const uint8_t AHTX0_STATUS_CALIBRATED = 0x08; // Status bit for calibrated

int8_t i2c_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf) {
	i2c_inst_t* dev = (i2c_inst_t*)intf;

	int num_write = i2c_write_blocking(dev, AHTX0_I2CADDR_DEFAULT, &reg_addr, 1, true);
	if (num_write != 1) {
		return -1;
	}

	int num_read = i2c_read_blocking(dev, AHTX0_I2CADDR_DEFAULT, reg_data, len, false);
	return num_read == len ? 0 : -1;
}

int8_t i2c_write(uint8_t reg_addr,
                 const uint8_t* reg_data,
                 uint32_t len,
                 void* intf) {
	i2c_inst_t* dev = (i2c_inst_t*)intf;

	uint8_t buf[64];
	uint32_t buf_len = len + 1;
	assert(buf_len <= sizeof(buf));

	// write reg_addr followed by reg_data in a single call to i2c_write_blocking
	buf[0] = reg_addr;
	std::memcpy(buf + 1, reg_data, len);
	int num_write = i2c_write_blocking(dev, AHTX0_I2CADDR_DEFAULT, buf, buf_len, false);
	return num_write == buf_len ? 0 : -1;
}

void delay_usec(uint32_t us, void* intf_ptr) {
	(void)intf_ptr; // Unused parameter
	vTaskDelay(us / 1000);
}

// https://github.com/adafruit/Adafruit_BME680/blob/master/Adafruit_BME680.cpp

} // namespace

// adapted from https://github.com/adafruit/Adafruit_CircuitPython_AHTx0/blob/main/adafruit_ahtx0.py

AHT21::AHT21(i2c_inst_t* i2c) : i2c_dev{i2c} {
	Init();
}

bool AHT21::Init() {
	uint8_t data[2] = {0x08, 0x00};
	auto ret = i2c_write(AHTX0_CMD_CALIBRATE, data, sizeof(data), i2c_dev);
	if (ret != 0) {
		return false;
	}

	auto status = Status();
	while (status.has_value() && status.value() & AHTX0_STATUS_BUSY) {
		vTaskDelay(10);
		status = Status();
	}

	if (!status) {
		return false;
	}

	is_init = status.value() & AHTX0_STATUS_CALIBRATED;
	log::info("AHT21: Initialized: {}", is_init);
	return is_init;
}

std::optional<uint8_t> AHT21::Status() {
	uint8_t data;
	int num_read = i2c_read_blocking(i2c_dev, AHTX0_I2CADDR_DEFAULT, &data, 1, false);
	if (num_read != 1) {
		return std::nullopt;
	}

	return data;
}

bool AHT21::Sample(AHT21::Data& out) {
	if (!is_init && !Init()) {
		return false;
	}

	uint8_t write_buf[2] = {0x33, 0x00};
	auto ret = i2c_write(AHTX0_CMD_TRIGGER, write_buf, sizeof(write_buf), i2c_dev);
	if (ret != 0) {
		return false;
	};

	auto status = Status();
	while (status.has_value() && status.value() & AHTX0_STATUS_BUSY) {
		vTaskDelay(80);
		status = Status();
	}

	if (!status) {
		return false;
	}

	uint8_t read_buf[6];
	int num_read = i2c_read_blocking(i2c_dev, AHTX0_I2CADDR_DEFAULT, read_buf, sizeof(read_buf), false);
	if (num_read != sizeof(read_buf)) {
		return false;
	}

	out.ttag_ms = xTaskGetTickCount();

	uint32_t humidity = (read_buf[1] << 12) | (read_buf[2] << 4) | (read_buf[3] >> 4);
	out.humidity_rh = (humidity * 100.0) / 0x100000;

	uint32_t temp = ((read_buf[3] & 0xF) << 16) | (read_buf[4] << 8) | read_buf[5];
	out.temperature_C = ((temp * 200.0) / 0x100000) - 50.0;

	return true;
}

} // namespace sensor
} // namespace b1g
