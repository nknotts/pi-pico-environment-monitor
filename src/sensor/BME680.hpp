#pragma once

#include "bme68x.h"

#include <hardware/i2c.h>

namespace sensor {

class BME680 {
public:
	BME680(i2c_inst_t* i2c, uint8_t device_address);

	struct Data {
		float temperature_C;
		float pressure_Pa;
		float humidity_rh;
		float gas_ohm;
	};

	bool Sample(Data& data);

	struct I2cDevice {
		i2c_inst_t* i2c;
		uint8_t device_address;
	};

private:
	bool Init();

	bool is_init = false;

	bme68x_conf gas_conf;
	bme68x_heatr_conf gas_heatr_conf;
	bme68x_dev dev;
	I2cDevice i2c_dev;
};

} // namespace sensor
