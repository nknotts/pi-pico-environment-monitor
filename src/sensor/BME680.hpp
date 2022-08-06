#pragma once

#include "bme68x.h"

#include <hardware/i2c.h>

namespace sensor {

class BME680 {
public:
	BME680();

	struct Data {
		float temperature_C;
		float pressure_Pa;
		float humidity_rh;
		float gas_ohm;
	};

	Data Sample();

private:
	bme68x_dev dev;
	i2c_inst_t* i2c;
};

} // namespace sensor
