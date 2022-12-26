#pragma once

#include <hardware/i2c.h>

#include <optional>

namespace b1g {
namespace sensor {

class AHT21 {
public:
	AHT21(i2c_inst_t* i2c);

	struct Data {
		uint32_t ttag_ms;
		float temperature_C;
		float humidity_rh;
	};

	bool Sample(Data& data);

private:
	bool Init();

	std::optional<uint8_t> Status();

	bool is_init = false;

	i2c_inst_t* i2c_dev;
};

} // namespace sensor
} // namespace b1g
