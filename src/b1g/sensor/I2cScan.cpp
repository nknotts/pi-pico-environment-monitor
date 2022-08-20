#include "I2cScan.hpp"

#include <fmt/core.h>

namespace b1g {
namespace sensor {

void i2c_scan(i2c_inst_t* i2c) {
	for (int addr = 0; addr < (1 << 7); ++addr) {
		if (addr % 16 == 0) {
			fmt::print("{:02X} ", addr);
		}

		// Perform a 1-byte dummy read from the probe address. If a slave
		// acknowledges this address, the function returns the number of bytes
		// transferred. If the address byte is ignored, the function returns
		// -1.

		// Skip over any reserved addresses.
		uint8_t rxdata;
		int ret = i2c_read_blocking(i2c, addr, &rxdata, 1, false);

		if (ret < 0) {
			fmt::print(" .");
		} else {
			fmt::print("{:02X}", addr);
		}
		fmt::print("{}", addr % 16 == 15 ? "\n" : "  ");
	}
}

} // namespace sensor
} // namespace b1g
