#include "Logger.hpp"

#include <FreeRTOS.h>
#include <task.h>

namespace log {
namespace internal {

void log_time(uint32_t ticks) {
	uint32_t sec = ticks / 1000;
	uint16_t msec = ticks - sec * 1000;
	fmt::print("[{:>9}.{:03}]", sec, msec);
}

void log_time_now() {
	log_time(xTaskGetTickCount());
}

} // namespace internal
} // namespace log
