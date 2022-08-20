#pragma once

#include <fmt/core.h>

#include <FreeRTOS.h>
#include <semphr.h>

#include <cstdint>

namespace b1g {
namespace log {

namespace internal {
void log_time(uint32_t ticks);
void log_time_now();
} // namespace internal

template <typename... Args>
void log(const char* level, Args&&... args) {
	static auto log_mutex = xSemaphoreCreateMutex();
	xSemaphoreTake(log_mutex, 1000);

	internal::log_time_now();
	fmt::print(" [{}] ", level);
	fmt::print(std::forward<Args>(args)...);
	fmt::print("\n");

	xSemaphoreGive(log_mutex);
}

template <typename... Args>
void debug(Args&&... args) {
	log("DEBUG", std::forward<Args>(args)...);
}

template <typename... Args>
void info(Args&&... args) {
	log("INFO ", std::forward<Args>(args)...);
}

template <typename... Args>
void warn(Args&&... args) {
	log("WARN ", std::forward<Args>(args)...);
}

template <typename... Args>
void error(Args&&... args) {
	log("ERROR", std::forward<Args>(args)...);
}

} // namespace log
} // namespace b1g