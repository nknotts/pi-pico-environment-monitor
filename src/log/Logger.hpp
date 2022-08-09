#pragma once

#include <fmt/core.h>

#include <cstdint>

namespace log {

namespace internal {
void log_time(uint32_t ticks);
void log_time_now();
} // namespace internal

template <typename... Args>
void log(const char* level, Args&&... args) {
	internal::log_time_now();
	fmt::print(" [{}] ", level);
	fmt::print(std::forward<Args>(args)...);
	fmt::print("\n");
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
