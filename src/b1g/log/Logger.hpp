#pragma once

#include <fmt/core.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include <tusb.h>

#include <cstdint>

namespace b1g {
namespace log {

template <typename... Args>
void log(const char* level, Args&&... args) {
	static char buf[CFG_TUD_CDC_TX_BUFSIZE];
	static auto log_mutex = xSemaphoreCreateMutex();
	xSemaphoreTake(log_mutex, 5);

	if (tud_cdc_n_connected(CDC_ITF_LOG)) {

		// write timestamp
		auto ticks = xTaskGetTickCount();
		uint32_t sec = ticks / 1000;
		uint16_t msec = ticks - sec * 1000;
		auto res = fmt::format_to_n(buf, sizeof(buf), "[{:>9}.{:03}]", sec, msec);
		tud_cdc_n_write(CDC_ITF_LOG, buf, res.size);

		// write level
		res = fmt::format_to_n(buf, sizeof(buf), " [{}] ", level);
		tud_cdc_n_write(CDC_ITF_LOG, buf, res.size);

		// write formatted payload
		res = fmt::format_to_n(buf, sizeof(buf), std::forward<Args>(args)...);
		tud_cdc_n_write(CDC_ITF_LOG, buf, res.size);

		// write newline
		static constexpr const char END_BUF[] = "\r\n";
		tud_cdc_n_write(CDC_ITF_LOG, END_BUF, 2);

		// flush
		tud_cdc_n_write_flush(CDC_ITF_LOG);
	}
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