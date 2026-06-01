/*
    Allow easy log through RTT, only on jlink probe.
    Use J-link RTT Viewer to display on host.
    To use ST link with Jlink fw, see: https://www.segger.com/products/debug-probes/j-link/models/other-j-links/st-link-on-board/
    Add a #define LOG_LEVEL LOG_LEVEL_XXX to select log level per file.c (default LOG_LEVEL_INFO)
*/


#ifndef LOG_H
#define LOG_H

#include "SEGGER_RTT.h"

#define LOG_LEVEL_ERR  0
#define LOG_LEVEL_WARN 1
#define LOG_LEVEL_INFO 2
#define LOG_LEVEL_DBG  3

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_ERR
#endif

static inline void _log_ts(uint32_t *sec, uint32_t *ms) {
    uint32_t t = HAL_GetTick();
    *sec = t / 1000U;
    *ms  = t % 1000U;
}

#define _LOG_EMIT(color, lvl, fmt, ...) do { \
    uint32_t _s, _m; _log_ts(&_s, &_m); \
    SEGGER_RTT_printf(0, color "[%5lu.%03lu] [" lvl "] " fmt RTT_CTRL_RESET "\r\n", \
                      (unsigned long)_s, (unsigned long)_m, ##__VA_ARGS__); \
} while (0)

#if LOG_LEVEL >= LOG_LEVEL_ERR
#define LOG_ERR(fmt, ...)  _LOG_EMIT(RTT_CTRL_TEXT_BRIGHT_RED,    "E", fmt, ##__VA_ARGS__)
#else
#define LOG_ERR(fmt, ...)  ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_WARN
#define LOG_WARN(fmt, ...) _LOG_EMIT(RTT_CTRL_TEXT_BRIGHT_YELLOW, "W", fmt, ##__VA_ARGS__)
#else
#define LOG_WARN(fmt, ...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_INFO
#define LOG_INFO(fmt, ...) _LOG_EMIT("",                          "I", fmt, ##__VA_ARGS__)
#else
#define LOG_INFO(fmt, ...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_DBG
#define LOG_DBG(fmt, ...)  _LOG_EMIT(RTT_CTRL_TEXT_CYAN,          "D", fmt, ##__VA_ARGS__)
#else
#define LOG_DBG(fmt, ...)  ((void)0)
#endif

#endif /* LOG_H */