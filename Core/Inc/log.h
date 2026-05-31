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
#define LOG_LEVEL LOG_LEVEL_DBG
#endif

#if LOG_LEVEL >= LOG_LEVEL_ERR
#define LOG_ERR(fmt, ...)  SEGGER_RTT_printf(0, RTT_CTRL_TEXT_BRIGHT_RED    "[E] " fmt RTT_CTRL_RESET "\r\n", ##__VA_ARGS__)
#else
#define LOG_ERR(fmt, ...)  ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_WARN
#define LOG_WARN(fmt, ...) SEGGER_RTT_printf(0, RTT_CTRL_TEXT_BRIGHT_YELLOW "[W] " fmt RTT_CTRL_RESET "\r\n", ##__VA_ARGS__)
#else
#define LOG_WARN(fmt, ...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_INFO
#define LOG_INFO(fmt, ...) SEGGER_RTT_printf(0, "[I] " fmt "\r\n", ##__VA_ARGS__)
#else
#define LOG_INFO(fmt, ...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_DBG
#define LOG_DBG(fmt, ...)  SEGGER_RTT_printf(0, RTT_CTRL_TEXT_CYAN          "[D] " fmt RTT_CTRL_RESET "\r\n", ##__VA_ARGS__)
#else
#define LOG_DBG(fmt, ...)  ((void)0)
#endif

#endif /* LOG_H */