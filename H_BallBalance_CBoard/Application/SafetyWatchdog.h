#ifndef APPLICATION_SAFETY_WATCHDOG_H
#define APPLICATION_SAFETY_WATCHDOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    SAFETY_FATAL_ERROR_HANDLER = 1U,
    SAFETY_FATAL_KERNEL_INIT = 2U,
    SAFETY_FATAL_KERNEL_START = 3U,
    SAFETY_FATAL_MALLOC = 4U,
    SAFETY_FATAL_STACK_OVERFLOW = 5U,
    SAFETY_FATAL_WATCHDOG_INIT = 6U,
    SAFETY_FATAL_NMI = 7U,
    SAFETY_FATAL_HARD_FAULT = 8U,
    SAFETY_FATAL_MEMORY_FAULT = 9U,
    SAFETY_FATAL_BUS_FAULT = 10U,
    SAFETY_FATAL_USAGE_FAULT = 11U,
};

extern volatile uint32_t safety_debug_fatal_code;
extern volatile uint32_t safety_debug_watchdog_refresh_count;
extern volatile uint32_t safety_debug_reset_flags;

void SafetyWatchdog_Init(void);
void SafetyWatchdog_Refresh(void);
void SafetyWatchdog_Fatal(uint32_t code);

#ifdef __cplusplus
}
#endif

#endif
