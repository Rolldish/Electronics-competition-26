#include "SafetyWatchdog.h"

#include "main.h"

volatile uint32_t safety_debug_fatal_code = 0U;
volatile uint32_t safety_debug_watchdog_refresh_count = 0U;
volatile uint32_t safety_debug_reset_flags = 0U;

enum {
    IWDG_WRITE_ACCESS_KEY = 0x5555U,
    IWDG_RELOAD_KEY = 0xAAAAU,
    IWDG_START_KEY = 0xCCCCU,
    IWDG_PRESCALER_DIV64 = 4U,
    IWDG_RELOAD_500MS_NOMINAL = 249U,
    IWDG_REGISTER_WAIT_LIMIT = 1000000U,
};

static uint8_t watchdog_started;

static void setFatalLed(void)
{
    if ((RCC->AHB1ENR & RCC_AHB1ENR_GPIOHEN) == 0U) {
        return;
    }
    HAL_GPIO_WritePin(
        LED_B_GPIO_Port, LED_B_Pin | LED_G_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
}

void SafetyWatchdog_Init(void)
{
    uint32_t waitRemaining = IWDG_REGISTER_WAIT_LIMIT;

    safety_debug_reset_flags = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF;
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
    IWDG->KR = IWDG_START_KEY;
    watchdog_started = 1U;
    IWDG->KR = IWDG_WRITE_ACCESS_KEY;
    IWDG->PR = IWDG_PRESCALER_DIV64;
    IWDG->RLR = IWDG_RELOAD_500MS_NOMINAL;

    while (IWDG->SR != 0U && waitRemaining > 0U) {
        --waitRemaining;
    }
    if (waitRemaining == 0U) {
        SafetyWatchdog_Fatal(SAFETY_FATAL_WATCHDOG_INIT);
    }

    IWDG->KR = IWDG_RELOAD_KEY;
}

void SafetyWatchdog_Refresh(void)
{
    if (watchdog_started == 0U || safety_debug_fatal_code != 0U) {
        return;
    }
    IWDG->KR = IWDG_RELOAD_KEY;
    ++safety_debug_watchdog_refresh_count;
}

void SafetyWatchdog_Fatal(const uint32_t code)
{
    if (safety_debug_fatal_code == 0U) {
        safety_debug_fatal_code = code;
    }
    setFatalLed();
    __disable_irq();
    for (;;) {
        /*
         * IWDG 已启动时这里会自动复位；若故障发生在启动前，
         * 保持红灯和停机现场，等待调试器定位。
         */
    }
}
