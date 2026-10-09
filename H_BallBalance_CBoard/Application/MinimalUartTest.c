#include "MinimalUartTest.h"

#include "MinimalUartProtocol.h"
#include "main.h"
#include "usart.h"

void MinimalUartTest_InitGpio(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOH_CLK_ENABLE();
    HAL_GPIO_WritePin(
        LED_B_GPIO_Port,
        LED_B_Pin | LED_G_Pin | LED_R_Pin,
        GPIO_PIN_RESET);

    gpio.Pin = LED_B_Pin | LED_G_Pin | LED_R_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_B_GPIO_Port, &gpio);
}

void MinimalUartTest_Run(void)
{
    MinimalUartParser parser;
    MinimalUartParser_Init(&parser);

    /* Blue: test firmware is running. Green: toggles per valid PING. */
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);

    for (;;) {
        uint8_t byte = 0U;
        const HAL_StatusTypeDef receive_status =
            HAL_UART_Receive(&huart6, &byte, 1U, 1000U);

        if (receive_status == HAL_OK) {
            if (!MinimalUartParser_Push(&parser, byte)) {
                continue;
            }

            HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
            if (HAL_UART_Transmit(
                    &huart6,
                    (uint8_t *)kMinimalUartPong,
                    (uint16_t)kMinimalUartPongSize,
                    100U) != HAL_OK) {
                HAL_GPIO_WritePin(
                    LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(
                    LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
            }
        } else if (receive_status != HAL_TIMEOUT) {
            HAL_GPIO_WritePin(
                LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
            (void)HAL_UART_AbortReceive(&huart6);
            MinimalUartParser_Init(&parser);
        }
    }
}
