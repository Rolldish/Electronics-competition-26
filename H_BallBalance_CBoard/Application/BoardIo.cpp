#include "BoardIo.h"

#include "main.h"

namespace {

// 控制任务每 5 ms 采样一次；连续 6 次低电平才认为按下，即约 30 ms 消抖。
constexpr uint8_t kKeyPressSamples = 6U;

uint8_t keyLowSamples{};
bool keyPressed{};

// C 板 RGB 三个颜色各由一个 GPIO 控制，高电平点亮。
void setLed(GPIO_TypeDef* const port, const uint16_t pin, const bool enabled)
{
    HAL_GPIO_WritePin(port, pin, enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

} // namespace

void BoardIo_UpdateKey()
{
    // C 板 KEY 为低电平有效：按下时读到 GPIO_PIN_RESET。
    const bool keyLow =
        HAL_GPIO_ReadPin(User_Key_GPIO_Port, User_Key_Pin) == GPIO_PIN_RESET;

    // 松手后立即清空计数，避免下一次按键沿用上一次的消抖结果。
    if (!keyLow) {
        keyLowSamples = 0U;
        keyPressed = false;
        return;
    }

    if (keyLowSamples < kKeyPressSamples) {
        ++keyLowSamples;
    }
    keyPressed = keyLowSamples >= kKeyPressSamples;
}

bool BoardIo_KeyPressed()
{
    return keyPressed;
}

void BoardIo_SetLed(const BoardLedStatus status, const uint32_t nowMs)
{
    // 每次先全部熄灭，再根据状态只打开需要的颜色。
    bool blue = false;
    bool green = false;
    bool red = false;

    switch (status) {
    case BoardLedStatus::WaitingForFeedback:
        blue = true;
        break;
    case BoardLedStatus::Ready:
        green = true;
        break;
    case BoardLedStatus::Running:
        // 250 ms 翻转一次，形成约 2 Hz 的绿灯闪烁。
        green = ((nowMs / 250U) & 1U) != 0U;
        break;
    case BoardLedStatus::Fault:
        red = true;
        break;
    default:
        red = true;
        break;
    }

    setLed(LED_B_GPIO_Port, LED_B_Pin, blue);
    setLed(LED_G_GPIO_Port, LED_G_Pin, green);
    setLed(LED_R_GPIO_Port, LED_R_Pin, red);
}
