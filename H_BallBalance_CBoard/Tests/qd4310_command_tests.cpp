#include "QD4310.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {

CAN_HandleTypeDef canHandle{};
std::array<std::uint8_t, 8> lastData{};
std::uint32_t lastDlc{};
std::uint32_t frameCount{};

void resetCapture()
{
    lastData.fill(0U);
    lastDlc = 0U;
    frameCount = 0U;
}

void require(const bool condition, const char *message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void testNonFiniteMotionTargetsAreRejected()
{
    QD4310 motor(&canHandle, 0U);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();

    resetCapture();
    motor.setAngle(nan);
    motor.setStepAngle(infinity);
    motor.setSpeed(-infinity);
    motor.setLowSpeed(nan);
    require(frameCount == 0U,
            "nonfinite motion target emitted a CAN command");
    require(motor.txErrorCount() == 4U,
            "nonfinite motion targets were not all counted");
}

void testNonFiniteCurrentSendsSafeZero()
{
    QD4310 motor(&canHandle, 0U);
    resetCapture();
    motor.setCurrent(std::numeric_limits<float>::quiet_NaN());
    require(frameCount == 1U && lastDlc == 3U,
            "nonfinite current did not emit one safety command");
    require(lastData[0] == 0x03U
                && lastData[1] == 0U
                && lastData[2] == 0U,
            "nonfinite current was not encoded as 0 A");
    require(motor.txErrorCount() == 1U,
            "nonfinite current was not counted");
}

void testFiniteSpeedEncodingIsUnchanged()
{
    QD4310 motor(&canHandle, 0U);
    resetCapture();
    motor.setSpeed(1000.0F);
    require(frameCount == 1U && lastData[0] == 0x04U,
            "finite speed command was rejected");
    require(lastData[1] == 0xFFU && lastData[2] == 0x7FU,
            "finite speed command encoding changed");
}

} // namespace

extern "C" {

uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *)
{
    return 1U;
}

HAL_StatusTypeDef HAL_CAN_AddTxMessage(
    CAN_HandleTypeDef *,
    CAN_TxHeaderTypeDef *header,
    std::uint8_t data[],
    std::uint32_t *mailbox)
{
    lastDlc = header->DLC;
    for (std::uint32_t index = 0U; index < header->DLC; ++index) {
        lastData[index] = data[index];
    }
    *mailbox = 0U;
    ++frameCount;
    return HAL_OK;
}

uint32_t HAL_GetTick(void)
{
    return 0U;
}

uint32_t __get_PRIMASK(void)
{
    return 0U;
}

void __disable_irq(void)
{
}

void __enable_irq(void)
{
}

} // extern "C"

int main()
{
    try {
        testNonFiniteMotionTargetsAreRejected();
        testNonFiniteCurrentSendsSafeZero();
        testFiniteSpeedEncodingIsUnchanged();
    } catch (const std::exception& exception) {
        std::cerr << "FAIL: " << exception.what() << '\n';
        return 1;
    }
    std::cout << "PASS: QD4310 command tests\n";
    return 0;
}
