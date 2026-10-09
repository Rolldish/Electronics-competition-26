#ifndef BMI088_REGISTERS_H
#define BMI088_REGISTERS_H

#include <cstdint>

namespace bmi088_reg {

constexpr std::uint8_t kReadMask = 0x80U;

constexpr std::uint8_t kAccelChipId = 0x00U;
constexpr std::uint8_t kAccelChipIdValue = 0x1EU;
constexpr std::uint8_t kAccelData = 0x12U;
constexpr std::uint8_t kAccelConfig = 0x40U;
constexpr std::uint8_t kAccelRange = 0x41U;
constexpr std::uint8_t kAccelInt1IoControl = 0x53U;
constexpr std::uint8_t kAccelIntMapData = 0x58U;
constexpr std::uint8_t kAccelSelfTest = 0x6DU;
constexpr std::uint8_t kAccelPowerConfig = 0x7CU;
constexpr std::uint8_t kAccelPowerControl = 0x7DU;
constexpr std::uint8_t kAccelSoftReset = 0x7EU;

constexpr std::uint8_t kAccelSoftResetCommand = 0xB6U;
constexpr std::uint8_t kAccelConfig800HzNormal = 0xABU;
constexpr std::uint8_t kAccelConfig1600HzNormal = 0xACU;
constexpr std::uint8_t kAccelRange3G = 0x00U;
constexpr std::uint8_t kAccelRange24G = 0x03U;
constexpr std::uint8_t kAccelPowerActive = 0x00U;
constexpr std::uint8_t kAccelEnable = 0x04U;
constexpr std::uint8_t kAccelInt1PushPullActiveLow = 0x08U;
constexpr std::uint8_t kAccelMapDataReadyToInt1 = 0x04U;
constexpr std::uint8_t kAccelSelfTestOff = 0x00U;
constexpr std::uint8_t kAccelSelfTestPositive = 0x0DU;
constexpr std::uint8_t kAccelSelfTestNegative = 0x09U;

constexpr std::uint8_t kGyroChipId = 0x00U;
constexpr std::uint8_t kGyroChipIdValue = 0x0FU;
constexpr std::uint8_t kGyroData = 0x02U;
constexpr std::uint8_t kGyroRange = 0x0FU;
constexpr std::uint8_t kGyroBandwidth = 0x10U;
constexpr std::uint8_t kGyroPowerMode = 0x11U;
constexpr std::uint8_t kGyroSoftReset = 0x14U;
constexpr std::uint8_t kGyroControl = 0x15U;
constexpr std::uint8_t kGyroInt3Int4IoConfig = 0x16U;
constexpr std::uint8_t kGyroInt3Int4IoMap = 0x18U;
constexpr std::uint8_t kGyroSelfTest = 0x3CU;

constexpr std::uint8_t kGyroSoftResetCommand = 0xB6U;
constexpr std::uint8_t kGyroRange2000Dps = 0x00U;
constexpr std::uint8_t kGyro1000Hz116Hz = 0x82U;
constexpr std::uint8_t kGyroNormalMode = 0x00U;
constexpr std::uint8_t kGyroDataReadyEnable = 0x80U;
constexpr std::uint8_t kGyroInt3PushPullActiveLow = 0x00U;
constexpr std::uint8_t kGyroMapDataReadyToInt3 = 0x01U;
constexpr std::uint8_t kGyroSelfTestTrigger = 0x01U;
constexpr std::uint8_t kGyroSelfTestReady = 0x02U;
constexpr std::uint8_t kGyroSelfTestFailed = 0x04U;

} // namespace bmi088_reg

#endif
