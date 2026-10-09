#include "BMI088.h"
#include "BMI088Registers.h"

#include "cmsis_os2.h"
#include "spi.h"

#include <cstring>

namespace {

constexpr std::uint32_t kSpiTimeoutMs = 10U;
constexpr std::uint32_t kAccelWriteDelayUs = 1000U;
constexpr std::uint32_t kRegisterSettleMs = 5U;
constexpr float kGravityMps2 = 9.80665f;
constexpr float kAccelScaleMps2 = 3.0f * kGravityMps2 / 32768.0f;

} // namespace

BMI088::Error BMI088::initialize() {
    error_ = Error::None;
    if (!stopRuntime()) {
        error_ = Error::Spi;
        return error_;
    }
    accel_generation_ = 0U;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
    delayMs(5U);

    gyro_tx_[0] = static_cast<std::uint8_t>(
        bmi088_reg::kGyroData | bmi088_reg::kReadMask);
    std::memset(gyro_tx_ + 1, 0xFF, sizeof(gyro_tx_) - 1U);
    accel_tx_[0] = static_cast<std::uint8_t>(
        bmi088_reg::kAccelData | bmi088_reg::kReadMask);
    std::memset(accel_tx_ + 1, 0xFF, sizeof(accel_tx_) - 1U);

    std::uint8_t chipId = 0U;
    if (!readAccel(bmi088_reg::kAccelChipId, &chipId, 1U) ||
        !readAccel(bmi088_reg::kAccelChipId, &chipId, 1U)) {
        error_ = Error::Spi;
        return error_;
    }
    if (chipId != bmi088_reg::kAccelChipIdValue) {
        error_ = Error::AccelNotFound;
        return error_;
    }

    if (!readGyro(bmi088_reg::kGyroChipId, &chipId, 1U) ||
        !readGyro(bmi088_reg::kGyroChipId, &chipId, 1U)) {
        error_ = Error::Spi;
        return error_;
    }
    if (chipId != bmi088_reg::kGyroChipIdValue) {
        error_ = Error::GyroNotFound;
        return error_;
    }

    if (!accelSelfTest()) {
        error_ = Error::AccelSelfTest;
        return error_;
    }
    if (!gyroSelfTest()) {
        error_ = Error::GyroSelfTest;
        return error_;
    }
    if (!configureAccel()) {
        error_ = Error::AccelConfig;
        return error_;
    }
    if (!configureGyro()) {
        error_ = Error::GyroConfig;
        return error_;
    }

    error_ = Error::None;
    return error_;
}

void BMI088::enableRuntime() {
    pending_gyro_ = false;
    pending_accel_ = false;
    active_ = Transfer::None;
    recovery_requested_ = false;
    runtime_monitor_.reset(HAL_GetTick());
    __DMB();
    runtime_enabled_ = true;
}

bool BMI088::pollRuntimeTimeout(const std::uint32_t nowMs) {
    const std::uint32_t previousPrimask = __get_PRIMASK();
    __disable_irq();
    const ImuRuntimeTimeout timeout = runtime_monitor_.poll(nowMs);
    if (timeout != ImuRuntimeTimeout::None) {
        captureTransferError(
            timeout == ImuRuntimeTimeout::Transfer
                ? TransferErrorSource::TransferTimeout
                : TransferErrorSource::SampleTimeout,
            HAL_TIMEOUT);
        pending_gyro_ = false;
        pending_accel_ = false;
        recovery_requested_ = true;
        runtime_enabled_ = false;
        __DMB();
    }
    if (previousPrimask == 0U) {
        __enable_irq();
    }
    return timeout != ImuRuntimeTimeout::None;
}

bool BMI088::stopRuntime() {
    runtime_enabled_ = false;
    __DMB();

    const bool recoveryNeeded =
        recovery_requested_ ||
        active_ != Transfer::None ||
        hspi1.State != HAL_SPI_STATE_READY ||
        hdma_spi1_rx.State != HAL_DMA_STATE_READY ||
        hdma_spi1_tx.State != HAL_DMA_STATE_READY;

    HAL_NVIC_DisableIRQ(SPI1_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Stream3_IRQn);

    pending_gyro_ = false;
    pending_accel_ = false;
    active_ = Transfer::None;
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);

    const HAL_StatusTypeDef abortStatus = HAL_SPI_Abort(&hspi1);
    bool ready =
        abortStatus == HAL_OK &&
        hspi1.State == HAL_SPI_STATE_READY &&
        hdma_spi1_rx.State == HAL_DMA_STATE_READY &&
        hdma_spi1_tx.State == HAL_DMA_STATE_READY;

    if (!ready) {
        const HAL_StatusTypeDef deinitStatus = HAL_SPI_DeInit(&hspi1);
        const HAL_StatusTypeDef initStatus =
            deinitStatus == HAL_OK ? HAL_SPI_Init(&hspi1) : HAL_ERROR;
        ready =
            deinitStatus == HAL_OK &&
            initStatus == HAL_OK &&
            hspi1.State == HAL_SPI_STATE_READY &&
            hdma_spi1_rx.State == HAL_DMA_STATE_READY &&
            hdma_spi1_tx.State == HAL_DMA_STATE_READY;
    }

    __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
    HAL_NVIC_ClearPendingIRQ(SPI1_IRQn);
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream0_IRQn);
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream3_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);

    if (!ready) {
        captureTransferError(
            TransferErrorSource::RecoveryFailed, abortStatus);
        recovery_requested_ = true;
        return false;
    }

    hspi1.ErrorCode = HAL_SPI_ERROR_NONE;
    hdma_spi1_rx.ErrorCode = HAL_DMA_ERROR_NONE;
    hdma_spi1_tx.ErrorCode = HAL_DMA_ERROR_NONE;
    recovery_requested_ = false;
    if (recoveryNeeded) {
        recovery_count_ = recovery_count_ + 1U;
    }
    return true;
}

void BMI088::onDataReady(const std::uint16_t gpioPin) {
    if (!runtime_enabled_) {
        return;
    }

    if (gpioPin == INT_GYRO_Pin) {
        if (pending_gyro_) {
            overrun_count_ = overrun_count_ + 1U;
        }
        pending_gyro_ = true;
    } else if (gpioPin == INT_ACCEL_Pin) {
        if (pending_accel_) {
            overrun_count_ = overrun_count_ + 1U;
        }
        pending_accel_ = true;
    } else {
        return;
    }

    startNextTransfer();
}

bool BMI088::onTransferComplete() {
    const Transfer completed = active_;
    if (completed == Transfer::None) {
        return false;
    }

    deselect(completed);
    runtime_monitor_.transferFinished();
    active_ = Transfer::None;
    __DMB();

    if (completed == Transfer::Accel) {
        std::memcpy(
            accel_snapshot_, accel_rx_ + 2, sizeof(accel_snapshot_));
        __DMB();
        accel_generation_ = accel_generation_ + 1U;
        runtime_monitor_.sampleReceived(HAL_GetTick());
    }

    startNextTransfer();
    return completed == Transfer::Accel;
}

void BMI088::onTransferError() {
    const Transfer failed = active_;
    if (failed != Transfer::None) {
        deselect(failed);
    }
    runtime_monitor_.transferFinished();
    active_ = Transfer::None;
    dma_error_count_ = dma_error_count_ + 1U;
    captureTransferError(TransferErrorSource::HalCallback, HAL_ERROR);
    pending_gyro_ = false;
    pending_accel_ = false;
    recovery_requested_ = true;
    __DMB();
    runtime_enabled_ = false;
}

bool BMI088::readLatestAccel(std::uint32_t &lastGeneration,
                             float out[3]) const {
    std::uint8_t bytes[6];
    const std::uint32_t previousPrimask = __get_PRIMASK();
    __disable_irq();
    const std::uint32_t generation = accel_generation_;
    if (generation == lastGeneration) {
        if (previousPrimask == 0U) {
            __enable_irq();
        }
        return false;
    }
    std::memcpy(bytes, accel_snapshot_, sizeof(bytes));
    lastGeneration = generation;
    if (previousPrimask == 0U) {
        __enable_irq();
    }

    out[0] = static_cast<float>(decode(bytes)) * kAccelScaleMps2;
    out[1] = static_cast<float>(decode(bytes + 2)) * kAccelScaleMps2;
    out[2] = static_cast<float>(decode(bytes + 4)) * kAccelScaleMps2;
    return true;
}

bool BMI088::readAccel(const std::uint8_t reg,
                       std::uint8_t *data,
                       const std::uint8_t length) {
    if (length == 0U || length > 6U) {
        return false;
    }

    std::uint8_t tx[8]{};
    std::uint8_t rx[8]{};
    tx[0] = static_cast<std::uint8_t>(reg | bmi088_reg::kReadMask);
    std::memset(tx + 1, 0xFF, static_cast<std::size_t>(length) + 1U);

    select(Transfer::Accel);
    const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        &hspi1, tx, rx, static_cast<std::uint16_t>(length + 2U),
        kSpiTimeoutMs);
    deselect(Transfer::Accel);
    delayUs(2U);

    if (status != HAL_OK) {
        return false;
    }
    std::memcpy(data, rx + 2, length);
    return true;
}

bool BMI088::writeAccel(const std::uint8_t reg,
                        const std::uint8_t value) {
    std::uint8_t tx[2]{reg, value};
    std::uint8_t rx[2]{};
    select(Transfer::Accel);
    const HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2U, kSpiTimeoutMs);
    deselect(Transfer::Accel);
    delayUs(kAccelWriteDelayUs);
    return status == HAL_OK;
}

bool BMI088::readGyro(const std::uint8_t reg,
                      std::uint8_t *data,
                      const std::uint8_t length) {
    if (length == 0U || length > 6U) {
        return false;
    }

    std::uint8_t tx[7]{};
    std::uint8_t rx[7]{};
    tx[0] = static_cast<std::uint8_t>(reg | bmi088_reg::kReadMask);
    std::memset(tx + 1, 0xFF, length);

    select(Transfer::Gyro);
    const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        &hspi1, tx, rx, static_cast<std::uint16_t>(length + 1U),
        kSpiTimeoutMs);
    deselect(Transfer::Gyro);
    delayUs(2U);

    if (status != HAL_OK) {
        return false;
    }
    std::memcpy(data, rx + 1, length);
    return true;
}

bool BMI088::writeGyro(const std::uint8_t reg,
                       const std::uint8_t value) {
    std::uint8_t tx[2]{reg, value};
    std::uint8_t rx[2]{};
    select(Transfer::Gyro);
    const HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2U, kSpiTimeoutMs);
    deselect(Transfer::Gyro);
    delayUs(2U);
    return status == HAL_OK;
}

bool BMI088::verifyAccel(const std::uint8_t reg,
                         const std::uint8_t expected) {
    std::uint8_t value = 0U;
    return readAccel(reg, &value, 1U) && value == expected;
}

bool BMI088::verifyGyro(const std::uint8_t reg,
                        const std::uint8_t expected) {
    std::uint8_t value = 0U;
    return readGyro(reg, &value, 1U) && value == expected;
}

bool BMI088::synchronizeAccelAfterReset() {
    std::uint8_t chipId = 0U;
    return readAccel(bmi088_reg::kAccelChipId, &chipId, 1U) &&
           readAccel(bmi088_reg::kAccelChipId, &chipId, 1U) &&
           chipId == bmi088_reg::kAccelChipIdValue;
}

bool BMI088::synchronizeGyroAfterReset() {
    std::uint8_t chipId = 0U;
    return readGyro(bmi088_reg::kGyroChipId, &chipId, 1U) &&
           readGyro(bmi088_reg::kGyroChipId, &chipId, 1U) &&
           chipId == bmi088_reg::kGyroChipIdValue;
}

bool BMI088::accelSelfTest() {
    if (!writeAccel(bmi088_reg::kAccelSoftReset,
                    bmi088_reg::kAccelSoftResetCommand)) {
        return false;
    }
    delayMs(80U);

    if (!synchronizeAccelAfterReset()) {
        return false;
    }

    const struct RegisterValue {
        std::uint8_t reg;
        std::uint8_t value;
    } config[] = {
        {bmi088_reg::kAccelPowerConfig, bmi088_reg::kAccelPowerActive},
        {bmi088_reg::kAccelPowerControl, bmi088_reg::kAccelEnable},
        {bmi088_reg::kAccelConfig, bmi088_reg::kAccelConfig1600HzNormal},
        {bmi088_reg::kAccelRange, bmi088_reg::kAccelRange24G},
    };

    for (const RegisterValue &entry : config) {
        if (!writeAccel(entry.reg, entry.value)) {
            return false;
        }
        delayMs(kRegisterSettleMs);
        if (!verifyAccel(entry.reg, entry.value)) {
            return false;
        }
    }

    std::int16_t response[2][3]{};
    const std::uint8_t selfTestValues[2]{
        bmi088_reg::kAccelSelfTestPositive,
        bmi088_reg::kAccelSelfTestNegative,
    };

    for (std::uint8_t test = 0U; test < 2U; ++test) {
        if (!writeAccel(bmi088_reg::kAccelSelfTest, selfTestValues[test])) {
            return false;
        }
        delayMs(80U);

        std::uint8_t data[6]{};
        if (!readAccel(bmi088_reg::kAccelData, data, sizeof(data))) {
            return false;
        }
        response[test][0] = decode(data);
        response[test][1] = decode(data + 2);
        response[test][2] = decode(data + 4);
    }

    const bool passed =
        (static_cast<std::int32_t>(response[0][0]) - response[1][0] >= 1365) &&
        (static_cast<std::int32_t>(response[0][1]) - response[1][1] >= 1365) &&
        (static_cast<std::int32_t>(response[0][2]) - response[1][2] >= 680);

    const bool stopped = writeAccel(
        bmi088_reg::kAccelSelfTest, bmi088_reg::kAccelSelfTestOff);
    delayMs(80U);
    const bool reset = writeAccel(
        bmi088_reg::kAccelSoftReset, bmi088_reg::kAccelSoftResetCommand);
    delayMs(80U);
    return passed && stopped && reset;
}

bool BMI088::gyroSelfTest() {
    if (!writeGyro(bmi088_reg::kGyroSoftReset,
                   bmi088_reg::kGyroSoftResetCommand)) {
        return false;
    }
    delayMs(80U);

    if (!synchronizeGyroAfterReset() ||
        !writeGyro(bmi088_reg::kGyroSelfTest,
                   bmi088_reg::kGyroSelfTestTrigger)) {
        return false;
    }

    for (std::uint8_t retry = 0U; retry < 10U; ++retry) {
        delayMs(10U);
        std::uint8_t result = 0U;
        if (!readGyro(bmi088_reg::kGyroSelfTest, &result, 1U)) {
            return false;
        }
        if ((result & bmi088_reg::kGyroSelfTestReady) != 0U) {
            return (result & bmi088_reg::kGyroSelfTestFailed) == 0U;
        }
    }
    return false;
}

bool BMI088::configureAccel() {
    if (!writeAccel(bmi088_reg::kAccelSoftReset,
                    bmi088_reg::kAccelSoftResetCommand)) {
        return false;
    }
    delayMs(80U);

    if (!synchronizeAccelAfterReset()) {
        return false;
    }

    const struct RegisterValue {
        std::uint8_t reg;
        std::uint8_t value;
    } config[] = {
        {bmi088_reg::kAccelPowerConfig, bmi088_reg::kAccelPowerActive},
        {bmi088_reg::kAccelPowerControl, bmi088_reg::kAccelEnable},
        {bmi088_reg::kAccelConfig, bmi088_reg::kAccelConfig800HzNormal},
        {bmi088_reg::kAccelRange, bmi088_reg::kAccelRange3G},
        {bmi088_reg::kAccelInt1IoControl,
         bmi088_reg::kAccelInt1PushPullActiveLow},
        {bmi088_reg::kAccelIntMapData,
         bmi088_reg::kAccelMapDataReadyToInt1},
    };

    for (const RegisterValue &entry : config) {
        if (!writeAccel(entry.reg, entry.value)) {
            return false;
        }
        delayMs(kRegisterSettleMs);
        if (!verifyAccel(entry.reg, entry.value)) {
            return false;
        }
    }
    return true;
}

bool BMI088::configureGyro() {
    if (!writeGyro(bmi088_reg::kGyroSoftReset,
                   bmi088_reg::kGyroSoftResetCommand)) {
        return false;
    }
    delayMs(80U);

    if (!synchronizeGyroAfterReset()) {
        return false;
    }

    const struct RegisterValue {
        std::uint8_t reg;
        std::uint8_t value;
    } config[] = {
        {bmi088_reg::kGyroRange, bmi088_reg::kGyroRange2000Dps},
        {bmi088_reg::kGyroBandwidth, bmi088_reg::kGyro1000Hz116Hz},
        {bmi088_reg::kGyroPowerMode, bmi088_reg::kGyroNormalMode},
        {bmi088_reg::kGyroControl, bmi088_reg::kGyroDataReadyEnable},
        {bmi088_reg::kGyroInt3Int4IoConfig,
         bmi088_reg::kGyroInt3PushPullActiveLow},
        {bmi088_reg::kGyroInt3Int4IoMap,
         bmi088_reg::kGyroMapDataReadyToInt3},
    };

    for (const RegisterValue &entry : config) {
        if (!writeGyro(entry.reg, entry.value)) {
            return false;
        }
        delayMs(kRegisterSettleMs);
        if (!verifyGyro(entry.reg, entry.value)) {
            return false;
        }
    }
    return true;
}

void BMI088::captureTransferError(
    const TransferErrorSource source,
    const HAL_StatusTypeDef startStatus) {
    last_transfer_error_source_ = source;
    last_start_status_ = static_cast<std::uint32_t>(startStatus);
    last_spi_error_code_ = hspi1.ErrorCode;
    last_spi_state_ = static_cast<std::uint32_t>(hspi1.State);
    last_rx_dma_error_code_ = hdma_spi1_rx.ErrorCode;
    last_rx_dma_state_ = static_cast<std::uint32_t>(hdma_spi1_rx.State);
    last_tx_dma_error_code_ = hdma_spi1_tx.ErrorCode;
    last_tx_dma_state_ = static_cast<std::uint32_t>(hdma_spi1_tx.State);
}

void BMI088::startNextTransfer() {
    if (!runtime_enabled_ || active_ != Transfer::None ||
        hspi1.State != HAL_SPI_STATE_READY ||
        hdma_spi1_rx.State != HAL_DMA_STATE_READY ||
        hdma_spi1_tx.State != HAL_DMA_STATE_READY) {
        return;
    }

    Transfer next = Transfer::None;
    std::uint8_t *tx = nullptr;
    std::uint8_t *rx = nullptr;
    std::uint16_t length = 0U;

    if (pending_gyro_) {
        pending_gyro_ = false;
        next = Transfer::Gyro;
        tx = gyro_tx_;
        rx = gyro_rx_;
        length = sizeof(gyro_tx_);
    } else if (pending_accel_) {
        pending_accel_ = false;
        next = Transfer::Accel;
        tx = accel_tx_;
        rx = accel_rx_;
        length = sizeof(accel_tx_);
    } else {
        return;
    }

    active_ = next;
    runtime_monitor_.transferStarted(HAL_GetTick());
    select(next);
    const HAL_StatusTypeDef startStatus =
        HAL_SPI_TransmitReceive_DMA(&hspi1, tx, rx, length);
    if (startStatus != HAL_OK) {
        deselect(next);
        runtime_monitor_.transferFinished();
        active_ = Transfer::None;
        pending_gyro_ = false;
        pending_accel_ = false;
        dma_error_count_ = dma_error_count_ + 1U;
        captureTransferError(
            startStatus == HAL_BUSY
                ? TransferErrorSource::StartBusy
                : TransferErrorSource::StartError,
            startStatus);
        recovery_requested_ = true;
        __DMB();
        runtime_enabled_ = false;
    }
}

void BMI088::select(const Transfer transfer) {
    if (transfer == Transfer::Accel) {
        HAL_GPIO_WritePin(
            CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
    } else if (transfer == Transfer::Gyro) {
        HAL_GPIO_WritePin(
            CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
    }
}

void BMI088::deselect(const Transfer transfer) {
    if (transfer == Transfer::Accel) {
        HAL_GPIO_WritePin(
            CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
    } else if (transfer == Transfer::Gyro) {
        HAL_GPIO_WritePin(
            CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
    }
}

void BMI088::delayUs(const std::uint32_t microseconds) {
    if (microseconds == 0U) {
        return;
    }
    const std::uint32_t cyclesPerMicrosecond = SystemCoreClock / 1000000U;
    const std::uint32_t start = DWT->CYCCNT;
    const std::uint32_t cycles = microseconds * cyclesPerMicrosecond;
    while (static_cast<std::uint32_t>(DWT->CYCCNT - start) < cycles) {
        __NOP();
    }
}

void BMI088::delayMs(const std::uint32_t milliseconds) {
    if (osKernelGetState() == osKernelRunning) {
        osDelay(milliseconds);
    } else {
        HAL_Delay(milliseconds);
    }
}

std::int16_t BMI088::decode(const std::uint8_t *bytes) {
    const std::uint16_t value =
        static_cast<std::uint16_t>(bytes[0]) |
        static_cast<std::uint16_t>(
            static_cast<std::uint16_t>(bytes[1]) << 8U);
    return static_cast<std::int16_t>(value);
}
