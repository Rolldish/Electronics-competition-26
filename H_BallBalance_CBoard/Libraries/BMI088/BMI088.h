#ifndef BMI088_H
#define BMI088_H

#include "main.h"
#include "ImuRuntimeMonitor.h"

#include <cstdint>

class BMI088 {
public:
    enum class Error : std::uint8_t {
        None = 0,
        Spi,
        AccelNotFound,
        GyroNotFound,
        AccelSelfTest,
        GyroSelfTest,
        AccelConfig,
        GyroConfig,
    };

    enum class TransferErrorSource : std::uint32_t {
        None = 0,
        HalCallback,
        StartBusy,
        StartError,
        RecoveryFailed,
        TransferTimeout,
        SampleTimeout,
    };

    BMI088() = default;

    Error initialize();
    void enableRuntime();
    bool stopRuntime();
    bool pollRuntimeTimeout(std::uint32_t nowMs);

    void onDataReady(std::uint16_t gpioPin);
    bool onTransferComplete();
    void onTransferError();

    bool readLatestAccel(std::uint32_t &lastGeneration, float out[3]) const;

    [[nodiscard]] Error error() const { return error_; }
    [[nodiscard]] std::uint32_t dmaErrorCount() const {
        return dma_error_count_;
    }
    [[nodiscard]] std::uint32_t overrunCount() const {
        return overrun_count_;
    }
    [[nodiscard]] bool recoveryRequested() const {
        return recovery_requested_;
    }
    [[nodiscard]] std::uint32_t recoveryCount() const {
        return recovery_count_;
    }
    [[nodiscard]] TransferErrorSource lastTransferErrorSource() const {
        return last_transfer_error_source_;
    }
    [[nodiscard]] std::uint32_t lastStartStatus() const {
        return last_start_status_;
    }
    [[nodiscard]] std::uint32_t lastSpiErrorCode() const {
        return last_spi_error_code_;
    }
    [[nodiscard]] std::uint32_t lastSpiState() const {
        return last_spi_state_;
    }
    [[nodiscard]] std::uint32_t lastRxDmaErrorCode() const {
        return last_rx_dma_error_code_;
    }
    [[nodiscard]] std::uint32_t lastRxDmaState() const {
        return last_rx_dma_state_;
    }
    [[nodiscard]] std::uint32_t lastTxDmaErrorCode() const {
        return last_tx_dma_error_code_;
    }
    [[nodiscard]] std::uint32_t lastTxDmaState() const {
        return last_tx_dma_state_;
    }

private:
    enum class Transfer : std::uint8_t {
        None,
        Gyro,
        Accel,
    };

    bool readAccel(std::uint8_t reg, std::uint8_t *data, std::uint8_t length);
    bool writeAccel(std::uint8_t reg, std::uint8_t value);
    bool readGyro(std::uint8_t reg, std::uint8_t *data, std::uint8_t length);
    bool writeGyro(std::uint8_t reg, std::uint8_t value);
    bool verifyAccel(std::uint8_t reg, std::uint8_t expected);
    bool verifyGyro(std::uint8_t reg, std::uint8_t expected);
    bool synchronizeAccelAfterReset();
    bool synchronizeGyroAfterReset();
    bool accelSelfTest();
    bool gyroSelfTest();
    bool configureAccel();
    bool configureGyro();
    void captureTransferError(TransferErrorSource source,
                              HAL_StatusTypeDef startStatus);
    void startNextTransfer();
    void select(Transfer transfer);
    void deselect(Transfer transfer);

    static void delayUs(std::uint32_t microseconds);
    static void delayMs(std::uint32_t milliseconds);
    static std::int16_t decode(const std::uint8_t *bytes);

    alignas(4) std::uint8_t gyro_tx_[7]{};
    alignas(4) std::uint8_t gyro_rx_[7]{};
    alignas(4) std::uint8_t accel_tx_[8]{};
    alignas(4) std::uint8_t accel_rx_[8]{};
    std::uint8_t accel_snapshot_[6]{};

    volatile Transfer active_{Transfer::None};
    volatile bool pending_gyro_{false};
    volatile bool pending_accel_{false};
    volatile bool runtime_enabled_{false};
    volatile std::uint32_t accel_generation_{0};
    volatile std::uint32_t dma_error_count_{0};
    volatile std::uint32_t overrun_count_{0};
    volatile bool recovery_requested_{false};
    volatile std::uint32_t recovery_count_{0};
    volatile TransferErrorSource last_transfer_error_source_{
        TransferErrorSource::None};
    volatile std::uint32_t last_start_status_{HAL_OK};
    volatile std::uint32_t last_spi_error_code_{HAL_SPI_ERROR_NONE};
    volatile std::uint32_t last_spi_state_{HAL_SPI_STATE_RESET};
    volatile std::uint32_t last_rx_dma_error_code_{HAL_DMA_ERROR_NONE};
    volatile std::uint32_t last_rx_dma_state_{HAL_DMA_STATE_RESET};
    volatile std::uint32_t last_tx_dma_error_code_{HAL_DMA_ERROR_NONE};
    volatile std::uint32_t last_tx_dma_state_{HAL_DMA_STATE_RESET};
    ImuRuntimeMonitor runtime_monitor_{};
    Error error_{Error::None};
};

#endif
