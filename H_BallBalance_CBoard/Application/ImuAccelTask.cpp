#include "task_public.h"

#include "BMI088.h"
#include "ImuVehicleSnapshotStore.h"
#include "VehicleAccelEstimator.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "main.h"
#include "spi.h"
#include "task.h"

#include <cmath>
#include <cstdint>

namespace {

constexpr std::uint32_t kStatusInitializing = 0U;
constexpr std::uint32_t kStatusReady = 1U;
constexpr std::uint32_t kStatusInitError = 2U;
constexpr std::uint32_t kStatusRuntimeError = 3U;
constexpr std::uint32_t kInitializationRetryMs = 500U;
constexpr std::uint32_t kRuntimePollMs = 20U;
constexpr std::uint32_t kCalibrationRetrySamples = 800U;
constexpr float kGravityMps2 = 9.80665f;
constexpr float kVehicleForwardAxisX = 1.0F;
constexpr float kVehicleForwardAxisY = 0.0F;
constexpr float kVehicleForwardAxisZ = 0.0F;

BMI088 imu;

VehicleAccelConfig makeVehicleAccelConfig() {
    VehicleAccelConfig config{};
    config.forwardAxisX = kVehicleForwardAxisX;
    config.forwardAxisY = kVehicleForwardAxisY;
    config.forwardAxisZ = kVehicleForwardAxisZ;
    return config;
}

VehicleAccelEstimator vehicleAccelEstimator(
    makeVehicleAccelConfig());

void publishDriverCounters() {
    imu_debug_dma_error_count = imu.dmaErrorCount();
    imu_debug_overrun_count = imu.overrunCount();
    imu_debug_recovery_count = imu.recoveryCount();
    imu_debug_last_transfer_error_source =
        static_cast<std::uint32_t>(imu.lastTransferErrorSource());
}

void publishVehicleAcceleration() {
    const VehicleAccelSnapshot snapshot =
        vehicleAccelEstimator.snapshot();
    imu_vehicle_status =
        static_cast<std::uint32_t>(snapshot.status);
    imu_vehicle_calibrated =
        snapshot.status == VehicleAccelStatus::Ready ? 1U : 0U;
    imu_vehicle_valid = snapshot.valid ? 1U : 0U;
    imu_vehicle_calibration_samples =
        snapshot.calibrationSampleCount;
    imu_vehicle_output_count = snapshot.outputCount;
    imu_vehicle_baseline_x_mps2 = snapshot.baselineXMps2;
    imu_vehicle_baseline_y_mps2 = snapshot.baselineYMps2;
    imu_vehicle_baseline_z_mps2 = snapshot.baselineZMps2;
    imu_vehicle_noise_mps2 =
        snapshot.calibrationNoiseMps2;
    imu_vehicle_deadband_mps2 = snapshot.deadbandMps2;
    imu_vehicle_forward_raw_mps2 =
        snapshot.forwardRawMps2;
    imu_vehicle_forward_filtered_mps2 =
        snapshot.forwardFilteredMps2;
    ImuVehicle_PublishControlSnapshot(ImuVehicleControlSnapshot{
        .status = static_cast<std::uint32_t>(snapshot.status),
        .calibrated = snapshot.status == VehicleAccelStatus::Ready,
        .valid = snapshot.valid,
        .sampleCount = snapshot.outputCount,
        .forwardAccelerationMps2 = snapshot.forwardFilteredMps2,
    });
}

} // namespace

extern "C" {

volatile float imu_debug_accel_x_mps2 = 0.0f;
volatile float imu_debug_accel_y_mps2 = 0.0f;
volatile float imu_debug_accel_z_mps2 = 0.0f;
volatile float imu_debug_accel_norm_g = 0.0f;
volatile std::uint32_t imu_debug_sample_count = 0U;
volatile std::uint32_t imu_debug_status = kStatusInitializing;
volatile std::uint32_t imu_debug_driver_error = 0U;
volatile std::uint32_t imu_debug_dma_error_count = 0U;
volatile std::uint32_t imu_debug_overrun_count = 0U;
volatile std::uint32_t imu_debug_recovery_count = 0U;
volatile std::uint32_t imu_debug_last_transfer_error_source = 0U;

volatile std::uint32_t imu_vehicle_status = 0U;
volatile std::uint32_t imu_vehicle_calibrated = 0U;
volatile std::uint32_t imu_vehicle_valid = 0U;
volatile std::uint32_t imu_vehicle_calibration_samples = 0U;
volatile std::uint32_t imu_vehicle_output_count = 0U;
volatile float imu_vehicle_baseline_x_mps2 = 0.0F;
volatile float imu_vehicle_baseline_y_mps2 = 0.0F;
volatile float imu_vehicle_baseline_z_mps2 = 0.0F;
volatile float imu_vehicle_noise_mps2 = 0.0F;
volatile float imu_vehicle_deadband_mps2 = 0.0F;
volatile float imu_vehicle_forward_raw_mps2 = 0.0F;
volatile float imu_vehicle_forward_filtered_mps2 = 0.0F;

void ImuAccelTask_Entry(void *argument) {
    (void)argument;

    std::uint32_t accelGeneration = 0U;
    std::uint32_t calibrationRetrySampleCount = 0U;
    float accelMps2[3]{};

    for (;;) {
        vehicleAccelEstimator.reset();
        publishVehicleAcceleration();
        imu_debug_status = kStatusInitializing;
        imu_debug_driver_error =
            static_cast<std::uint32_t>(imu.initialize());
        publishDriverCounters();

        if (imu_debug_driver_error !=
            static_cast<std::uint32_t>(BMI088::Error::None)) {
            imu_debug_status = kStatusInitError;
            osDelay(kInitializationRetryMs);
            continue;
        }

        accelGeneration = 0U;
        calibrationRetrySampleCount = 0U;
        imu.enableRuntime();
        imu_debug_status = kStatusReady;

        while (!imu.recoveryRequested()) {
            (void)ulTaskNotifyTake(
                pdTRUE, pdMS_TO_TICKS(kRuntimePollMs));
            publishDriverCounters();

            if (imu.pollRuntimeTimeout(HAL_GetTick())) {
                break;
            }
            if (imu.recoveryRequested()) {
                break;
            }

            if (imu.readLatestAccel(accelGeneration, accelMps2)) {
                const float normMps2 = std::sqrt(
                    accelMps2[0] * accelMps2[0] +
                    accelMps2[1] * accelMps2[1] +
                    accelMps2[2] * accelMps2[2]);

                imu_debug_accel_x_mps2 = accelMps2[0];
                imu_debug_accel_y_mps2 = accelMps2[1];
                imu_debug_accel_z_mps2 = accelMps2[2];
                imu_debug_accel_norm_g = normMps2 / kGravityMps2;
                imu_debug_sample_count =
                    imu_debug_sample_count + 1U;

                (void)vehicleAccelEstimator.update(
                    accelMps2[0], accelMps2[1],
                    accelMps2[2]);
                publishVehicleAcceleration();

                if (vehicleAccelEstimator.snapshot().status ==
                    VehicleAccelStatus::CalibrationRejected) {
                    ++calibrationRetrySampleCount;
                    if (calibrationRetrySampleCount >=
                        kCalibrationRetrySamples) {
                        vehicleAccelEstimator.reset();
                        calibrationRetrySampleCount = 0U;
                        publishVehicleAcceleration();
                    }
                } else {
                    calibrationRetrySampleCount = 0U;
                }
            }
        }

        ImuVehicleControlSnapshot invalidSnapshot{};
        if (!ImuVehicle_TryGetControlSnapshot(invalidSnapshot)) {
            invalidSnapshot = ImuVehicleControlSnapshot{};
        }
        invalidSnapshot.valid = false;
        ImuVehicle_PublishControlSnapshot(invalidSnapshot);
        imu_vehicle_valid = 0U;
        imu_debug_status = kStatusRuntimeError;
        imu_debug_driver_error =
            static_cast<std::uint32_t>(BMI088::Error::Spi);
        if (!imu.stopRuntime()) {
            imu_debug_driver_error =
                static_cast<std::uint32_t>(BMI088::Error::Spi);
        }
        publishDriverCounters();
        osDelay(kInitializationRetryMs);
    }
}

void HAL_GPIO_EXTI_Callback(const std::uint16_t gpioPin) {
    if (gpioPin == INT_ACCEL_Pin || gpioPin == INT_GYRO_Pin) {
        imu.onDataReady(gpioPin);
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi != &hspi1) {
        return;
    }

    const bool accelCompleted = imu.onTransferComplete();
    if (accelCompleted && ImuAccelTaskHandle != nullptr) {
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(
            static_cast<TaskHandle_t>(ImuAccelTaskHandle),
            &higherPriorityTaskWoken);
        portYIELD_FROM_ISR(higherPriorityTaskWoken);
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    if (hspi != &hspi1) {
        return;
    }

    imu.onTransferError();
    if (ImuAccelTaskHandle != nullptr) {
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(
            static_cast<TaskHandle_t>(ImuAccelTaskHandle),
            &higherPriorityTaskWoken);
        portYIELD_FROM_ISR(higherPriorityTaskWoken);
    }
}

} // extern "C"
