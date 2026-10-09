#include "task_public.h"

#include "BallLinkReceiver.h"
#include "BallLinkStreamParser.h"
#include "VisionLink.h"
#include "FreeRTOS.h"
#include "main.h"
#include "task.h"
#include "usart.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

extern "C" {

volatile std::uint32_t vision_debug_status = 0U;
volatile std::int32_t vision_debug_x_0p1mm = 0;
volatile std::int32_t vision_debug_v_mmps = 0;
volatile std::int32_t vision_debug_target_x_0p1mm = 0;
volatile std::uint32_t vision_debug_confidence = 0U;
volatile std::uint32_t vision_debug_flags = 0U;
volatile std::uint32_t vision_debug_task_id = 0U;
volatile std::uint32_t vision_debug_control_flags = 0U;
volatile std::uint32_t vision_debug_run_id = 0U;
volatile std::uint32_t vision_debug_measurement_age_ms = 0U;
volatile std::uint32_t vision_debug_measurement_update_count = 0U;
volatile std::uint32_t vision_debug_session_change_count = 0U;
volatile std::uint32_t vision_debug_run_change_count = 0U;
volatile std::uint32_t vision_debug_last_session_id = 0U;
volatile std::uint32_t vision_debug_rx_byte_count = 0U;
volatile std::uint32_t vision_debug_rx_frame_count = 0U;
volatile std::uint32_t vision_debug_accepted_frame_count = 0U;
volatile std::uint32_t vision_debug_valid_measurement_count = 0U;
volatile std::uint32_t vision_debug_crc_error_count = 0U;
volatile std::uint32_t vision_debug_range_error_count = 0U;
volatile std::uint32_t vision_debug_protocol_error_count = 0U;
volatile std::uint32_t vision_debug_timestamp_error_count = 0U;
volatile std::uint32_t vision_debug_sequence_gap_count = 0U;
volatile std::uint32_t vision_debug_duplicate_count = 0U;
volatile std::uint32_t vision_debug_out_of_order_count = 0U;
volatile std::uint32_t vision_debug_timeout_count = 0U;
volatile std::uint32_t vision_debug_discarded_byte_count = 0U;
volatile std::uint32_t vision_debug_parser_overflow_count = 0U;
volatile std::uint32_t vision_debug_dma_overflow_count = 0U;
volatile std::uint32_t vision_debug_uart_error_count = 0U;
volatile std::uint32_t vision_debug_uart_last_error_code = 0U;
volatile std::uint32_t vision_debug_uart_parity_error_count = 0U;
volatile std::uint32_t vision_debug_uart_noise_error_count = 0U;
volatile std::uint32_t vision_debug_uart_frame_error_count = 0U;
volatile std::uint32_t vision_debug_uart_overrun_error_count = 0U;
volatile std::uint32_t vision_debug_uart_dma_error_count = 0U;

} // extern "C"

namespace {

constexpr std::uint32_t kTaskPollMs = 5U;
constexpr std::size_t kDmaReceiveSize = 128U;
constexpr std::size_t kRxRingCapacity = 512U;
constexpr std::size_t kRxRingMask = kRxRingCapacity - 1U;

static_assert((kRxRingCapacity & kRxRingMask) == 0U);
static_assert(std::atomic_uint32_t::is_always_lock_free);
static_assert(std::atomic_bool::is_always_lock_free);

std::array<std::uint8_t, kDmaReceiveSize> dmaReceiveBuffer{};
std::array<std::uint8_t, kRxRingCapacity> rxRing{};
std::array<std::uint32_t, kRxRingCapacity> rxArrivalMs{};
std::atomic_uint32_t rxHead{0U};
std::atomic_uint32_t rxTail{0U};
std::atomic_uint32_t rxDmaOverflowCount{0U};
std::atomic_uint32_t uartErrorCount{0U};
std::atomic_bool rxRestartRequested{false};
std::atomic_bool rxDiscardRequested{false};

BallLinkStreamParser parser;
BallLinkReceiver receiver;
VisionLinkSnapshot publishedSnapshot{};
BallLinkVisionState publishedState{BallLinkVisionState::NoLink};

void notifyVisionTaskFromIsr() {
    if (VisionCommTaskHandle == nullptr) {
        return;
    }
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(
        static_cast<TaskHandle_t>(VisionCommTaskHandle),
        &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

bool startDmaReceive() {
    if (HAL_UARTEx_ReceiveToIdle_DMA(
            &huart6, dmaReceiveBuffer.data(),
            static_cast<std::uint16_t>(
                dmaReceiveBuffer.size())) != HAL_OK) {
        return false;
    }
    __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
    return true;
}

void pushDmaBytesFromIsr(
    const std::uint16_t size,
    const std::uint32_t arrivalMs) {
    std::uint32_t head =
        rxHead.load(std::memory_order_relaxed);
    const std::uint32_t tail =
        rxTail.load(std::memory_order_acquire);
    const std::size_t boundedSize =
        std::min<std::size_t>(size, dmaReceiveBuffer.size());

    for (std::size_t index = 0U; index < boundedSize; ++index) {
        const std::uint32_t next =
            (head + 1U) & kRxRingMask;
        if (next == tail) {
            rxDmaOverflowCount.fetch_add(
                1U, std::memory_order_relaxed);
            rxDiscardRequested.store(
                true, std::memory_order_release);
            break;
        }
        rxRing[head] = dmaReceiveBuffer[index];
        rxArrivalMs[head] = arrivalMs;
        head = next;
    }
    rxHead.store(head, std::memory_order_release);
}

std::size_t popRingBytes(
    std::uint8_t *output,
    std::uint32_t *outputArrivalMs,
    const std::size_t capacity) {
    if (output == nullptr || outputArrivalMs == nullptr ||
        capacity == 0U) {
        return 0U;
    }

    std::uint32_t tail =
        rxTail.load(std::memory_order_relaxed);
    const std::uint32_t head =
        rxHead.load(std::memory_order_acquire);
    std::size_t count = 0U;
    while (tail != head && count < capacity) {
        output[count] = rxRing[tail];
        outputArrivalMs[count] = rxArrivalMs[tail];
        count += 1U;
        tail = (tail + 1U) & kRxRingMask;
    }
    rxTail.store(tail, std::memory_order_release);
    return count;
}

void recordReceiveResult(
    const BallLinkReceiveResult result) {
    switch (result) {
    case BallLinkReceiveResult::RangeError:
        vision_debug_range_error_count += 1U;
        break;
    case BallLinkReceiveResult::TimestampError:
        vision_debug_timestamp_error_count += 1U;
        break;
    case BallLinkReceiveResult::DuplicateSequence:
        vision_debug_duplicate_count += 1U;
        break;
    case BallLinkReceiveResult::OutOfOrderSequence:
        vision_debug_out_of_order_count += 1U;
        break;
    case BallLinkReceiveResult::NewMeasurement:
        vision_debug_valid_measurement_count += 1U;
        vision_debug_accepted_frame_count += 1U;
        vision_debug_sequence_gap_count +=
            receiver.missingFrameCount();
        break;
    case BallLinkReceiveResult::LegalFrame:
        vision_debug_accepted_frame_count += 1U;
        vision_debug_sequence_gap_count +=
            receiver.missingFrameCount();
        break;
    case BallLinkReceiveResult::ProtocolError:
    default:
        vision_debug_protocol_error_count += 1U;
        break;
    }
}

void parseBufferedFrames() {
    BallLinkFrame frame{};
    for (;;) {
        std::uint32_t frameArrivalMs = 0U;
        switch (parser.next(frame, frameArrivalMs)) {
        case BallLinkParseStatus::FrameReady:
            vision_debug_rx_frame_count += 1U;
            recordReceiveResult(
                receiver.process(frame, frameArrivalMs));
            break;
        case BallLinkParseStatus::ByteDiscarded:
            vision_debug_discarded_byte_count += 1U;
            break;
        case BallLinkParseStatus::CrcError:
            vision_debug_crc_error_count += 1U;
            break;
        case BallLinkParseStatus::NeedMoreData:
        default:
            return;
        }
    }
}

void parseAvailableBytes() {
    std::array<std::uint8_t, 64U> chunk{};
    std::array<std::uint32_t, 64U> chunkArrivalMs{};
    for (;;) {
        const std::size_t count =
            popRingBytes(
                chunk.data(), chunkArrivalMs.data(),
                chunk.size());
        if (count == 0U) {
            return;
        }
        vision_debug_rx_byte_count +=
            static_cast<std::uint32_t>(count);
        const std::size_t parserDropped = parser.append(
            chunk.data(), chunkArrivalMs.data(), count);
        vision_debug_parser_overflow_count +=
            static_cast<std::uint32_t>(parserDropped);
        if (parserDropped != 0U) {
            rxDiscardRequested.store(
                true, std::memory_order_release);
            return;
        }
        parseBufferedFrames();
    }
}

void publishReceiverSnapshot(const std::uint32_t nowMs) {
    const VisionLinkSnapshot snapshot = receiver.snapshot(nowMs);
    const BallLinkVisionState state = receiver.state();
    if (publishedState != BallLinkVisionState::NoLink &&
        state == BallLinkVisionState::NoLink) {
        vision_debug_timeout_count += 1U;
    }
    publishedState = state;

    taskENTER_CRITICAL();
    publishedSnapshot = snapshot;
    taskEXIT_CRITICAL();

    vision_debug_status = static_cast<std::uint32_t>(state);
    vision_debug_x_0p1mm = snapshot.position_0p1mm;
    vision_debug_v_mmps = snapshot.velocity_mmps;
    vision_debug_target_x_0p1mm =
        snapshot.target_position_0p1mm;
    vision_debug_confidence = snapshot.confidence;
    vision_debug_flags = snapshot.flags;
    vision_debug_task_id = snapshot.task_id;
    vision_debug_control_flags = snapshot.control_flags;
    vision_debug_run_id = snapshot.run_id;
    vision_debug_measurement_age_ms =
        snapshot.measurement_age_ms;
    vision_debug_measurement_update_count =
        receiver.measurementUpdateCount();
    vision_debug_session_change_count =
        receiver.sessionChangeCount();
    vision_debug_run_change_count = receiver.runChangeCount();
    vision_debug_last_session_id = snapshot.pi_session_id;
    vision_debug_dma_overflow_count =
        rxDmaOverflowCount.load(std::memory_order_relaxed);
    vision_debug_uart_error_count =
        uartErrorCount.load(std::memory_order_relaxed);
}

void recoverReceiveIfNeeded() {
    if (!rxRestartRequested.exchange(
            false, std::memory_order_acq_rel)) {
        return;
    }
    (void)HAL_UART_AbortReceive(&huart6);
    if (!startDmaReceive()) {
        rxRestartRequested.store(
            true, std::memory_order_release);
        rxDiscardRequested.store(
            true, std::memory_order_release);
    }
}

void discardPendingReceiveState() {
    if (!rxDiscardRequested.exchange(
            false, std::memory_order_acq_rel)) {
        return;
    }

    taskENTER_CRITICAL();
    rxHead.store(0U, std::memory_order_relaxed);
    rxTail.store(0U, std::memory_order_relaxed);
    taskEXIT_CRITICAL();
    parser.clear();
    receiver.invalidateLink();
    publishedState = BallLinkVisionState::NoLink;
}

} // namespace

extern "C" {

void VisionLink_GetSnapshot(VisionLinkSnapshot *output) {
    if (output == nullptr) {
        return;
    }
    taskENTER_CRITICAL();
    *output = publishedSnapshot;
    taskEXIT_CRITICAL();
}

void VisionCommTask_Entry(void *argument) {
    (void)argument;

    if (!startDmaReceive()) {
        rxRestartRequested.store(
            true, std::memory_order_release);
        rxDiscardRequested.store(
            true, std::memory_order_release);
    }

    for (;;) {
        discardPendingReceiveState();
        recoverReceiveIfNeeded();
        discardPendingReceiveState();
        parseAvailableBytes();
        discardPendingReceiveState();
        const std::uint32_t nowMs = HAL_GetTick();
        receiver.refresh(nowMs);
        publishReceiverSnapshot(nowMs);
        (void)ulTaskNotifyTake(
            pdTRUE, pdMS_TO_TICKS(kTaskPollMs));
    }
}

void HAL_UARTEx_RxEventCallback(
    UART_HandleTypeDef *huart, const std::uint16_t size) {
    if (huart != &huart6) {
        return;
    }
    pushDmaBytesFromIsr(size, HAL_GetTick());
    if (!startDmaReceive()) {
        rxRestartRequested.store(
            true, std::memory_order_release);
        rxDiscardRequested.store(
            true, std::memory_order_release);
    }
    notifyVisionTaskFromIsr();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart != &huart6) {
        // Other UART instances are intentionally not handled here;
        // their owners must provide a dedicated recovery hook.
        return;
    }

    const std::uint32_t errorCode = huart->ErrorCode;
    vision_debug_uart_last_error_code = errorCode;
    if ((errorCode & HAL_UART_ERROR_PE) != 0U) {
        vision_debug_uart_parity_error_count += 1U;
    }
    if ((errorCode & HAL_UART_ERROR_NE) != 0U) {
        vision_debug_uart_noise_error_count += 1U;
    }
    if ((errorCode & HAL_UART_ERROR_FE) != 0U) {
        vision_debug_uart_frame_error_count += 1U;
    }
    if ((errorCode & HAL_UART_ERROR_ORE) != 0U) {
        vision_debug_uart_overrun_error_count += 1U;
    }
    if ((errorCode & HAL_UART_ERROR_DMA) != 0U) {
        vision_debug_uart_dma_error_count += 1U;
    }

    uartErrorCount.fetch_add(1U, std::memory_order_relaxed);
    rxRestartRequested.store(true, std::memory_order_release);
    rxDiscardRequested.store(true, std::memory_order_release);
    notifyVisionTaskFromIsr();
}

} // extern "C"
