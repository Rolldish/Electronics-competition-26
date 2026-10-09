$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$buildDirectory = Join-Path $projectRoot 'build\balllink-receiver-tests'
$executable = Join-Path $buildDirectory 'balllink_receiver_tests.exe'

New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null

$compilerCommand = Get-Command -Name 'g++' -CommandType Application -ErrorAction SilentlyContinue |
    Select-Object -First 1
if ($null -eq $compilerCommand) {
    throw 'g++ was not found. Run Tools\Enter-CBoardEnv.ps1 before running host tests.'
}

$compileArguments = @(
    '-std=c++20'
    '-Wall'
    '-Wextra'
    '-Werror'
    '-I'
    (Join-Path $projectRoot 'Application')
    (Join-Path $PSScriptRoot 'balllink_receiver_tests.cpp')
    (Join-Path $projectRoot 'Application\BallLinkProtocol.cpp')
    (Join-Path $projectRoot 'Application\BallLinkStreamParser.cpp')
    (Join-Path $projectRoot 'Application\BallLinkReceiver.cpp')
    '-o'
    $executable
)

& $compilerCommand.Source @compileArguments
if ($LASTEXITCODE -ne 0) {
    throw "BallLink receiver test compilation failed with exit code $LASTEXITCODE"
}

& $executable
if ($LASTEXITCODE -ne 0) {
    throw "BallLink receiver tests failed with exit code $LASTEXITCODE"
}

$taskPath = Join-Path $projectRoot 'Application\VisionCommTask.cpp'
$taskSource = Get-Content -Raw -LiteralPath $taskPath

$forbiddenPatterns = @(
    'HAL_UART_Transmit'
    'HAL_UART_TxCpltCallback'
    'sendStatusIfDue'
    'vision_debug_tx_'
    'BallLinkCBoardStatus'
)
foreach ($pattern in $forbiddenPatterns) {
    if ($taskSource -match $pattern) {
        throw "VisionCommTask must be receive-only; forbidden pattern found: $pattern"
    }
}

$requiredPatterns = @(
    'HAL_UARTEx_ReceiveToIdle_DMA'
    'HAL_UARTEx_RxEventCallback'
    'BallLinkReceiver'
    'vision_debug_target_x_0p1mm'
    'vision_debug_task_id'
    'vision_debug_control_flags'
    'vision_debug_run_id'
    'vision_debug_measurement_update_count'
    'vision_debug_session_change_count'
    'vision_debug_run_change_count'
    'case BallLinkReceiveResult::TimestampError'
    'vision_debug_timestamp_error_count \+= 1U'
    'std::atomic_bool::is_always_lock_free'
    'rxArrivalMs'
    'pushDmaBytesFromIsr\(size, HAL_GetTick\(\)\)'
    'parser.append\(\s*chunk.data\(\), chunkArrivalMs.data\(\), count\)'
    'parser.next\(frame, frameArrivalMs\)'
    'receiver.process\(frame, frameArrivalMs\)'
    'discardPendingReceiveState'
    'parser.clear\(\)'
    'receiver.invalidateLink\(\)'
    'Other UART instances are intentionally not handled'
)
foreach ($pattern in $requiredPatterns) {
    if ($taskSource -notmatch $pattern) {
        throw "VisionCommTask is missing required receive-only integration: $pattern"
    }
}

$usartSource = Get-Content -Raw -LiteralPath (
    Join-Path $projectRoot 'Core\Src\usart.c')
$usartRequiredPatterns = @(
    'UART_MODE_TX_RX'
    'GPIO_PIN_14'
    'GPIO_AF8_USART6'
)
foreach ($pattern in $usartRequiredPatterns) {
    if ($usartSource -notmatch $pattern) {
        throw "USART6 TX hardware capability must remain configured: $pattern"
    }
}
