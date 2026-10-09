$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot

$config = Get-Content -Raw -LiteralPath (
    Join-Path $projectRoot 'Core\Inc\FreeRTOSConfig.h')
if ($config -notmatch 'configCHECK_FOR_STACK_OVERFLOW\s+2') {
    throw 'FreeRTOS stack overflow level 2 checking is not enabled'
}
if ($config -notmatch 'configUSE_MALLOC_FAILED_HOOK\s+1') {
    throw 'FreeRTOS malloc failure hook is not enabled'
}

$freertos = Get-Content -Raw -LiteralPath (
    Join-Path $projectRoot 'Core\Src\freertos.c')
if ($freertos -notmatch 'vApplicationMallocFailedHook' -or
    $freertos -notmatch 'vApplicationStackOverflowHook') {
    throw 'FreeRTOS fatal hooks are missing'
}

$main = Get-Content -Raw -LiteralPath (
    Join-Path $projectRoot 'Core\Src\main.c')
if ($main -notmatch 'SafetyWatchdog_Init\(\)' -or
    $main -notmatch 'osKernelInitialize\(\)\s*!=\s*osOK' -or
    $main -notmatch 'osKernelStart\(\)\s*!=\s*osOK') {
    throw 'Kernel startup is not supervised by the safety watchdog'
}

$watchdog = Get-Content -Raw -LiteralPath (
    Join-Path $projectRoot 'Application\SafetyWatchdog.c')
if ($watchdog -notmatch 'DBGMCU_APB1_FZ_DBG_IWDG_STOP' -or
    $watchdog -notmatch 'IWDG->KR\s*=\s*IWDG_START_KEY') {
    throw 'IWDG start or debugger freeze configuration is missing'
}

$interrupts = Get-Content -Raw -LiteralPath (
    Join-Path $projectRoot 'Core\Src\stm32f4xx_it.c')
foreach ($faultCode in @(
        'SAFETY_FATAL_NMI',
        'SAFETY_FATAL_HARD_FAULT',
        'SAFETY_FATAL_MEMORY_FAULT',
        'SAFETY_FATAL_BUS_FAULT',
        'SAFETY_FATAL_USAGE_FAULT')) {
    if ($interrupts -notmatch $faultCode) {
        throw "Processor fault handler does not report $faultCode"
    }
}

$refreshLocations = Get-ChildItem -LiteralPath (
    Join-Path $projectRoot 'Application') -File -Filter '*.cpp' |
    Select-String -Pattern 'SafetyWatchdog_Refresh\('
if ($refreshLocations.Count -ne 1 -or
    $refreshLocations[0].Path -notlike '*BallControlTask.cpp') {
    throw 'Only BallControlTask may refresh the safety watchdog'
}

$cmake = Get-Content -Raw -LiteralPath (
    Join-Path $projectRoot 'CMakeLists.txt')
if ($cmake -notmatch 'Application/SafetyWatchdog\.c' -or
    $cmake -notmatch 'Application/ImuRuntimeMonitor\.cpp') {
    throw 'Safety sources are missing from the firmware build'
}

Write-Output 'PASS: RTOS and watchdog static safety checks'
