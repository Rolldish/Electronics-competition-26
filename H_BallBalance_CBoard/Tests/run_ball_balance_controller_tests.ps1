$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$buildDirectory = Join-Path $projectRoot 'build\ball-balance-controller-tests'
$executable = Join-Path $buildDirectory 'ball_balance_controller_tests.exe'

New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null

$compilerCommand = Get-Command -Name 'g++' -CommandType Application -ErrorAction SilentlyContinue |
    Select-Object -First 1
if ($null -eq $compilerCommand) {
    throw 'g++ was not found. Run Tools\Enter-CBoardEnv.ps1 before running ball control tests.'
}

$compileArguments = @(
    '-std=c++20'
    '-Wall'
    '-Wextra'
    '-Werror'
    '-pedantic'
    '-I'
    (Join-Path $projectRoot 'Application')
    (Join-Path $PSScriptRoot 'ball_balance_controller_tests.cpp')
    (Join-Path $projectRoot 'Application\BallBalanceController.cpp')
    (Join-Path $projectRoot 'Application\CompetitionTaskGuard.cpp')
    '-o'
    $executable
)

& $compilerCommand.Source @compileArguments
if ($LASTEXITCODE -ne 0) {
    throw "Ball balance controller test compilation failed with exit code $LASTEXITCODE"
}

& $executable
if ($LASTEXITCODE -ne 0) {
    throw "Ball balance controller tests failed with exit code $LASTEXITCODE"
}

$configSource = Get-Content -Raw -LiteralPath (
    Join-Path $projectRoot 'Application\BallControlConfig.h'
)
if ($configSource -notmatch 'kBallControlCommissioned\s*=\s*true') {
    throw 'Installed automatic ball control is not commissioned'
}
if ($configSource -notmatch 'kMechanicalLevelAngleRad\s*=\s*1\.2461F') {
    throw 'Installed mechanical level angle is not the commissioned value'
}

$taskSource = Get-Content -Raw -LiteralPath (
    Join-Path $projectRoot 'Application\BallControlTask.cpp'
)
if (([regex]::Matches($taskSource, 'motor\.snapshot\(\)')).Count -ne 1) {
    throw 'BallControlTask must take exactly one coherent QD4310 snapshot per cycle'
}
if (([regex]::Matches($taskSource, 'VisionLink_GetSnapshot\(')).Count -ne 1) {
    throw 'BallControlTask must take exactly one coherent vision snapshot per cycle'
}
if (([regex]::Matches($taskSource,
        'ImuVehicle_TryGetControlSnapshot\(')).Count -ne 1) {
    throw 'BallControlTask must take exactly one coherent IMU snapshot per cycle'
}
if (([regex]::Matches($taskSource,
        'imu_vehicle_forward_filtered_mps2')).Count -ne 2) {
    throw 'production control must not assemble IMU state from volatile globals'
}
if ($taskSource -match 'setSpeed\(\s*(?:10|100)(?:\.0*)?F?\s*\)') {
    throw 'BallControlTask still contains the old continuous RPM test command'
}
if ($taskSource -notmatch 'CompetitionTaskGuard\s+competitionTaskGuard') {
    throw 'BallControlTask does not own the competition task guard'
}
if ($taskSource -notmatch 'input\.vision\.control_flags\s*=\s*taskContext\.effectiveControlFlags') {
    throw 'BallControlTask does not apply validated control flags'
}
if ($taskSource -match 'input\.vision\.target_position_0p1mm\s*=') {
    throw 'BallControlTask must not overwrite the Pi target'
}
if ($taskSource -match 'target_position_0p1mm\s*=\s*(?:500|-500)') {
    throw 'C board must not generate the T3 target sequence'
}

$competitionDebugVariables = @(
    'competition_task_debug_state'
    'competition_task_debug_error'
    'competition_task_debug_context_valid'
    'competition_task_debug_task_id'
    'competition_task_debug_run_id'
    'competition_task_debug_target_0p1mm'
    'competition_task_debug_raw_control_flags'
    'competition_task_debug_effective_control_flags'
    'competition_task_debug_reject_count'
)
$publicHeader = Get-Content -Raw -LiteralPath (
    Join-Path $projectRoot 'Application\task_public.h'
)
foreach ($variable in $competitionDebugVariables) {
    if ($taskSource -notmatch [regex]::Escape($variable)) {
        throw "BallControlTask does not publish $variable"
    }
    if ($publicHeader -notmatch [regex]::Escape($variable)) {
        throw "task_public.h does not export $variable"
    }
}

$runtimeTuningVariables = @(
    'ball_control_tune_kp_deg_per_mm'
    'ball_control_tune_kd_deg_per_mmps'
)
foreach ($variable in $runtimeTuningVariables) {
    if ($taskSource -notmatch [regex]::Escape($variable)) {
        throw "BallControlTask does not publish runtime tuning variable $variable"
    }
    if ($publicHeader -notmatch [regex]::Escape($variable)) {
        throw "task_public.h does not export runtime tuning variable $variable"
    }
}
if ($taskSource -notmatch 'controller\.setPositionGains\(') {
    throw 'BallControlTask does not apply the runtime Kp/Kd values'
}

$cmakeSource = Get-Content -Raw -LiteralPath (
    Join-Path $projectRoot 'CMakeLists.txt'
)
if ($cmakeSource -notmatch 'Application/BallControlTask\.cpp') {
    throw 'CMake does not build the BallControlTask QD4310 owner'
}
if ($cmakeSource -match 'Application/MotorBringupTask\.cpp') {
    throw 'CMake still builds the legacy MotorBringupTask QD4310 owner'
}
if ($cmakeSource -notmatch 'Application/CompetitionTaskGuard\.cpp') {
    throw 'CMake does not build CompetitionTaskGuard'
}

Write-Output 'PASS: ball control static safety checks'
