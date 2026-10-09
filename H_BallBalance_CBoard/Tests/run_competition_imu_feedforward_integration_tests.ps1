$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$cmakeSource = Get-Content -Raw -LiteralPath (
    Join-Path $projectRoot 'CMakeLists.txt'
)
$configSource = Get-Content -Raw -LiteralPath (
    Join-Path $projectRoot 'Application\BallControlConfig.h'
)
$taskSource = Get-Content -Raw -LiteralPath (
    Join-Path $projectRoot 'Application\BallControlTask.cpp'
)
$publicSource = Get-Content -Raw -LiteralPath (
    Join-Path $projectRoot 'Application\task_public.h'
)
$bringupSource = Get-Content -Raw -LiteralPath (
    Join-Path $projectRoot 'Documentation\BRINGUP.md'
)

if ($cmakeSource -notmatch 'Application/CompetitionImuFeedforward\.cpp') {
    throw 'production competition IMU feedforward is not in the ARM build'
}
if ($cmakeSource -notmatch 'Application/VehicleLaunchFeedforward\.cpp') {
    throw 'vehicle launch compensation is not in the ARM build'
}
if ($cmakeSource -notmatch 'Application/ImuVehicleSnapshotStore\.cpp') {
    throw 'coherent IMU snapshot store is not in the ARM build'
}
if ($configSource -notmatch
    'kCompetitionImuFeedforwardEnabled\s*=\s*true') {
    throw 'production competition IMU feedforward is not enabled by default'
}
if ($configSource -notmatch
    'kImuFeedforwardStaleTimeoutMs\s*=\s*100U') {
    throw 'production IMU stale timeout is not 100 ms'
}
if (([regex]::Matches($taskSource,
        'ImuVehicle_TryGetControlSnapshot\(')).Count -ne 1) {
    throw 'BallControlTask must take exactly one coherent IMU snapshot per cycle'
}
if ($taskSource -notmatch
    'CompetitionImuFeedforwardController\s+competitionImuFeedforward') {
    throw 'BallControlTask does not own the production feedforward controller'
}
if ($taskSource -notmatch
    'output\.state\s*==\s*BallControlState::Balancing') {
    throw 'production feedforward is not gated by Balancing state'
}
$feedforwardCall = [regex]::Match($taskSource,
    '(?s)competitionImuFeedforward\.update\(\s*CompetitionImuFeedforwardInput\{.*?\}\);').Value
if ($feedforwardCall -notmatch '\.piSessionId\s*=\s*visionSnapshot\.pi_session_id' -or
    $feedforwardCall -notmatch '\.runId\s*=\s*taskContext\.runId') {
    throw 'launch compensation does not receive the validated run identity'
}
if ($feedforwardCall -notmatch 'input\.vision\.valid' -or
    $feedforwardCall -notmatch
        'input\.vision\.measurement_age_ms\s*<=\s*ball_control_config::kVisionMaxMeasurementAgeMs') {
    throw 'launch compensation can remain active with an unusable vision measurement'
}
$launchParameters = @(
    'kVehicleLaunchFeedforwardEnabled',
    'kVehicleLaunchAngleOffsetDeg',
    'kVehicleLaunchAccelerationThresholdMps2',
    'kVehicleLaunchRequiredNewSamples',
    'kVehicleLaunchHoldMs',
    'kVehicleLaunchReleaseMs',
    'kVehicleLaunchMaxTotalOffsetDeg'
)
foreach ($parameter in $launchParameters) {
    if ($configSource -notmatch [regex]::Escape($parameter) -or
        $taskSource -notmatch [regex]::Escape("ball_control_config::$parameter")) {
        throw "production launch configuration is not wired to $parameter"
    }
}
if ($taskSource -notmatch
    'output\.angleOffsetDeg\s*=\s*competitionFeedforwardOutput\.totalOffsetDeg') {
    throw 'PD and feedforward total is not applied to the controller output'
}
if ($taskSource -notmatch
    'output\.targetAngleRad\s*=\s*output\.mechanicalLevelAngleRad') {
    throw 'combined offset is not converted back to an absolute target'
}
$feedforwardIndex = $taskSource.IndexOf('competitionImuFeedforward.update(')
$slewIndex = $taskSource.IndexOf('installedCommandSlew.update(')
if ($feedforwardIndex -lt 0 -or $slewIndex -lt 0 -or $feedforwardIndex -gt $slewIndex) {
    throw 'combined target does not pass through installed-command slew limiting'
}

$diagnostics = @(
    'imu_feedforward_debug_configured_enabled',
    'imu_feedforward_debug_gate_reason',
    'imu_feedforward_debug_sample_fresh',
    'imu_feedforward_debug_sample_age_ms',
    'imu_feedforward_debug_pd_offset_deg',
    'imu_feedforward_debug_total_offset_deg',
    'imu_feedforward_debug_invalid_cycle_count',
    'imu_feedforward_debug_stale_cycle_count',
    'vehicle_launch_debug_state',
    'vehicle_launch_debug_active',
    'vehicle_launch_debug_offset_deg',
    'vehicle_launch_debug_elapsed_ms',
    'vehicle_launch_debug_trigger_count'
)
foreach ($variable in $diagnostics) {
    if ($taskSource -notmatch [regex]::Escape($variable)) {
        throw "BallControlTask does not publish $variable"
    }
    if ($publicSource -notmatch [regex]::Escape($variable)) {
        throw "task_public.h does not export $variable"
    }
}

$bringupContracts = @(
    'CompetitionImuFeedforward',
    'imu_feedforward_debug_configured_enabled',
    'imu_feedforward_debug_gate_reason',
    '100 ms',
    'T4/T5/T6',
    'RUN_ACTIVE',
    '3507'
)
foreach ($contract in $bringupContracts) {
    if ($bringupSource -notmatch [regex]::Escape($contract)) {
        throw "Documentation/BRINGUP.md does not document $contract"
    }
}

Write-Output 'PASS: production competition IMU feedforward integration checks'
