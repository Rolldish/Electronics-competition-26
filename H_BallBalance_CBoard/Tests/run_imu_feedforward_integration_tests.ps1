$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$cmakeSource = Get-Content -Raw -LiteralPath (
    Join-Path $projectRoot 'CMakeLists.txt'
)
$presetSource = Get-Content -Raw -LiteralPath (
    Join-Path $projectRoot 'CMakePresets.json'
)
$taskSource = Get-Content -Raw -LiteralPath (
    Join-Path $projectRoot 'Application\BallControlTask.cpp'
)
$publicSource = Get-Content -Raw -LiteralPath (
    Join-Path $projectRoot 'Application\task_public.h'
)

if ($cmakeSource -notmatch 'HBALL_IMU_FEEDFORWARD_TEST') {
    throw 'CMake does not define the standalone IMU feedforward mode'
}
if ($cmakeSource -notmatch 'Application/ImuFeedforwardController\.cpp') {
    throw 'Firmware does not compile the IMU feedforward controller'
}
if ($presetSource -notmatch '"name"\s*:\s*"ImuFeedforwardTest"') {
    throw 'CMake preset ImuFeedforwardTest is missing'
}
if ($taskSource -notmatch
    'imuFeedforwardController\.update\(\s*imu_vehicle_valid\s*!=\s*0U,\s*imu_vehicle_forward_filtered_mps2\)') {
    throw 'BallControlTask does not consume the filtered vehicle acceleration'
}
if ($taskSource -notmatch
    'desiredAngleRad\s*=\s*imuFeedforwardOutput\.targetAngleRad') {
    throw 'IMU feedforward target is not connected to the motor command'
}
if ($publicSource -notmatch 'ball_control_debug_imu_feedforward_mode') {
    throw 'IMU feedforward mode is not visible in Live Watch'
}
if ($publicSource -notmatch 'imu_feedforward_debug_angle_offset_deg') {
    throw 'IMU feedforward angle offset is not visible in Live Watch'
}
if ($taskSource -notmatch
    'imuFeedforwardPeakHold\.observe\(imuFeedforwardOutput\)') {
    throw 'BallControlTask does not update IMU feedforward peak hold'
}
if ($taskSource -notmatch '#if defined\(HBALL_IMU_FEEDFORWARD_TEST\)') {
    throw 'standalone IMU feedforward build was removed'
}
if ($taskSource -notmatch 'competitionImuFeedforward\.update\(') {
    throw 'production competition feedforward path is missing'
}
foreach ($peakVariable in @(
    'imu_feedforward_debug_valid_sample_count',
    'imu_feedforward_debug_max_accel_mps2',
    'imu_feedforward_debug_min_accel_mps2',
    'imu_feedforward_debug_max_angle_offset_deg',
    'imu_feedforward_debug_min_angle_offset_deg',
    'imu_feedforward_debug_min_target_angle_rad',
    'imu_feedforward_debug_max_target_angle_rad'
)) {
    if ($publicSource -notmatch $peakVariable) {
        throw "IMU feedforward peak variable $peakVariable is not visible in Live Watch"
    }
}

Write-Output 'PASS: IMU feedforward integration static checks'
