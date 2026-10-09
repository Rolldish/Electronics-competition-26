$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$buildDirectory = Join-Path $projectRoot 'build\imu-vehicle-snapshot-store-tests'
$executable = Join-Path $buildDirectory 'imu_vehicle_snapshot_store_tests.exe'
New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null

$compiler = Get-Command g++ -CommandType Application -ErrorAction Stop |
    Select-Object -First 1
$compileArguments = @(
    '-std=c++20', '-Wall', '-Wextra', '-Werror', '-pedantic', '-pthread',
    '-I', (Join-Path $projectRoot 'Application'),
    (Join-Path $PSScriptRoot 'imu_vehicle_snapshot_store_tests.cpp'),
    (Join-Path $projectRoot 'Application\ImuVehicleSnapshotStore.cpp'),
    '-o', $executable
)

& $compiler.Source @compileArguments
if ($LASTEXITCODE -ne 0) {
    throw "IMU vehicle snapshot store test compilation failed: $LASTEXITCODE"
}
& $executable
if ($LASTEXITCODE -ne 0) {
    throw "IMU vehicle snapshot store tests failed: $LASTEXITCODE"
}

$taskSource = Get-Content -Raw -LiteralPath (
    Join-Path $projectRoot 'Application\ImuAccelTask.cpp'
)
if ($taskSource -notmatch '#include\s+"ImuVehicleSnapshotStore\.h"') {
    throw 'ImuAccelTask does not own the coherent control snapshot publisher'
}
if ($taskSource -notmatch
    'ImuVehicle_PublishControlSnapshot\(ImuVehicleControlSnapshot\{[\s\S]*\.sampleCount\s*=\s*snapshot\.outputCount[\s\S]*\.forwardAccelerationMps2\s*=\s*snapshot\.forwardFilteredMps2') {
    throw 'ImuAccelTask does not publish estimator generations coherently'
}
if ($taskSource -notmatch
    'invalidSnapshot\.valid\s*=\s*false;[\s\S]*ImuVehicle_PublishControlSnapshot\(invalidSnapshot\)') {
    throw 'ImuAccelTask does not invalidate the coherent snapshot during recovery'
}
Write-Output 'PASS: IMU snapshot publication static checks'
