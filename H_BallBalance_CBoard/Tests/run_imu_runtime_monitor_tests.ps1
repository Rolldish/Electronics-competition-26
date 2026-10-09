$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$buildDirectory = Join-Path $projectRoot 'build\imu-runtime-monitor-tests'
$executable = Join-Path $buildDirectory 'imu_runtime_monitor_tests.exe'
New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null

$compiler = Get-Command -Name 'g++' -CommandType Application -ErrorAction Stop |
    Select-Object -First 1
$arguments = @(
    '-std=c++20'
    '-Wall'
    '-Wextra'
    '-Werror'
    '-pedantic'
    '-I'
    (Join-Path $projectRoot 'Application')
    (Join-Path $PSScriptRoot 'imu_runtime_monitor_tests.cpp')
    (Join-Path $projectRoot 'Application\ImuRuntimeMonitor.cpp')
    '-o'
    $executable
)
& $compiler.Source @arguments
if ($LASTEXITCODE -ne 0) {
    throw "IMU runtime monitor test compilation failed with exit code $LASTEXITCODE"
}
& $executable
if ($LASTEXITCODE -ne 0) {
    throw "IMU runtime monitor tests failed with exit code $LASTEXITCODE"
}
