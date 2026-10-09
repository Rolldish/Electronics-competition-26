$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$buildDirectory = Join-Path $projectRoot 'build\imu-feedforward-controller-tests'
$executable = Join-Path $buildDirectory 'imu_feedforward_controller_tests.exe'

New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null

$compilerCommand = Get-Command -Name 'g++' -CommandType Application -ErrorAction SilentlyContinue |
    Select-Object -First 1
if ($null -eq $compilerCommand) {
    throw 'g++ was not found. Run Tools\Enter-CBoardEnv.ps1 before running IMU feedforward tests.'
}

$compileArguments = @(
    '-std=c++20'
    '-Wall'
    '-Wextra'
    '-Werror'
    '-pedantic'
    '-I'
    (Join-Path $projectRoot 'Application')
    (Join-Path $PSScriptRoot 'imu_feedforward_controller_tests.cpp')
    (Join-Path $projectRoot 'Application\ImuFeedforwardController.cpp')
    '-o'
    $executable
)

& $compilerCommand.Source @compileArguments
if ($LASTEXITCODE -ne 0) {
    throw "IMU feedforward test compilation failed with exit code $LASTEXITCODE"
}

& $executable
if ($LASTEXITCODE -ne 0) {
    throw "IMU feedforward tests failed with exit code $LASTEXITCODE"
}
