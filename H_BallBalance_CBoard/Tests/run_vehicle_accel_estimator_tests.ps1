$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$buildDirectory = Join-Path $projectRoot 'build\vehicle-accel-tests'
$executable = Join-Path $buildDirectory 'vehicle_accel_estimator_tests.exe'

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
    (Join-Path $PSScriptRoot 'vehicle_accel_estimator_tests.cpp')
    (Join-Path $projectRoot 'Application\VehicleAccelEstimator.cpp')
    '-o'
    $executable
)

& $compilerCommand.Source @compileArguments
if ($LASTEXITCODE -ne 0) {
    throw "Vehicle acceleration estimator test compilation failed with exit code $LASTEXITCODE"
}

& $executable
if ($LASTEXITCODE -ne 0) {
    throw "Vehicle acceleration estimator tests failed with exit code $LASTEXITCODE"
}
