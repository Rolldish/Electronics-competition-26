$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$buildDirectory = Join-Path $projectRoot 'build\vehicle-launch-trace-tests'
New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null
$executable = Join-Path $buildDirectory 'vehicle_launch_trace_tests.exe'
$compiler = Get-Command g++ -CommandType Application -ErrorAction Stop | Select-Object -First 1
& $compiler.Source '-std=c++20' '-Wall' '-Wextra' '-Werror' '-pedantic' `
    '-I' (Join-Path $projectRoot 'Application') `
    (Join-Path $PSScriptRoot 'vehicle_launch_trace_tests.cpp') `
    (Join-Path $projectRoot 'Application\VehicleLaunchTrace.cpp') '-o' $executable
if ($LASTEXITCODE -ne 0) { throw "Trace test compilation failed: $LASTEXITCODE" }
& $executable
if ($LASTEXITCODE -ne 0) { throw "Trace tests failed: $LASTEXITCODE" }
