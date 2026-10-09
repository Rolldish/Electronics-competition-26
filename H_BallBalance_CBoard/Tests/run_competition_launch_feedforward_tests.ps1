$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$buildDirectory = Join-Path $projectRoot 'build\competition-launch-feedforward-tests'
$executable = Join-Path $buildDirectory 'competition_launch_feedforward_tests.exe'
New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null

$compiler = Get-Command g++ -CommandType Application -ErrorAction Stop |
    Select-Object -First 1
$compileArguments = @(
    '-std=c++20', '-Wall', '-Wextra', '-Werror', '-pedantic',
    '-I', (Join-Path $projectRoot 'Application'),
    (Join-Path $PSScriptRoot 'competition_launch_feedforward_tests.cpp'),
    (Join-Path $projectRoot 'Application\CompetitionImuFeedforward.cpp'),
    (Join-Path $projectRoot 'Application\ImuFeedforwardController.cpp'),
    (Join-Path $projectRoot 'Application\VehicleLaunchFeedforward.cpp'),
    '-o', $executable
)

& $compiler.Source @compileArguments
if ($LASTEXITCODE -ne 0) {
    throw "competition launch feedforward test compilation failed: $LASTEXITCODE"
}
& $executable
if ($LASTEXITCODE -ne 0) {
    throw "competition launch feedforward tests failed: $LASTEXITCODE"
}
