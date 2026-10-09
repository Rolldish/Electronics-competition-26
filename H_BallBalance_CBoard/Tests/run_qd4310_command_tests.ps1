$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$buildDirectory = Join-Path $projectRoot 'build\qd4310-command-tests'
$executable = Join-Path $buildDirectory 'qd4310_command_tests.exe'
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
    (Join-Path $PSScriptRoot 'Fakes')
    '-I'
    (Join-Path $projectRoot 'Libraries\QD4310')
    (Join-Path $PSScriptRoot 'qd4310_command_tests.cpp')
    (Join-Path $projectRoot 'Libraries\QD4310\QD4310.cpp')
    '-o'
    $executable
)
& $compiler.Source @arguments
if ($LASTEXITCODE -ne 0) {
    throw "QD4310 command test compilation failed with exit code $LASTEXITCODE"
}
& $executable
if ($LASTEXITCODE -ne 0) {
    throw "QD4310 command tests failed with exit code $LASTEXITCODE"
}
