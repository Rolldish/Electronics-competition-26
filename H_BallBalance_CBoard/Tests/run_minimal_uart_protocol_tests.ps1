$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$buildDirectory = Join-Path $projectRoot 'build\minimal-uart-tests'
$executable = Join-Path $buildDirectory 'minimal_uart_protocol_tests.exe'

New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null

$compilerCommand = Get-Command -Name 'g++' -CommandType Application -ErrorAction SilentlyContinue |
    Select-Object -First 1
if ($null -eq $compilerCommand) {
    throw 'g++ was not found. Run Tools\Enter-CBoardEnv.ps1 first.'
}

$compileArguments = @(
    '-std=c++20'
    '-Wall'
    '-Wextra'
    '-Werror'
    '-I'
    (Join-Path $projectRoot 'Application')
    (Join-Path $PSScriptRoot 'minimal_uart_protocol_tests.cpp')
    (Join-Path $projectRoot 'Application\MinimalUartProtocol.c')
    '-o'
    $executable
)

& $compilerCommand.Source @compileArguments
if ($LASTEXITCODE -ne 0) {
    throw "Minimal UART test compilation failed with exit code $LASTEXITCODE"
}

& $executable
if ($LASTEXITCODE -ne 0) {
    throw "Minimal UART tests failed with exit code $LASTEXITCODE"
}

Write-Output 'PASS: minimal UART protocol parser'
