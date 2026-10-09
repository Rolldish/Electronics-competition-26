$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
. (Join-Path $PSScriptRoot 'Enter-CBoardEnv.ps1')

$firmware = 'build/SimVisionMotorTest/H_BallBalance_CBoard.elf'
$firmwarePath = Join-Path $projectRoot $firmware
if (-not (Test-Path -LiteralPath $firmwarePath -PathType Leaf)) {
    throw "Simulation firmware was not found at $firmwarePath. Build SimVisionMotorTest first."
}

Push-Location $projectRoot
try {
    $openOcdArguments = @(
        '-f'
        'interface/stlink.cfg'
        '-f'
        'target/stm32f4x.cfg'
        '-c'
        "program $firmware verify reset exit"
    )
    & openocd @openOcdArguments
    if ($LASTEXITCODE -ne 0) {
        throw "OpenOCD flash failed with exit code $LASTEXITCODE"
    }
}
finally {
    Pop-Location
}
