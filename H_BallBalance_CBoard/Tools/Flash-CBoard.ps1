$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
# 先加载 Arm GCC、CMake、Ninja 和 OpenOCD 的路径。
. (Join-Path $PSScriptRoot 'Enter-CBoardEnv.ps1')

# 新编译后优先烧录 Debug ELF；项目精简后使用已通过真机验证的最终固件。
$buildFirmware = 'build/Debug/H_BallBalance_CBoard.elf'
$finalFirmware = 'Firmware/H_BallBalance_CBoard_Final.elf'
$firmware = if (Test-Path -LiteralPath (Join-Path $projectRoot $buildFirmware) -PathType Leaf) {
    $buildFirmware
}
else {
    $finalFirmware
}
$firmwarePath = Join-Path $projectRoot $firmware
if (-not (Test-Path -LiteralPath $firmwarePath -PathType Leaf)) {
    throw "Firmware was not found. Build Debug or restore $finalFirmware."
}

Push-Location $projectRoot
try {
    # OpenOCD 通过 ST-Link 连接 STM32F4，随后依次编程、校验、复位并退出。
    # 使用参数数组可保证整条 program 命令作为一个 -c 参数传入。
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
