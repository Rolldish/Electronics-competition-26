$ErrorActionPreference = 'Stop'

# 工程目录是 H_BallBalance_CBoard；共享工具放在工程上一级的 .tools。
$projectRoot = Split-Path -Parent $PSScriptRoot
$sharedRoot = Split-Path -Parent $projectRoot
$toolsRoot = Join-Path $sharedRoot '.tools'
$toolsLink = Get-Item -LiteralPath $toolsRoot -Force -ErrorAction SilentlyContinue
if ($null -ne $toolsLink -and $toolsLink.LinkType -eq 'Junction') {
    # 直接使用 junction 的目标路径，避免 GCC 向链接器传递中文工具链路径。
    $toolsLinkTarget = [string](@($toolsLink.Target)[0])
    $targetCompilerProbe = Join-Path $toolsLinkTarget 'winlibs-portable\mingw64\bin\g++.exe'
    if (Test-Path -LiteralPath $targetCompilerProbe -PathType Leaf) {
        $toolsRoot = $toolsLinkTarget
    }
}
$hostCompilerProbe = Join-Path $toolsRoot 'winlibs-portable\mingw64\bin\g++.exe'
if (-not (Test-Path -LiteralPath $hostCompilerProbe -PathType Leaf)) {
    # 项目移动后旧 .tools junction 可能失效；优先复用随项目保存的工具。
    $toolsRoot = Join-Path $sharedRoot 'HBall_CBoard_Tools'
}
$armBin = Join-Path $toolsRoot 'arm-gnu-toolchain-15.2.rel1-mingw-w64-x86_64-arm-none-eabi\bin'
$hostBin = Join-Path $toolsRoot 'winlibs-portable\mingw64\bin'

# CMake 和 OpenOCD 由 winget 安装，因此从当前用户的 WinGet 目录自动查找。
if ([string]::IsNullOrWhiteSpace($env:LOCALAPPDATA)) {
    throw 'LOCALAPPDATA is unavailable; CMake and OpenOCD cannot be discovered.'
}

$wingetPackages = Join-Path $env:LOCALAPPDATA 'Microsoft\WinGet\Packages'
$cmakeExecutable = Get-ChildItem `
    -Path (Join-Path $wingetPackages 'Kitware.CMake_*\cmake-*\bin\cmake.exe') `
    -File -ErrorAction SilentlyContinue |
    Sort-Object FullName -Descending |
    Select-Object -First 1
$openOcdExecutable = Get-ChildItem `
    -Path (Join-Path $wingetPackages 'xpack-dev-tools.openocd-xpack_*\xpack-openocd-*\bin\openocd.exe') `
    -File -ErrorAction SilentlyContinue |
    Sort-Object FullName -Descending |
    Select-Object -First 1

if ($null -eq $cmakeExecutable) {
    throw "CMake was not found under $wingetPackages."
}
if ($null -eq $openOcdExecutable) {
    throw "OpenOCD was not found under $wingetPackages."
}

$cmakeBin = Split-Path -Parent $cmakeExecutable.FullName
$openOcdBin = Split-Path -Parent $openOcdExecutable.FullName
$requiredTools = [ordered]@{
    'CMake' = $cmakeExecutable.FullName
    'Ninja' = Join-Path $hostBin 'ninja.exe'
    'Arm GCC' = Join-Path $armBin 'arm-none-eabi-gcc.exe'
    'Host G++' = Join-Path $hostBin 'g++.exe'
    'OpenOCD' = $openOcdExecutable.FullName
}

# 提前检查所有工具，缺少时给出明确路径，而不是等到编译中途才报错。
foreach ($tool in $requiredTools.GetEnumerator()) {
    if (-not (Test-Path -LiteralPath $tool.Value -PathType Leaf)) {
        throw "$($tool.Key) was not found at $($tool.Value)."
    }
}

# 只修改当前 PowerShell 进程的 PATH，不会永久改动 Windows 系统环境变量。
$env:PATH = @(
    $armBin
    $cmakeBin
    $openOcdBin
    $hostBin
    $env:PATH
) -join [IO.Path]::PathSeparator

Write-Host 'C-board environment is ready.' -ForegroundColor Green
Write-Host "  Project : $projectRoot"
Write-Host "  Arm GCC: $armBin"
Write-Host "  CMake  : $cmakeBin"
Write-Host "  Ninja  : $hostBin"
Write-Host "  Host G++: $hostBin"
Write-Host "  OpenOCD: $openOcdBin"
