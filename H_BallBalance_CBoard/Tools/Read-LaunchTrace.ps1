[CmdletBinding()]
param(
    [string]$FirmwarePath,
    [string]$PythonPath,
    [switch]$PrepareOnly
)

$ErrorActionPreference = 'Stop'
$taskProjectRoot = Split-Path -Parent $PSScriptRoot
. (Join-Path $PSScriptRoot 'Enter-CBoardEnv.ps1')
if (-not $FirmwarePath) {
    $FirmwarePath = Join-Path $taskProjectRoot 'build\Debug\H_BallBalance_CBoard.elf'
}
$taskFirmware = (Resolve-Path -LiteralPath $FirmwarePath).Path
if (-not $PythonPath) {
    $PythonPath = (Get-Command python -CommandType Application -ErrorAction Stop |
        Select-Object -First 1).Source
}

# All OpenOCD filenames are generated ASCII relative paths, never user text
# interpolated into Tcl. The ELF supplies the address; no hard-coded RAM map.
$taskTag = (Get-Date -Format 'yyyyMMdd-HHmmss') + '-' + ([guid]::NewGuid().ToString('N').Substring(0, 6))
$taskRelativeDirectory = "build/launch-traces/$taskTag"
$taskOutputDirectory = Join-Path $taskProjectRoot $taskRelativeDirectory
New-Item -ItemType Directory -Force -Path $taskOutputDirectory | Out-Null
$taskToolFirmware = "$taskRelativeDirectory/firmware.elf"
$taskSnapshotFirmware = Join-Path $taskProjectRoot $taskToolFirmware
Copy-Item -LiteralPath $taskFirmware -Destination $taskSnapshotFirmware
Push-Location $taskProjectRoot
try {
$taskNmOutput = & arm-none-eabi-nm -n -S --defined-only $taskToolFirmware
if ($LASTEXITCODE -ne 0) { throw 'Cannot read firmware symbols.' }
$taskMatch = [regex]::Match(($taskNmOutput -join "`n"),
    '(?m)^([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+[bBdD]\s+vehicle_launch_trace\s*$')
if (-not $taskMatch.Success) { throw 'This ELF does not contain the onboard launch recorder. Build the diagnostic firmware first.' }
$taskAddress = [Convert]::ToUInt32($taskMatch.Groups[1].Value, 16)
$taskStorageBytes = [Convert]::ToUInt32($taskMatch.Groups[2].Value, 16)
if ($taskStorageBytes -ne 64080 -or $taskAddress -lt 0x20000000 -or
    ([uint64]$taskAddress + $taskStorageBytes) -gt 0x20020000) {
    throw 'Unexpected trace layout or SRAM address; refusing to decode a different format.'
}
$taskAddressHex = '0x{0:x8}' -f $taskAddress
$taskRecordsAddressHex = '0x{0:x8}' -f ($taskAddress + 80)
$taskExpectedBin = Join-Path $taskOutputDirectory 'expected-flash.bin'
& arm-none-eabi-objcopy -O binary $taskToolFirmware "$taskRelativeDirectory/expected-flash.bin"
if ($LASTEXITCODE -ne 0) { throw 'Cannot extract expected firmware bytes.' }
$taskFlashBytes = (Get-Item -LiteralPath $taskExpectedBin).Length
if ($taskFlashBytes -le 0 -or $taskFlashBytes -gt 1048576) { throw 'Unexpected flash image length.' }
}
finally {
    Pop-Location
}

# Explicit commands read RAM/flash; there is no halt, reset, program or target
# checksum algorithm. Disable the stock examine-end debug-register writes.
# Compare flash bytes on the host because target checksum algorithms
# can require executing code in the target's working area.
$taskOpenOcdCommands = @(
    'source [find interface/stlink.cfg]', 'source [find target/stm32f4x.cfg]',
    '$_TARGETNAME configure -event examine-end {}',
    'gdb port disabled', 'tcl port disabled', 'telnet port disabled',
    'adapter speed 1800', 'init',
    "dump_image $taskRelativeDirectory/header-before.bin $taskAddressHex 80",
    "dump_image $taskRelativeDirectory/records.bin $taskRecordsAddressHex 64000",
    "dump_image $taskRelativeDirectory/header-after.bin $taskAddressHex 80",
    "dump_image $taskRelativeDirectory/installed-flash.bin 0x08000000 $taskFlashBytes",
    'shutdown'
)
$taskOpenOcdCommands | Set-Content -LiteralPath (Join-Path $taskOutputDirectory 'read.cfg') -Encoding ascii
$taskOpenOcdArguments = @('-f', "$taskRelativeDirectory/read.cfg")
[pscustomobject]@{
    Firmware = $taskFirmware
    SnapshotFirmware = $taskSnapshotFirmware
    ExpectedElfSHA256 = (Get-FileHash -LiteralPath $taskSnapshotFirmware).Hash
    TraceAddress = $taskAddressHex
    TraceBytes = $taskStorageBytes
    OpenOcdArguments = $taskOpenOcdArguments
    OpenOcdCommands = $taskOpenOcdCommands
} | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $taskOutputDirectory 'read-plan.json') -Encoding utf8
if ($PrepareOnly) {
    Write-Output "Prepared read-only plan without connecting: $taskOutputDirectory"
    return
}

Push-Location $taskProjectRoot
try {
    $taskOpenOcdPath = (Get-Command openocd -CommandType Application -ErrorAction Stop).Source
    $taskProcess = Start-Process -FilePath $taskOpenOcdPath -ArgumentList $taskOpenOcdArguments `
        -WorkingDirectory $taskProjectRoot -WindowStyle Hidden -Wait -PassThru `
        -RedirectStandardOutput (Join-Path $taskOutputDirectory 'openocd.stdout.log') `
        -RedirectStandardError (Join-Path $taskOutputDirectory 'openocd.stderr.log')
    $taskExit = $taskProcess.ExitCode
    $taskLog = @(Get-Content -LiteralPath (Join-Path $taskOutputDirectory 'openocd.stdout.log')) +
        @(Get-Content -LiteralPath (Join-Path $taskOutputDirectory 'openocd.stderr.log'))
    $taskLog | Out-File -LiteralPath (Join-Path $taskOutputDirectory 'openocd-read.log') -Encoding utf8
    if ($taskExit -ne 0) {
        throw "ST-Link read failed ($taskExit). Check the connection. Log: $taskOutputDirectory\openocd-read.log"
    }
    $taskExpectedHash = (Get-FileHash -LiteralPath $taskExpectedBin).Hash
    $taskInstalledHash = (Get-FileHash -LiteralPath (Join-Path $taskOutputDirectory 'installed-flash.bin')).Hash
    if ($taskExpectedHash -ne $taskInstalledHash) {
        throw 'Board firmware differs from the selected ELF. Keep the ELF used to flash this trial and retry with -FirmwarePath.'
    }
    & $PythonPath (Join-Path $PSScriptRoot 'decode_launch_trace.py') `
        '--before' (Join-Path $taskOutputDirectory 'header-before.bin') `
        '--records' (Join-Path $taskOutputDirectory 'records.bin') `
        '--after' (Join-Path $taskOutputDirectory 'header-after.bin') `
        '--output' $taskOutputDirectory '--elf' $taskSnapshotFirmware '--flash-sha256' $taskInstalledHash
    if ($LASTEXITCODE -ne 0) {
        throw 'Trace is not ready. Keep the C board powered, wait at least 40 seconds from task selection, then retry. Do not select another moving task before export.'
    }
    $taskZipPath = $taskOutputDirectory + '.zip'
    Compress-Archive -LiteralPath $taskOutputDirectory -DestinationPath $taskZipPath
    Write-Output "Trial data ZIP: $taskZipPath"
}
finally {
    Pop-Location
}
