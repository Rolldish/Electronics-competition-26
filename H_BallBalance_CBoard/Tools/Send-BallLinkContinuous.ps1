[CmdletBinding()]
param(
    [string]$PortName = "COM18",
    [ValidateRange(1, 1000)]
    [int]$IntervalMs = 20,
    [ValidateRange(0, 86400)]
    [int]$DurationSeconds = 60
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Get-Crc16CcittFalse {
    param([byte[]]$Data)

    [uint16]$crc = 0xFFFF
    foreach ($value in $Data) {
        $crc = [uint16]($crc -bxor ([uint16]$value -shl 8))
        for ($bit = 0; $bit -lt 8; $bit++) {
            if (($crc -band 0x8000) -ne 0) {
                $crc = [uint16]((($crc -shl 1) -bxor 0x1021) -band 0xFFFF)
            } else {
                $crc = [uint16](($crc -shl 1) -band 0xFFFF)
            }
        }
    }
    return $crc
}

function New-BallLinkFrame {
    param(
        [uint32]$SessionId,
        [byte]$Sequence,
        [uint32]$TimestampMs
    )

    [byte[]]$payload =
        [BitConverter]::GetBytes($SessionId) +
        [BitConverter]::GetBytes($TimestampMs) +
        [BitConverter]::GetBytes([uint32]15000) +
        [BitConverter]::GetBytes([int16]500) +
        [BitConverter]::GetBytes([int16]-250) +
        [byte[]]@(220, 0x07) +
        [BitConverter]::GetBytes([int16]500) +
        [byte[]]@(3, 0x03) +
        [BitConverter]::GetBytes([uint16]42)

    [byte[]]$crcInput =
        [byte[]]@(0x02, 0x10, $Sequence, 0x18) + $payload
    [uint16]$crc = Get-Crc16CcittFalse -Data $crcInput

    return [byte[]](
        [byte[]]@(0xA5, 0x5A) +
        $crcInput +
        [byte[]]@(
            ($crc -band 0xFF),
            (($crc -shr 8) -band 0xFF)
        )
    )
}

$unixMilliseconds = [uint64][DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()
[uint32]$sessionId =
    [uint32]($unixMilliseconds % [uint64]4294967296)
[byte]$sequence = 0
[long]$sentFrames = 0
[long]$nextSendMs = 0
[long]$nextReportMs = 1000

$serial = [IO.Ports.SerialPort]::new(
    $PortName,
    115200,
    [IO.Ports.Parity]::None,
    8,
    [IO.Ports.StopBits]::One
)
$serial.Handshake = [IO.Ports.Handshake]::None
$serial.WriteTimeout = 1000
$serial.DtrEnable = $false
$serial.RtsEnable = $false

try {
    $serial.Open()
    $clock = [Diagnostics.Stopwatch]::StartNew()

    Write-Host "BallLink continuous sender started"
    Write-Host "Port: $PortName, interval: $IntervalMs ms, session: 0x$($sessionId.ToString('X8'))"
    Write-Host "Press Ctrl+C to stop."

    while (
        $DurationSeconds -eq 0 -or
        $clock.Elapsed.TotalSeconds -lt $DurationSeconds
    ) {
        [uint32]$timestampMs = [uint32]$clock.ElapsedMilliseconds
        [byte[]]$frame = New-BallLinkFrame `
            -SessionId $sessionId `
            -Sequence $sequence `
            -TimestampMs $timestampMs

        $serial.Write($frame, 0, $frame.Length)
        $sentFrames++
        $sequence = [byte](($sequence + 1) -band 0xFF)

        $nextSendMs += $IntervalMs
        while ($clock.ElapsedMilliseconds -lt $nextSendMs) {
            [long]$remainingMs =
                $nextSendMs - $clock.ElapsedMilliseconds
            if ($remainingMs -gt 2) {
                [Threading.Thread]::Sleep(
                    [int]($remainingMs - 1)
                )
            } else {
                [Threading.Thread]::SpinWait(100)
            }
        }

        if ($clock.ElapsedMilliseconds -ge $nextReportMs) {
            Write-Host (
                "time={0:F1}s frames={1} bytes={2}" -f
                $clock.Elapsed.TotalSeconds,
                $sentFrames,
                ($sentFrames * 32)
            )
            $nextReportMs += 1000
        }
    }

    Write-Host "Finished: frames=$sentFrames bytes=$($sentFrames * 32)"
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
}
