$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$buildDirectory = Join-Path $projectRoot 'build\competition-task-guard-tests'
$executable = Join-Path $buildDirectory 'competition_task_guard_tests.exe'
New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null

$compiler = Get-Command g++ -CommandType Application -ErrorAction Stop |
    Select-Object -First 1
$arguments = @(
    '-std=c++20', '-Wall', '-Wextra', '-Werror', '-pedantic',
    '-I', (Join-Path $projectRoot 'Application'),
    (Join-Path $PSScriptRoot 'competition_task_guard_tests.cpp'),
    (Join-Path $projectRoot 'Application\CompetitionTaskGuard.cpp'),
    '-o', $executable
)
& $compiler.Source @arguments
if ($LASTEXITCODE -ne 0) { throw "competition task guard compilation failed: $LASTEXITCODE" }
& $executable
if ($LASTEXITCODE -ne 0) { throw "competition task guard tests failed: $LASTEXITCODE" }

$parentRoot = Split-Path -Parent $projectRoot
$documents = @()
foreach ($prefix in @('00_26', '01_', '02_', '03_')) {
    $matches = @(Get-ChildItem -LiteralPath $parentRoot -File |
        Where-Object { $_.Name -like "$prefix*.md" })
    if ($matches.Count -ne 1) {
        throw "Expected exactly one documentation file with prefix $prefix"
    }
    $documents += $matches[0].FullName
}
$documents += Join-Path $projectRoot 'Documentation\BRINGUP.md'
foreach ($document in $documents) {
    $documentSource = Get-Content -Raw -Encoding UTF8 -LiteralPath $document
    if ($documentSource -notmatch 'CompetitionTaskGuard') {
        throw "CompetitionTaskGuard documentation is missing: $document"
    }
    if ($documentSource -notmatch 'Pi \u8d1f\u8d23\u76ee\u6807\u548c\u4efb\u52a1\u6d41\u7a0b') {
        throw "Pi target-flow ownership is missing: $document"
    }
    if ($documentSource -notmatch 'C \u677f\u4e0d\u751f\u6210 T3 \u76ee\u6807') {
        throw "C-board T3 ownership boundary is missing: $document"
    }
}
Write-Output 'PASS: competition task guard documentation checks'
