$ErrorActionPreference = 'Stop'

$projectPath = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$workspaceStorage = Join-Path $env:APPDATA 'Code\User\workspaceStorage'

if (-not (Test-Path -LiteralPath $workspaceStorage)) {
    throw "VS Code workspace storage was not found: $workspaceStorage"
}

$stateDatabase = $null
Get-ChildItem -LiteralPath $workspaceStorage -Directory | ForEach-Object {
    if ($null -ne $stateDatabase) {
        return
    }

    $workspaceFile = Join-Path $_.FullName 'workspace.json'
    $candidateDatabase = Join-Path $_.FullName 'state.vscdb'
    if ((-not (Test-Path -LiteralPath $workspaceFile)) -or
        (-not (Test-Path -LiteralPath $candidateDatabase))) {
        return
    }

    try {
        $workspace = Get-Content -Raw -LiteralPath $workspaceFile |
            ConvertFrom-Json
        if (-not $workspace.folder) {
            return
        }
        $decodedFolder = [Uri]::UnescapeDataString(
            [string]$workspace.folder)
        if ($decodedFolder -match '^file:///([A-Za-z]:/.*)$') {
            $folderPath = $Matches[1]
        }
        else {
            $folderPath = ([Uri]$workspace.folder).LocalPath
        }
        if ([string]::Equals(
                (Resolve-Path -LiteralPath $folderPath).Path,
                $projectPath,
                [StringComparison]::OrdinalIgnoreCase)) {
            $stateDatabase = $candidateDatabase
        }
    }
    catch {
        return
    }
}

if ($null -eq $stateDatabase) {
    throw 'The H_BallBalance_CBoard VS Code workspace state was not found.'
}

$backup = "$stateDatabase.before-idle-center-livewatch.$(Get-Date -Format 'yyyyMMdd-HHmmss').bak"
Copy-Item -LiteralPath $stateDatabase -Destination $backup

$python = Get-Command python -ErrorAction SilentlyContinue
if ($null -eq $python) {
    throw 'Python was not found.'
}

$env:HBALL_VSCODE_STATE_DB = $stateDatabase
$pythonScript = @'
import json
import os
import sqlite3

expressions = [
    "vision_debug_task_id",
    "ball_control_debug_state",
    "ball_control_debug_fault",
    "ball_control_debug_motor_enabled",
    "ball_control_debug_target_position_0p1mm",
    "ball_control_debug_position_0p1mm",
    "ball_control_debug_velocity_mmps",
    "ball_control_debug_angle_offset_deg",
]

database = os.environ["HBALL_VSCODE_STATE_DB"]
extension_key = "marus25.cortex-debug"

with sqlite3.connect(database, timeout=10.0) as connection:
    row = connection.execute(
        "SELECT value FROM ItemTable WHERE key = ?",
        (extension_key,),
    ).fetchone()
    state = json.loads(row[0]) if row and row[0] else {}
    tree = state.get("livewatch.watchTree")
    if not isinstance(tree, dict):
        tree = {
            "name": "",
            "expr": "",
            "expanded": True,
            "children": [],
        }
    children = tree.setdefault("children", [])
    existing = {
        child.get("expr")
        for child in children
        if isinstance(child, dict)
    }
    added = []
    for expression in expressions:
        if expression in existing:
            continue
        children.append(
            {
                "name": expression,
                "expr": expression,
                "expanded": False,
                "children": [],
            }
        )
        added.append(expression)

    state["livewatch.version"] = 2
    state["livewatch.watchTree"] = tree
    encoded = json.dumps(state, ensure_ascii=False, separators=(",", ":"))
    connection.execute(
        "INSERT OR REPLACE INTO ItemTable(key, value) VALUES(?, ?)",
        (extension_key, encoded),
    )
    connection.commit()

print(f"Added {len(added)} expressions; {len(children)} total.")
'@

$pythonScript | & $python.Source -
if ($LASTEXITCODE -ne 0) {
    throw "Failed to update Live Watch. Backup: $backup"
}

Write-Host 'Cortex Live Watch expressions were updated.'
Write-Host "Backup: $backup"
Write-Host 'Reopen VS Code and start C Board: Debug with ST-Link.'
