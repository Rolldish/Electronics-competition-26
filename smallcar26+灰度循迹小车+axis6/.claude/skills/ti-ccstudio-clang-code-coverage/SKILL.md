---
name: ti-clang-code-coverage
description: Generate a code coverage or profiling report for a TI Clang project. Use when the user asks for code coverage, coverage report, profiling instrumentation, or wants to measure test coverage on an embedded target. Supports both TI Arm Clang (TMS470) and TI C29 Clang (C2000) compilers.
allowed-tools: Bash, Read, Glob, Grep
---

# Code Coverage for TI Clang Projects (Arm and C29)

Generate a code coverage or profiling report for a project compiled with the TI Arm Clang compiler
or the TI C29 Clang compiler. Both compilers use LLVM-based coverage instrumentation with
identical flag names and the same LLVM profiling runtime, so the high-level flow is the same.
The differences are in the `.cproject` superClass prefix and the coverage tool binary names.

## High-Level Flow

1. Add code coverage compiler flags to the project `.cproject` file
2. Build the project
3. Debug/run the application on target for a user-determined amount of time
4. Extract profiling counters from target memory (produces a `.cnt` file)
5. Generate the .profdata file
6. Generate the coverage report
7. Ask user if coverage flags should be removed

## Step 1 - Add Compiler Flags to .cproject

### Optional Flags

Ask the user if they want any optional flags beyond the mandatory pair:

| Flag | CLI Equivalent | Purpose |
|------|---------------|---------|
| `--mcdc` | `-fmcdc` | Enable MC/DC (Modified Condition/Decision Coverage). Increases instrumentation cost. |
| `--function-only` | `-ffunction-coverage-only` | Function-level coverage only. Decreases instrumentation cost. |
| `--counter-size 32` | `-fprofile-counter-size=32` | Use 32-bit instead of default 64-bit counters. Saves memory on target. |

### Adding Flags with the Script

Use `manage_cproject_coverage.js` to add the coverage flags. The script automatically detects
the compiler version prefix from the `.cproject`, adds mandatory flags
(`-fcoverage-mapping`, `-fprofile-instr-generate`) to **all** build configurations (Debug,
Release, RAM, FLASH, etc.), and is idempotent.

**macOS / Linux:**
```bash
node ./scripts/manage_cproject_coverage.js add --cproject <project_location>/.cproject \
    [--mcdc] [--function-only] [--counter-size 32|64]
```

**Windows:**
```powershell
node .\scripts\manage_cproject_coverage.js add --cproject <project_location>\.cproject `
    [--mcdc] [--function-only] [--counter-size 32|64]
```

**Reference**: Chapter 12 of the TI Arm Clang Compiler Tools User's Guide (and the equivalent
section of the TI C29 Clang Compiler Tools User's Guide) describes the full code coverage tools
and collection process.

### Resolving Tool Paths

Several steps require paths that can be derived from the project and CCStudio IDE installation:

- **CCStudio IDE installation path** (`<ccspath>`): Use `{ccs-install-dir}`.
- **Compiler tools path** (`<compiler tools path>`): Read the project's `.cproject` file and find
  the `OPT_CODEGEN_VERSION` option (e.g., `value="TICLANG_4.0.4.LTS"` for Arm or
  `value="TICLANG_2.0.0.STS"` for C29). Match this version string against the compilers returned
  by `getCompilers` to get the install location.
  - For TI Arm Clang projects the compiler family is `TMS470`, tools are named `tiarmprofdata` /
    `tiarmcov`.
  - For TI C29 Clang projects the compiler family is `C2000` with a `TICLANG_` version, tools are
    named `c29profdata` / `c29cov`. The install directory name starts with `ti-cgt-c29_`.
  - Note: `device.toolVersions` from the project descriptor lists all *compatible* compilers, not
    the one configured for the project — do not use it for path resolution.
- **CCXML path**: Use the project descriptor's `activeTargetConfiguration` field, resolved
  relative to the project `location` (e.g.,
  `<project location>/targetConfigs/MSPM0G3507.ccxml`).
- **Executable path**: `<project location>/<activeBuildConfiguration>/<project name>.out`
  (e.g., `Debug/adc_to_uart_LP_MSPM0G3507_nortos_ticlang.out`).
- **Core pattern** for the `--core` option in Step 4: Derive from the project descriptor's
  `device.compatibleCores[0].id`. Map the core ID to a short pattern:

  | Core ID | Pattern |
  |---|---|
  | `CORTEX_M0P` | `M0` |
  | `CORTEX_M4` | `M4` |
  | `CORTEX_M33` | `M33` |
  | `CORTEX_R5` | `R5` |
  | `C29xx_CPU1` | `CPU1` |
  | `C29xx_CPU2` | `CPU2` |
  | `C29xx_CPU3` | `CPU3` |

  If the mapping is unclear, fall back to the first word after the underscore in the core ID.

## Step 2 - Build the Project

After editing `.cproject`, CCS IDE holds the project configuration in memory and may not detect
the file change before the build starts. To ensure CCS reloads the updated `.cproject`, update
the modification timestamp of any source file in the project before building.

Use the platform-appropriate command via the Bash tool:

**macOS / Linux:**
```bash
touch <project_location>/<any_source_file>.c
```

**Windows:**
```powershell
(Get-Item "<project_location>/<any_source_file>.c").LastWriteTime = Get-Date
```

Then use `buildProject` to compile the project with the coverage flags enabled. Verify the build
succeeds and that the coverage flags appear in the compiler command output before proceeding.

## Step 3 - Run the Application

The application must be loaded and run on the target for a sufficient duration to exercise the
code paths the user wants to measure. The user determines when enough execution has occurred.

- Use `debugProject` to start a debug session
- Use `continue` to run the application
- Wait for the user to indicate the application has run sufficiently
- Use `pause` to halt execution before extracting data

## Step 4 - Extract Coverage Data from Target Memory

### Prerequisites
- Project was built with `-fprofile-instr-generate` and `-fcoverage-mapping`
- Debug session is active and execution is **paused**

**CRITICAL — Do NOT terminate the debug session before reading counters.** Calling `terminate`
resumes the CPU from its current PC before closing the session. On devices with lengthy GEL
`OnTargetConnect` sequences (e.g. F29H85x, some Arm devices), the application will continue
running for 10–30 seconds while the scripting tool reconnects, accumulating spurious execution
counts. Reading via the active debug session avoids this entirely.

### Reading counter memory from the active debug session

Use the debug MCP tools while the session is still active and paused:

1. **Resolve counter section bounds** using `evaluate`:
   - `__start___llvm_prf_cnts` → start address of the counter section
   - `__stop___llvm_prf_cnts`  → end address (exclusive)
   - Byte count = stop − start

2. **Read counter section** using `readMemory` with `type="8"`.

   **IMPORTANT — 1024-byte limit**: The `readMemory` tool accepts at most 1024 bytes per
   call. If the counter section is larger than 1024 bytes, split it into multiple calls,
   each reading up to 1024 bytes of contiguous memory. Read all chunks in parallel.

3. **Resolve MC/DC section bounds** (may not be present — handle gracefully):
   - `__start___llvm_prf_bits` → start address
   - `__stop___llvm_prf_bits`  → end address
   - If the symbols do not exist, treat the MC/DC section as empty.

4. **Read MC/DC section** (if present) using `readMemory` with `type="8"`.

5. **Save each chunk to a temp JSON file** using the Write tool. Write the `values` array
   from each `readMemory` result directly as a JSON file, e.g.:

   ```json
   ["01","00","ff","3a",...]
   ```

   Use a distinct temp file name for each chunk (e.g. `/tmp/cnt_chunk1.json`,
   `/tmp/cnt_chunk2.json`, `/tmp/mcdc.json`). Using the Write tool avoids shell
   command-line length limits that occur when embedding large arrays inline.

6. **Combine chunks into the `.cnt` file** using `write_cnt.js` via the Bash tool:

   **macOS / Linux:**
   ```bash
   node ./scripts/write_cnt.js \
       --output <output_path>.cnt \
       --chunk /tmp/cnt_chunk1.json \
       --chunk /tmp/cnt_chunk2.json \
       --mcdc  /tmp/mcdc.json
   ```

   **Windows:**
   ```powershell
   node .\scripts\write_cnt.js `
       --output <output_path>.cnt `
       --chunk $env:TEMP\cnt_chunk1.json `
       --chunk $env:TEMP\cnt_chunk2.json `
       --mcdc  $env:TEMP\mcdc.json
   ```

   Omit `--mcdc` if the MC/DC section is not present. Add as many `--chunk` flags as
   needed — they are concatenated in the order given.

7. **Terminate the debug session** using `terminate` only after the `.cnt` file has been
   successfully written.

The LLVM profiling symbol names (`__start___llvm_prf_cnts`, `__stop___llvm_prf_cnts`,
`__start___llvm_prf_bits`, `__stop___llvm_prf_bits`) are identical for both Arm and C29
compiler families.

## Step 5 - Generate the .profdata

The tool name depends on compiler family:

- **TI Arm Clang:** `<compiler tools path>/bin/tiarmprofdata`
- **TI C29 Clang:** `<compiler tools path>/bin/c29profdata`

Execute:
```
<profdata tool> merge -sparse -obj-file=<.out file> <.cnt file> -o <.profdata file>
```

Multiple coverage runs can be combined with the profdata tool and a single report generated
from the merged data.

## Step 6 - Generate the Report

The tool name depends on compiler family:

- **TI Arm Clang:** `<compiler tools path>/bin/tiarmcov`
- **TI C29 Clang:** `<compiler tools path>/bin/c29cov`

Execute:
```
<cov tool> show --format=html --show-instantiations --show-branches=count \
    --object=<.out file> -instr-profile=<.profdata file> \
    --output-dir=<cov report directory>
```

The coverage report directory must be placed **inside the project's active build configuration
folder**: `<project location>/<activeBuildConfiguration>/coverage_report/`

For example, for a project `myproject` with active build configuration `Debug`:
- Report directory: `<project location>/Debug/coverage_report/`

If the MCDC option was selected, add `--show-mcdc-summary` to the command.

Present the final report path to the user.

## Step 6.5 - Patch the HTML Report

Run the post-processing script to apply TI branding, fix CCStudio IDE webview usability issues,
and add project metadata.

### Resolving the shared stylesheet path

The script accepts a `--ti-css` argument pointing to the shared `ti-brand.css` file in the CCS
installation. The CSS file is at `{ccs-ai-resources-dir}/styles/ti-brand.css` — i.e., in a
`styles/` subdirectory alongside `CCS.md`. If the file cannot be read, the script falls back
to built-in styles automatically.

For Linux or macOS: execute `<ccspath>/ccs/scripting/run.sh` with arguments:
For Windows: execute `<ccspath>/ccs/scripting/run.bat` with arguments:

```
./scripts/patch_coverage_report.js <cov report directory>
    --project <project name>
    --compiler <compiler display string>
    --coverage-type <coverage type>
    --toolchain <toolchain name>
    --ti-css <path to ti-brand.css>
```

- `--project`: project name (optional but recommended)
- `--compiler`: compiler display string from `getCompilers` (e.g., `"TI Clang v4.0.4.LTS"` for
  Arm, `"TI Clang v2.2.0.LTS"` for C29)
- `--coverage-type`: describe the collection mode based on flags enabled. Examples:
  `"Source Coverage"` (default), `"Function Coverage"` (with `-ffunction-coverage-only`),
  `"Source Coverage + MC/DC"` (with `-fmcdc`)
- `--toolchain`: toolchain brand string used in the report footer. Use `"TI Arm Clang"` for Arm
  Clang projects and `"TI C29 Clang"` for C29 projects. Defaults to `"TI Arm Clang"` if omitted.
- `--ti-css`: full path to `ti-brand.css` from the CCS installation `ai/styles/` directory.
  Provides the shared TI brand stylesheet; falls back to built-in styles if omitted or unreadable.

The script applies these changes:
1. Renames the report title from "Coverage Report" to "Code Coverage Report"
2. Adds a TI-branded red header bar with project name, compiler version, and generation date
3. Applies TI brand colors (TI red #CC0000) and improved table/typography styling
4. Removes the "jump to first uncovered line" link — that link changes the URL hash
   fragment, which causes the Theia IDE outer panel to scroll (hiding editor tabs).
5. Adds a "← Summary" back-navigation link in the sticky file-name header — the Theia
   webview has no browser back button, so without this there is no way to return to the
   summary index page.

The script is idempotent — safe to re-run on an already-patched report.

## Step 7 - Ask user if the coverage compiler options should be removed

If the user wants to remove the coverage flags, use `manage_cproject_coverage.js remove`:

**macOS / Linux:**
```bash
node ./scripts/manage_cproject_coverage.js remove --cproject <project_location>/.cproject
```

**Windows:**
```powershell
node .\scripts\manage_cproject_coverage.js remove --cproject <project_location>\.cproject
```

The script removes all coverage-related options from all build configurations and is idempotent.
