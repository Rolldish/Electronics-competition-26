// manage_cproject_coverage.js
//
// Add or remove LLVM code coverage flags in a CCStudio .cproject file.
//
// Background:
//   CCStudio projects store build settings in a .cproject XML file.  To enable
//   LLVM code coverage instrumentation, three things must happen at compile time:
//     -fprofile-instr-generate  — injects counter increment code into every
//                                  instrumented function and basic block
//     -fcoverage-mapping        — embeds source-to-counter mapping metadata in
//                                  the binary so the coverage report tool can
//                                  map raw counts back to source lines
//   Optional flags:
//     -fmcdc                    — also tracks Modified Condition/Decision Coverage
//     -ffunction-coverage-only  — records entry counts only (no line/branch detail)
//     -fprofile-counter-size=32 — uses 32-bit instead of 64-bit counters (saves RAM)
//
//   Each flag is represented as a CCS <option> element inside the compiler <tool>
//   element in the .cproject file.  The option's superClass ID encodes both the
//   compiler version and the flag name, e.g.:
//     com.ti.ccstudio.buildDefinitions.TMS470_TICLANG_4.0.compilerID.FCOVERAGE_MAPPING
//   This script detects the correct version prefix automatically so the right IDs
//   are always used regardless of which compiler version is configured.
//
// Usage:
//   node manage_cproject_coverage.js add --cproject <path>
//       [--mcdc] [--function-only] [--counter-size 32|64]
//
//   node manage_cproject_coverage.js remove --cproject <path>
//
// The add command:
//   - Reads the .cproject and detects the compiler version prefix automatically
//   - Adds mandatory coverage flags to ALL compiler tool elements found in ALL
//     build configurations (Debug, Release, RAM, FLASH, etc.) so that any
//     configuration the user builds will have coverage instrumentation
//   - Optionally adds MC/DC, function-only, and counter-size flags
//   - Assigns unique numeric option IDs that do not collide with existing IDs
//   - Idempotent: safe to run multiple times; tools already containing
//     FCOVERAGE_MAPPING are skipped without modification
//
// The remove command:
//   - Removes all coverage-related <option> elements from ALL build configurations
//   - Idempotent: safe to run when coverage flags are already absent

"use strict";

var fs   = require("fs");
var path = require("path");

// =============================================================================
// Argument parsing
// =============================================================================

function parseArgs() {
    // process.argv = ["node", "script.js", arg1, arg2, ...]
    // slice(2) gives us the user-supplied arguments only
    var args = process.argv.slice(2);

    var opts = {
        command:      null,    // "add" | "remove"
        cproject:     null,    // path to the .cproject file
        mcdc:         false,   // --mcdc: add -fmcdc flag
        functionOnly: false,   // --function-only: add -ffunction-coverage-only
        counterSize:  null,    // --counter-size 32|64: add -fprofile-counter-size=N
    };

    if (args.length === 0) {
        printUsageAndExit();
    }

    // First positional argument is the sub-command
    opts.command = args[0];
    if (opts.command !== "add" && opts.command !== "remove") {
        console.error("Error: command must be 'add' or 'remove', got: " + opts.command);
        printUsageAndExit();
    }

    // Parse remaining named arguments
    for (var i = 1; i < args.length; i++) {
        switch (args[i]) {
            case "--cproject":
                // Path to the .cproject file to modify
                opts.cproject = args[++i];
                break;
            case "--mcdc":
                // Enable Modified Condition/Decision Coverage instrumentation.
                // This adds the -fmcdc compiler flag which tracks whether each
                // boolean sub-expression independently affects the branch outcome.
                // Increases binary size and RAM usage.
                opts.mcdc = true;
                break;
            case "--function-only":
                // Only record function entry counts, not line/branch coverage.
                // Significantly reduces instrumentation overhead when full source
                // coverage detail is not needed.
                opts.functionOnly = true;
                break;
            case "--counter-size":
                // Use 32-bit counters instead of the default 64-bit.
                // Halves the RAM required for the counter section, which matters
                // on resource-constrained embedded targets.
                opts.counterSize = args[++i];
                if (opts.counterSize !== "32" && opts.counterSize !== "64") {
                    console.error("Error: --counter-size must be 32 or 64");
                    process.exit(1);
                }
                break;
            default:
                console.error("Unknown argument: " + args[i]);
                printUsageAndExit();
        }
    }

    if (!opts.cproject) {
        console.error("Error: --cproject is required");
        printUsageAndExit();
    }

    // Verify the file actually exists before proceeding
    if (!fs.existsSync(opts.cproject)) {
        console.error("Error: .cproject file not found: " + opts.cproject);
        process.exit(1);
    }

    return opts;
}

function printUsageAndExit() {
    console.error(
        "Usage:\n" +
        "  node manage_cproject_coverage.js add --cproject <path>\n" +
        "      [--mcdc] [--function-only] [--counter-size 32|64]\n" +
        "\n" +
        "  node manage_cproject_coverage.js remove --cproject <path>"
    );
    process.exit(1);
}

// =============================================================================
// Compiler version prefix extraction
// =============================================================================
//
// Each compiler tool element in the .cproject has a superClass attribute like:
//   com.ti.ccstudio.buildDefinitions.TMS470_TICLANG_4.0.exe.compilerDebug
//   com.ti.ccstudio.buildDefinitions.C2000_TICLANG_2.0.exe.compilerDebug
//
// The version prefix (e.g. "TMS470_TICLANG_4.0" or "C2000_TICLANG_2.0") must
// be embedded in every coverage option's id and superClass attributes so that
// CCS knows which compiler definition the option belongs to.  Without the
// correct prefix, CCS silently ignores the option.
//
// Supported prefixes:
//   TMS470_TICLANG_<major>.<minor>  — TI Arm Clang compiler (Cortex-M/R devices)
//   C2000_TICLANG_<major>.<minor>   — TI C29 Clang compiler (C2000 devices)

// Regex breakdown:
//   buildDefinitions\.   — literal anchor before the prefix
//   (                    — start capture group for the prefix
//     (?:TMS470|C2000)   — non-capturing: match either Arm or C29 family name
//     _TICLANG_          — literal separator
//     [\d]+\.[\d]+       — version number, e.g. "4.0" or "2.1"
//   )                    — end capture group
//   \.exe\.              — literal suffix after the version, before the tool type
var PREFIX_RE = /buildDefinitions\.((?:TMS470|C2000)_TICLANG_[\d]+\.[\d]+)\.exe\./;

function extractPrefix(superClass) {
    var m = PREFIX_RE.exec(superClass);
    // Return the captured prefix group, or null if the superClass doesn't match
    // (e.g. it's a linker, hex utility, or SysConfig tool — those are skipped)
    return m ? m[1] : null;
}

// =============================================================================
// Option ID management
// =============================================================================
//
// Every <option> element in a .cproject must have a globally unique numeric
// suffix in its id attribute, e.g.:
//   id="com.ti.ccstudio.buildDefinitions.TMS470_TICLANG_4.0.compilerID.WALL.683568894"
//                                                                              ^^^^^^^^^
//                                                                          unique suffix
//
// To avoid collisions with any existing options, we scan the entire .cproject
// for the largest numeric suffix currently in use and start new IDs one above it.
// We also leave a gap of 10 between configurations so that adding multiple
// configs in the same file never produces duplicate IDs.

function findMaxOptionId(content) {
    // Start at 100,000,000 as a floor — well above zero but leaves room below
    // the typical CCS-generated IDs which are random 9-digit numbers
    var max = 100000000;

    // Match any id="..." attribute whose last dot-separated segment is 6+ digits.
    // This catches the standard CCS option ID format reliably.
    var re = /id="[^"]*\.(\d{6,})"/g;
    var m;
    while ((m = re.exec(content)) !== null) {
        var n = parseInt(m[1], 10);
        if (n > max) {
            max = n;
        }
    }
    return max;
}

// =============================================================================
// XML option element generation
// =============================================================================
//
// Generates the XML <option> elements to insert into the compiler tool.
// Each option uses the full CCS build definition namespace so CCS can match
// the option to the correct compiler version and flag.
//
// The indentation constant matches the standard depth of option elements
// inside the compilerDebug tool in a typical CCStudio .cproject file.

// 32 spaces = 8 levels × 4 spaces each — matches standard .cproject indentation
var INDENT = "                                ";

function buildOptionsXml(prefix, opts, baseId) {
    // Build the namespace prefix shared by all options for this compiler version
    // e.g. "com.ti.ccstudio.buildDefinitions.TMS470_TICLANG_4.0.compilerID."
    var ns = "com.ti.ccstudio.buildDefinitions." + prefix + ".compilerID.";

    var lines = [];

    // -------------------------------------------------------------------------
    // Mandatory flag 1: -fcoverage-mapping
    // Embeds source-to-counter mapping metadata in the binary.  This is what
    // allows the coverage report tool (tiarmcov / c29cov) to map raw execution
    // counts back to specific source lines and branches.
    // -------------------------------------------------------------------------
    lines.push(
        INDENT + "<option id=\"" + ns + "FCOVERAGE_MAPPING." + (baseId) + "\"\n" +
        INDENT + "        superClass=\"" + ns + "FCOVERAGE_MAPPING\"\n" +
        INDENT + "        value=\"true\" valueType=\"boolean\"/>"
    );

    // -------------------------------------------------------------------------
    // Mandatory flag 2: -fprofile-instr-generate
    // Injects counter increment instructions at every instrumented point
    // (function entries, branches, etc.).  The counters are stored in the
    // __llvm_prf_cnts section in RAM and read out after execution.
    // -------------------------------------------------------------------------
    lines.push(
        INDENT + "<option id=\"" + ns + "FPROFILE_INSTR_GENERATE." + (baseId + 1) + "\"\n" +
        INDENT + "        superClass=\"" + ns + "FPROFILE_INSTR_GENERATE\"\n" +
        INDENT + "        value=\"true\" valueType=\"boolean\"/>"
    );

    // -------------------------------------------------------------------------
    // Optional flag: -fmcdc
    // Modified Condition/Decision Coverage tracks whether each boolean
    // sub-expression (condition) independently influences the decision outcome.
    // Required for DO-178C Level A/B certification.  Increases binary size and
    // runtime overhead.  Adds MC/DC bitmap data to the __llvm_prf_bits section.
    // -------------------------------------------------------------------------
    if (opts.mcdc) {
        lines.push(
            INDENT + "<option id=\"" + ns + "FMCDC." + (baseId + 2) + "\"\n" +
            INDENT + "        superClass=\"" + ns + "FMCDC\"\n" +
            INDENT + "        value=\"true\" valueType=\"boolean\"/>"
        );
    }

    // -------------------------------------------------------------------------
    // Optional flag: -ffunction-coverage-only
    // Restricts coverage instrumentation to function entry points only.
    // No line, branch, or region counters are added.  Much lower overhead
    // than full source coverage; useful for a quick first pass to identify
    // completely untested functions.
    // -------------------------------------------------------------------------
    if (opts.functionOnly) {
        lines.push(
            INDENT + "<option id=\"" + ns + "FFUNCTION_COVERAGE_ONLY." + (baseId + 3) + "\"\n" +
            INDENT + "        superClass=\"" + ns + "FFUNCTION_COVERAGE_ONLY\"\n" +
            INDENT + "        value=\"true\" valueType=\"boolean\"/>"
        );
    }

    // -------------------------------------------------------------------------
    // Optional flag: -fprofile-counter-size=N
    // Controls the size of each counter (32 or 64 bits).  The default is 64-bit,
    // which can overflow-saturate only after 2^64 hits — essentially never.
    // Using 32-bit halves the __llvm_prf_cnts section size at the cost of
    // potential overflow after ~4 billion hits per counter.
    //
    // IMPORTANT: The value attribute MUST include the leading '=' character.
    // CCStudio concatenates the flag name with the value string directly, so:
    //   value="32"  → produces the INVALID flag:  -fprofile-counter-size32
    //   value="=32" → produces the CORRECT flag:  -fprofile-counter-size=32
    // -------------------------------------------------------------------------
    if (opts.counterSize) {
        lines.push(
            INDENT + "<option id=\"" + ns + "FPROFILE_COUNTER_SIZE." + (baseId + 4) + "\"\n" +
            INDENT + "        superClass=\"" + ns + "FPROFILE_COUNTER_SIZE\"\n" +
            INDENT + "        value=\"=" + opts.counterSize + "\" valueType=\"string\"/>"
        );
    }

    // Join all option elements with newlines and add a trailing newline so the
    // closing </tool> tag ends up on its own line after the last option
    return lines.join("\n") + "\n";
}

// =============================================================================
// Add coverage flags to all compiler tool elements
// =============================================================================
//
// Strategy:
//   1. Find every compiler tool element (compilerDebug or compilerRelease)
//      in the .cproject using a regex that matches on the superClass attribute.
//   2. For each tool, extract the version prefix and locate the closing </tool>.
//   3. Check whether FCOVERAGE_MAPPING is already present (idempotency guard).
//   4. Collect all insertion points first, then apply them in REVERSE order
//      so that inserting text at a later position does not shift the character
//      offsets of earlier positions.

function addFlags(content, opts) {
    // Regex to find compiler tool open tags.
    // We look for superClass attributes containing "compilerDebug" or
    // "compilerRelease" to catch all build configurations.
    // We deliberately exclude linker, objcopy, hex, and SysConfig tools.
    var toolOpenRe = /<tool\s[^>]*superClass="([^"]*(?:compilerDebug|compilerRelease)[^"]*)"[^>]*>/g;

    // Each entry records where to insert and which prefix to use
    var insertions = [];
    var m;

    while ((m = toolOpenRe.exec(content)) !== null) {
        var superClass = m[1];

        // Extract the version prefix from this tool's superClass.
        // Returns null for non-compiler tools (linker, SysConfig, etc.) which
        // we skip.
        var prefix = extractPrefix(superClass);
        if (!prefix) {
            continue;
        }

        // The content between the opening <tool...> tag and its </tool> is
        // the tool body — this is where we look for existing options.
        // searchFrom is the character position right after the opening tag.
        var searchFrom = m.index + m[0].length;

        // Find the closing </tool> tag.  Since tool elements do not nest in
        // .cproject files, the first </tool> after the opening tag is always
        // the correct matching close.
        var closePos = content.indexOf("</tool>", searchFrom);
        if (closePos === -1) {
            // Malformed XML — skip this tool to be safe
            console.log("  warning: no closing </tool> found for tool at index " + m.index);
            continue;
        }

        // Idempotency check: if FCOVERAGE_MAPPING is already present in this
        // tool body, the flags were already added — skip to avoid duplicates.
        var toolBody = content.substring(searchFrom, closePos);
        if (toolBody.indexOf("FCOVERAGE_MAPPING") !== -1) {
            console.log("  skip (already flagged): " + prefix);
            continue;
        }

        // Record this as a valid insertion point
        insertions.push({ closePos: closePos, prefix: prefix });
    }

    // Nothing to do — return the original content unchanged
    if (insertions.length === 0) {
        return content;
    }

    // Find the highest existing option ID in the entire file so new IDs are
    // guaranteed not to collide with any existing ones.
    var maxId = findMaxOptionId(content);
    var nextId = maxId + 1;

    // Sort insertions from LAST to FIRST by character position.
    // This is critical: when we insert text, all character positions after the
    // insertion point shift by the length of the inserted text.  By working
    // backwards, we ensure that positions recorded for earlier tools remain
    // valid when we reach them.
    insertions.sort(function(a, b) { return b.closePos - a.closePos; });

    var result = content;
    for (var i = 0; i < insertions.length; i++) {
        var ins = insertions[i];

        // Build the XML option elements for this tool
        var xml = buildOptionsXml(ins.prefix, opts, nextId);

        // Insert the options immediately before the </tool> closing tag
        result =
            result.substring(0, ins.closePos) +
            xml +
            result.substring(ins.closePos);

        // Advance the ID counter by 10 to leave a gap between each config's
        // options — this prevents ID collisions when multiple configs are
        // present (e.g. both Debug and Release in the same file)
        nextId += 10;

        console.log("  added flags for prefix: " + ins.prefix);
    }

    return result;
}

// =============================================================================
// Remove coverage flags from all compiler tool elements
// =============================================================================
//
// Removes every <option> element whose superClass attribute contains one of
// the five coverage-related flag identifiers.  The regex handles multi-line
// option elements (which this script generates, with attributes on separate
// lines) as well as single-line elements.
//
// Regex breakdown:
//   \n[ \t]*            — consume the newline + indentation that precedes the
//                          option element, so no blank lines are left behind
//   <option             — opening tag
//   (?:[^>]|\n)*?       — any content up to the superClass attr (non-greedy,
//                          handles multi-line attribute layout)
//   superClass="[^"]*   — start of superClass value
//   (?:FCOVERAGE_MAPPING|FPROFILE_INSTR_GENERATE|FMCDC|
//      FFUNCTION_COVERAGE_ONLY|FPROFILE_COUNTER_SIZE)
//                       — one of the five coverage flag identifiers
//   [^"]*"              — remainder of the superClass value
//   (?:[^>]|\n)*?       — remaining attributes (value, valueType)
//   \/>                 — self-closing tag end (coverage options are never
//                          container elements, so /> is always correct)

var REMOVE_RE = /\n[ \t]*<option(?:[^>]|\n)*?superClass="[^"]*(?:FCOVERAGE_MAPPING|FPROFILE_INSTR_GENERATE|FMCDC|FFUNCTION_COVERAGE_ONLY|FPROFILE_COUNTER_SIZE)[^"]*"(?:[^>]|\n)*?\/>/g;

function removeFlags(content) {
    var count = 0;

    // Replace every matching <option> element with an empty string.
    // The replacement function is used so we can count how many were removed.
    var result = content.replace(REMOVE_RE, function() {
        count++;
        return "";
    });

    return { content: result, count: count };
}

// =============================================================================
// Main entry point
// =============================================================================

// Parse command-line arguments
var opts = parseArgs();

// Read the entire .cproject file as a UTF-8 string.
// We work with the raw text rather than parsing it as XML to avoid needing
// any external XML library — the file format is regular enough for regex-based
// manipulation, and working with text preserves all existing formatting and
// comments that an XML round-trip might discard.
var content = fs.readFileSync(opts.cproject, "utf-8");
var original = content; // Keep a copy to detect whether any changes were made

if (opts.command === "add") {
    content = addFlags(content, opts);

    if (content === original) {
        // addFlags returns the original string unchanged when all tools were
        // already flagged (idempotency path)
        console.log("No changes made — coverage flags already present in all compiler tools.");
    } else {
        // Write the modified .cproject back to disk
        fs.writeFileSync(opts.cproject, content, "utf-8");
        console.log("Coverage flags added to " + path.basename(opts.cproject));
    }

} else { // command === "remove"
    var result = removeFlags(content);

    if (result.count === 0) {
        // No coverage options were found — nothing to remove
        console.log("No changes made — no coverage flags found.");
    } else {
        fs.writeFileSync(opts.cproject, result.content, "utf-8");
        console.log(
            "Removed " + result.count + " coverage option(s) from " +
            path.basename(opts.cproject)
        );
    }
}
