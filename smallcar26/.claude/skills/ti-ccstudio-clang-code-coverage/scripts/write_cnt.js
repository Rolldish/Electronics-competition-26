// write_cnt.js
//
// Combines memory chunks read via the debug MCP readMemory tool into a
// binary .cnt file for use with tiarmprofdata / c29profdata merge.
//
// Background:
//   When LLVM coverage instrumentation is active, the compiler places two
//   sections in the binary's RAM:
//     __llvm_prf_cnts  — execution counter array (one 64-bit counter per
//                        instrumented region: function entries, branches, etc.)
//     __llvm_prf_bits  — MC/DC bitmap (only present when -fmcdc is used;
//                        one bit per condition per branch decision)
//
//   After the application has run, these raw bytes must be extracted from the
//   target and fed to the profdata tool, which correlates them with the
//   compile-time mapping metadata embedded in the .out file to produce a
//   .profdata file.  The .profdata file is then used by the coverage report
//   tool to generate an HTML report.
//
//   The .cnt file is simply the raw bytes of __llvm_prf_cnts followed
//   immediately by the raw bytes of __llvm_prf_bits (if present), with no
//   header or framing.  The profdata tool uses the .out file's symbol table
//   to know how many bytes belong to each section.
//
// Why this script exists:
//   The debug MCP readMemory tool has a 1024-byte limit per call, so large
//   counter sections must be read in multiple chunks.  Rather than embedding
//   hundreds of hex values inline in a shell command (which risks truncation
//   at the shell's command-line length limit), the agent writes each chunk to
//   a small JSON file using the Write tool and then calls this script to
//   assemble all the pieces into the final binary .cnt file.
//
// Input format:
//   Each JSON file must contain a JSON array of hex byte strings exactly as
//   returned by the readMemory tool's "values" field, e.g.:
//     ["01","00","ff","3a","00","00","00","00",...]
//   The strings are two hex digits each (no "0x" prefix).
//
// Usage:
//   node write_cnt.js --output <output.cnt> \
//       --chunk <chunk1.json> [--chunk <chunk2.json>]... \
//       [--mcdc <mcdc.json>]
//
//   --output  Required. Path for the output .cnt binary file.
//   --chunk   Path to a JSON file containing one readMemory "values" array.
//             Repeat this flag for each chunk, in address order.
//             All chunks are concatenated in the order they are listed.
//   --mcdc    Path to a JSON file containing the __llvm_prf_bits section bytes.
//             This is appended after all counter chunks.  Omit if -fmcdc was
//             not used (the symbols __start___llvm_prf_bits /
//             __stop___llvm_prf_bits will not exist in that case).

"use strict";

var fs   = require("fs");
var path = require("path");

// =============================================================================
// Argument parsing
// =============================================================================

function parseArgs() {
    // process.argv = ["node", "write_cnt.js", arg1, arg2, ...]
    // slice(2) gives us only the user-supplied arguments
    var args = process.argv.slice(2);

    var opts = {
        output: null,   // path for the output .cnt binary file
        chunks: [],     // ordered list of counter section chunk JSON files
        mcdc:   null,   // optional path to the MC/DC section JSON file
    };

    for (var i = 0; i < args.length; i++) {
        var arg = args[i];

        if (arg === "--output" && i + 1 < args.length) {
            // Destination path for the assembled .cnt binary
            opts.output = args[++i];

        } else if (arg === "--chunk" && i + 1 < args.length) {
            // Each --chunk argument names one JSON file from a single readMemory
            // call.  Multiple chunks cover a counter section larger than the
            // 1024-byte readMemory limit.  They are appended in listing order,
            // so the caller must list them in ascending address order.
            opts.chunks.push(args[++i]);

        } else if (arg === "--mcdc" && i + 1 < args.length) {
            // Optional: the __llvm_prf_bits (MC/DC bitmap) section bytes.
            // The profdata tool expects these to follow immediately after the
            // counter bytes in the .cnt file.
            opts.mcdc = args[++i];

        } else {
            console.error("Unknown argument: " + arg);
            process.exit(1);
        }
    }

    // Both --output and at least one --chunk are required
    if (!opts.output || opts.chunks.length === 0) {
        console.error(
            "Usage: node write_cnt.js --output <output.cnt> " +
            "--chunk <chunk.json> [--chunk <chunk2.json>]... [--mcdc <mcdc.json>]"
        );
        process.exit(1);
    }

    return opts;
}

// =============================================================================
// Hex JSON file reader
// =============================================================================
//
// Reads a JSON file produced by the agent (containing a readMemory "values"
// array) and converts it to a Node.js Buffer of raw bytes.
//
// The "values" array from readMemory looks like:
//   ["01","00","ff","3a","00","00","00","00"]
// where each string is exactly two hex digits representing one byte.

function readHexFile(filePath) {
    // Read the JSON text from disk
    var raw = fs.readFileSync(filePath, "utf-8");

    // Parse the JSON — we expect an array of hex-string elements
    var arr = JSON.parse(raw);
    if (!Array.isArray(arr)) {
        throw new Error(filePath + ": expected a JSON array of hex strings");
    }

    // Convert each "ff" hex string to the integer 255, then wrap in a Buffer.
    // parseInt(h, 16) converts a hex string to an integer (base 16).
    return Buffer.from(arr.map(function(h) { return parseInt(h, 16); }));
}

// =============================================================================
// Main entry point
// =============================================================================

var opts = parseArgs();

// We collect all byte buffers here in order, then concatenate them once at the
// end.  This is more efficient than repeatedly concatenating growing buffers.
var buffers = [];

// Track counter byte total separately from MC/DC bytes for the summary message
var totalCntBytes = 0;

// -----------------------------------------------------------------------
// Process counter section chunks (--chunk flags)
// Each chunk corresponds to one readMemory call that covered up to 1024
// bytes of the __llvm_prf_cnts section.  Chunks must be listed in
// ascending address order so the bytes are concatenated correctly.
// -----------------------------------------------------------------------
for (var i = 0; i < opts.chunks.length; i++) {
    var buf = readHexFile(opts.chunks[i]);
    buffers.push(buf);
    totalCntBytes += buf.length;
    // Print a summary line for each chunk so the user can verify sizes
    console.log("  chunk " + path.basename(opts.chunks[i]) + ": " + buf.length + " bytes");
}

// -----------------------------------------------------------------------
// Process MC/DC section (--mcdc flag, optional)
// The __llvm_prf_bits section contains a compact bitmap of MC/DC results.
// It is only present when -fmcdc was used at compile time.  The profdata
// tool expects these bytes to follow the counter bytes in the .cnt file.
// -----------------------------------------------------------------------
var mcdcBytes = 0;
if (opts.mcdc) {
    var mcdcBuf = readHexFile(opts.mcdc);
    buffers.push(mcdcBuf);
    mcdcBytes = mcdcBuf.length;
    console.log("  mcdc  " + path.basename(opts.mcdc) + ": " + mcdcBytes + " bytes");
}

// -----------------------------------------------------------------------
// Concatenate all buffers and write the binary .cnt file
// Buffer.concat efficiently merges all the individual chunk buffers into
// one contiguous binary blob without any intermediate copies.
// -----------------------------------------------------------------------
var output = Buffer.concat(buffers);
fs.writeFileSync(opts.output, output);

// Print a summary so the user can verify the total byte count against
// the counter section size reported by the debug session
console.log(
    "Wrote " + output.length + " bytes " +
    "(" + totalCntBytes + " counter + " + mcdcBytes + " MC/DC) to " +
    opts.output
);
