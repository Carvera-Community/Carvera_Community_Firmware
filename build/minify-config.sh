#!/bin/sh
# minify-config.sh - Strip comments, blank lines, and excess whitespace from
# config files before embedding them into the firmware binary.
#
# Usage: minify-config.sh <input> <output>
#
# This reduces flash usage by removing data that the config parser skips anyway:
#   - Lines starting with # (comments)
#   - Blank or whitespace-only lines
#   - Trailing comments (everything after the value's first # on a line)
#   - Runs of tabs/spaces used for column alignment (collapsed to single tab)
#   - Trailing whitespace on each line

set -e

if [ $# -ne 2 ]; then
    echo "Usage: $0 <input> <output>" >&2
    exit 1
fi

INPUT="$1"
OUTPUT="$2"

# ============================================================================
# IMPORTANT: Each regex below is carefully chosen to ONLY remove bytes that
# the config parser (FirmConfigSource) already ignores at runtime. None of
# these will ever match or alter an actual config key or value.
#
# If you modify these rules, verify with:
#   diff <(grep -v '^\s*#' config.default | grep -v '^\s*$' | awk '{print $1,$2}' | sort) \
#        <(grep -v '^\s*#' config.default.min | grep -v '^\s*$' | awk '{print $1,$2}' | sort)
# The output should be empty (no differences in key-value pairs).
#
# Rule-by-rule breakdown:
#
#   1) /^[[:space:]]*#/d
#      DELETE entire lines that are pure comments (optional leading whitespace
#      followed by #). These are documentation lines the parser skips.
#        "# This is a comment"             → deleted
#        "   # indented comment"           → deleted
#        "switch.vacuum.enable  true"      → KEPT (no leading #)
#
#   2) /^[[:space:]]*$/d
#      DELETE blank lines or lines containing only whitespace. The parser
#      skips these too.
#        ""                                → deleted
#        "   "                             → deleted
#        "alpha_step_pin  1.28"            → KEPT (has content)
#
#   3) s/[[:space:]]#[[:space:]].*$//
#      STRIP trailing comments from value lines. Matches a whitespace char,
#      then #, then another whitespace char, then everything to end-of-line.
#      The double-whitespace requirement (space-#-space) prevents false
#      matches on values that might contain # adjacent to text. In practice,
#      no config values in this project contain " # " as part of their value.
#        "junction_deviation  0.01  # comment"  → "junction_deviation  0.01"
#        "alpha_dir_pin  1.29!"                 → KEPT (no trailing comment)
#        "some_key  value#notacomment"          → KEPT (# has no surrounding spaces)
#
#   4) s/[[:space:]]*$//
#      STRIP trailing whitespace from each line. Just removes spaces/tabs at
#      the end that carry no meaning.
#        "alpha_en_pin  nc      "          → "alpha_en_pin  nc"
#
#   5) s/[[:space:]]\{2,\}/\t/g
#      COLLAPSE runs of 2+ whitespace characters (spaces or tabs used for
#      column alignment) into a single tab. The parser splits on any
#      whitespace, so "key\t\t\tvalue" and "key\tvalue" are equivalent.
#        "alpha_steps_per_mm          200" → "alpha_steps_per_mm\t200"
#        "x  y"                            → "x\ty"
#        "a b"                             → KEPT (single space, not 2+)
#
# ============================================================================
sed \
    -e '/^[[:space:]]*#/d' \
    -e '/^[[:space:]]*$/d' \
    -e 's/[[:space:]]#[[:space:]].*$//' \
    -e 's/[[:space:]]*$//' \
    -e 's/[[:space:]]\{2,\}/\t/g' \
    "$INPUT" > "$OUTPUT"
