#!/usr/bin/env bash
# Compiles MakeraFrameParser for the host and runs its tests.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
CXX="${CXX:-c++}"

OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

"$CXX" -std=c++11 -Wall -Wextra -Werror -O1 -g \
    -I"$ROOT/src" -I"$ROOT/src/libs" \
    "$ROOT/src/libs/MakeraFrameParser.cpp" \
    "$ROOT/src/libs/crc_table.cpp" \
    "$HERE/test_makera_frame_parser.cpp" \
    -o "$OUT/test_makera_frame_parser"

"$OUT/test_makera_frame_parser"
