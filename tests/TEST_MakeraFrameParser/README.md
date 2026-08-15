# TEST_MakeraFrameParser

Host-side tests for `src/libs/MakeraFrameParser.cpp`, the receive-side framing
shared by the UART console (`SerialConsole`) and the wifi provider
(`WifiProvider`).

Unlike the other directories under `tests/`, this is not a gcode fixture: the
parser is plain C++ with no `Kernel` or `mbed` dependency, so it can be
compiled and driven natively. That is the reason for the split between
`MakeraFrameParser` (pure) and `MakeraControl` (which touches `THEKERNEL` and
is therefore not covered here).

## Running

```shell
./tests/TEST_MakeraFrameParser/run.sh
```

Any C++11 host compiler will do; override with `CXX=g++-14 ./run.sh` if the
default `c++` is not what you want. The script builds with `-Wall -Wextra
-Werror`, which is stricter than the firmware build, so it doubles as a
warning check on the parser. It exits non-zero if any check fails.

This is not wired into CI. Run it by hand when changing the parser.

## What is covered

Frame framing and resynchronisation (header hunting across noise, a split
header pair, length validation, CRC and footer rejection), the command queue
(ordering, capacity, overflow reporting, drain and refill, the payload size
boundary), and the frame-type dispatch that decides between a control byte, a
queued command, a file-start and an unhandled type.

`crc16_ccitt` is checked against the standard CRC-16/XMODEM vector
(`"123456789"` gives `0x31C3`), which is what this protocol uses.

## A note on writing tests here

Several rejection paths are reachable for more than one reason -- a frame with
a corrupt length will also fail its CRC a few bytes later -- so a test that
only asserts `FrameError` can pass even when the check it is aiming at has
been removed. Where that applies, the tests feed bytes one at a time and
assert *when* the rejection happens. Worth preserving if you extend them.
