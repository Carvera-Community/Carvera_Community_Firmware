# TEST_MakeraFrameParser

Host-side tests for `src/libs/MakeraFrame.cpp`, the generic receive-side frame
decoder shared by the UART console (`SerialConsole`) and the Wi-Fi provider
(`WifiProvider`).

Unlike the other directories under `tests/`, this is not a gcode fixture: the
decoder is plain C++ with no `Kernel` or `mbed` dependency, so it can be
compiled and driven natively.

## Running

```shell
./tests/TEST_MakeraFrameParser/run.sh
```

Any C++17 host compiler will do; override with `CXX=g++-14 ./run.sh` if the
default `c++` is not what you want. The script builds with `-Wall -Wextra
-Werror`, which is stricter than the firmware build, so it doubles as a
warning check on the decoder. It exits non-zero if any check fails.

This is not wired into CI. Run it by hand when changing the parser.

## What is covered

Frame framing and resynchronisation (header hunting across noise, a split
header pair, length validation, CRC and footer rejection), the command queue
(ordering, capacity, overflow reporting, drain and refill, the payload size
boundary), and the shared interpretation of Makera control bytes.

## A note on writing tests here

Several rejection paths are reachable for more than one reason -- a frame with
a corrupt length will also fail its CRC a few bytes later -- so a test that
only asserts an error result can pass even when the check it is aiming at has
been removed. Where that applies, the tests feed bytes one at a time and
assert *when* the rejection happens. Worth preserving if you extend them.
