#!/usr/bin/env bash

# mri/gdb.sh
# Helper script to launch arm‑none‑eabi‑gdb against a Carvera running MRI.
#
# Behaviour:
# 1. Ensures the default ARM GCC toolchain is available via build/gcc.sh.
# 2. Determines a serial port if none supplied.
# 3. Selects a sensible default baud rate.
# 4. Execs GDB with init.gdb and sets $PORT within GDB without disturbing shell quoting.

set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( dirname "$SCRIPT_DIR" )"

######################################################################
# Select serial port (if not provided)
######################################################################

PORT="${1:-}"
if [[ -z "$PORT" ]]; then
    uname_out="$(uname -s)"
    case "${uname_out}" in
        Darwin)
            # Prefer cu.* usb serial devices, avoiding Bluetooth.
            for dev in /dev/cu.usbserial*; do
                [[ -e "$dev" ]] || continue
                [[ "$dev" == *Bluetooth* ]] && continue
                PORT="$dev"
                break
            done
            ;;
        Linux)
            for pattern in /dev/ttyACM* /dev/ttyUSB*; do
                for dev in $pattern; do
                    [[ -e "$dev" ]] || continue
                    PORT="$dev"
                    break 2
                done
            done
            ;;
    esac
fi

if [[ -z "$PORT" ]]; then
    echo "Error: Unable to determine a serial port automatically. Please provide one." >&2
    echo "Usage: $0 [serial_port] [baud]" >&2
    exit 1
fi

######################################################################
# Baud rate
######################################################################

BAUD="${2:-115200}"

######################################################################
# Verify ELF binary exists
######################################################################

ELF_FILE="$PROJECT_ROOT/LPC1768/main.elf"
if [[ ! -f "$ELF_FILE" ]]; then
    echo "Error: Firmware ELF binary not found at $ELF_FILE. Build the firmware first." >&2
    exit 1
fi

######################################################################
# Launch GDB
######################################################################

# Ensure the correct GDB (from the default toolchain) is in the PATH
gcc_env_cmd="$("$PROJECT_ROOT/build/gcc.sh" --env)" # Use default GCC
if [[ $? -ne 0 || -z "$gcc_env_cmd" ]]; then
    echo "Error: Failed to get PATH from build/gcc.sh" >&2
    exit 1
fi
eval "$gcc_env_cmd"

# Now execute gdb, it should be found in the modified PATH
exec arm-none-eabi-gdb -b "$BAUD" -x "$SCRIPT_DIR/init.gdb" -ex "target remote $PORT" "$ELF_FILE"
