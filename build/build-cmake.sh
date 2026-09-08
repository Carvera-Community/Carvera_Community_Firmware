#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." >/dev/null 2>&1 && pwd)"
ORIGINAL_PWD="$PWD"

DEFAULT_GCC_VERSION="14.2"

print_help() {
    cat <<EOF
Usage: $0 [options] [NAME=value | -DNAME=value ...]

Configures and builds the LPC1768 firmware with native CMake support.
The default is a Release build with AXIS=5 and PAXIS=3.

Options:
  --gcc <version>  Select the GCC version managed by build/gcc.sh (default: ${DEFAULT_GCC_VERSION}).
  --clean          Clean the selected CMake build tree before building.
  --output <path>  Copy firmware.bin to an existing directory or file path.
  --debug          Build the Debug profile with MRI support.
  --release        Build the Release profile (default).
  --help           Show this help and exit.

Examples:
  $0 --clean
  $0 --debug VERSION=my-debug-build
  $0 AXIS=5 PAXIS=3 NO_VESC_SPINDLE=ON
EOF
}

main() {
    local gcc_version="$DEFAULT_GCC_VERSION"
    local build_type="Release"
    local machine="carvera"
    local run_clean=false
    local output_path=""
    local extra_cmake_args=()

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --gcc)
                [[ -n "${2:-}" ]] || { echo "Error: --gcc requires a version." >&2; exit 1; }
                gcc_version="$2"
                shift 2
                ;;
            --clean)
                run_clean=true
                shift
                ;;
            --output)
                [[ -n "${2:-}" ]] || { echo "Error: --output requires a path." >&2; exit 1; }
                output_path="$2"
                shift 2
                ;;
            --debug)
                build_type="Debug"
                shift
                ;;
            --release)
                build_type="Release"
                shift
                ;;
            --help)
                print_help
                return 0
                ;;
            --)
                shift
                while [[ $# -gt 0 ]]; do
                    if [[ "$1" == -D*=* ]]; then
                        extra_cmake_args+=("$1")
                    elif [[ "$1" == *=* ]]; then
                        extra_cmake_args+=("-D$1")
                    else
                        echo "Error: expected NAME=value or -DNAME=value, got '$1'." >&2
                        exit 1
                    fi
                    shift
                done
                ;;
            -D*=*)
                extra_cmake_args+=("$1")
                shift
                ;;
            -* )
                echo "Error: unknown option '$1'." >&2
                print_help >&2
                exit 1
                ;;
            *=*)
                if [[ "$1" == MACHINE=* ]]; then
                    machine="${1#MACHINE=}"
                fi
                extra_cmake_args+=("-D$1")
                shift
                ;;
            *)
                echo "Error: expected NAME=value or -DNAME=value, got '$1'." >&2
                exit 1
                ;;
        esac
    done

    [[ "$gcc_version" =~ ^[A-Za-z0-9._-]+$ ]] || { echo "Error: invalid GCC version '$gcc_version'." >&2; exit 1; }

    local gcc_env_cmd
    if ! gcc_env_cmd="$("$SCRIPT_DIR/gcc.sh" --gcc "$gcc_version" --env)"; then
        echo "Error: failed to prepare GCC $gcc_version." >&2
        exit 1
    fi

    [[ "$machine" == "carvera" || "$machine" == "z1" ]] || { echo "Error: unsupported MACHINE '$machine'." >&2; exit 1; }
    local build_dir="$PROJECT_ROOT/build/cmake/gcc-${gcc_version}/${machine}/${build_type}"
    local artifact_dir="LPC1768"
    [[ "$machine" == "z1" ]] && artifact_dir="LPC1768-z1"
    local artifact="$build_dir/$artifact_dir/firmware.bin"
    local jobs=1
    local generator_args=()
    case "$(uname -s)" in
        Linux*) jobs="$(nproc)" ;;
        Darwin*) jobs="$(sysctl -n hw.ncpu)" ;;
    esac

    (
        eval "$gcc_env_cmd"

        if [[ ! -f "$build_dir/CMakeCache.txt" ]]; then
            if command -v ninja >/dev/null 2>&1; then
                generator_args=(-G Ninja)
                echo "Using Ninja generator." >&2
            elif command -v make >/dev/null 2>&1; then
                generator_args=(-G "Unix Makefiles")
                echo "Ninja not found; using Unix Makefiles generator." >&2
            else
                echo "Error: neither ninja nor make is available for CMake." >&2
                exit 1
            fi
        fi

        echo "Configuring CMake ${build_type} build with GCC ${gcc_version}." >&2
        cmake \
            -S "$PROJECT_ROOT" \
            -B "$build_dir" \
            "${generator_args[@]}" \
            -DCMAKE_TOOLCHAIN_FILE="$PROJECT_ROOT/cmake/arm-none-eabi-toolchain.cmake" \
            -DCMAKE_BUILD_TYPE="$build_type" \
            -DAXIS=5 \
            -DPAXIS=3 \
            "${extra_cmake_args[@]}"

        if [[ "$run_clean" == true ]]; then
            echo "Cleaning $build_dir." >&2
            cmake --build "$build_dir" --target clean
        fi

        echo "Building with $jobs parallel jobs." >&2
        cmake --build "$build_dir" --parallel "$jobs"
    )

    [[ -f "$artifact" ]] || { echo "Error: build artifact not found at $artifact." >&2; exit 1; }

    if [[ -n "$output_path" ]]; then
        if [[ "$output_path" != /* ]]; then
            output_path="$ORIGINAL_PWD/$output_path"
        fi
        local destination destination_dir
        if [[ -d "$output_path" ]]; then
            destination="$output_path/firmware.bin"
            destination_dir="$output_path"
        else
            destination="$output_path"
            destination_dir="$(dirname "$output_path")"
        fi
        [[ -d "$destination_dir" ]] || { echo "Error: destination directory '$destination_dir' does not exist." >&2; exit 1; }
        cp "$artifact" "$destination"
        echo "Copied firmware.bin to $destination." >&2
    fi

    echo "CMake build finished: $artifact" >&2
}

main "$@"
