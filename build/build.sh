#!/usr/bin/env bash

set -euo pipefail

# Get the directory where the script resides
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." &> /dev/null && pwd)"
ORIGINAL_PWD="$PWD"

# --- Configuration ---
DEFAULT_GCC_VERSION="14.2" # Match the default in gcc.sh
BASE_MAKE_ARGS="AXIS=5 PAXIS=3 CNC=1" # Standard args for this firmware

# --- Helper Functions ---

# Usage: detect_os
# Returns "linux" or "darwin"
detect_os() {
    local uname_s
    uname_s="$(uname -s)"
    case "${uname_s}" in
        Linux*)     echo "linux";;
        Darwin*)    echo "darwin";;
        *)          echo "Unsupported OS: ${uname_s}" >&2; exit 1;;
    esac
}

# Usage: get_cpu_count os_type
get_cpu_count() {
    local os_type="$1"
    case "$os_type" in
        linux)  nproc ;;
        darwin) sysctl -n hw.ncpu ;;
        *)      echo "1" ;; # Default to 1 core if OS detection failed
    esac
}


print_help() {
  echo "Usage: $0 [options] [make_variable=value...]"
  echo ""
  echo "Runs the firmware build process using the specified ARM GCC toolchain."
  echo "Passes standard arguments (AXIS=5 PAXIS=3 CNC=1) and any additional"
  echo "key=value pairs directly to make."
  echo ""
  echo "Options:"
  echo "  --gcc <version>  Specify GCC version (e.g., '4.8', '14.2')."
  echo "                   Defaults to ${DEFAULT_GCC_VERSION}."
  echo "                   Supported versions are determined by build/gcc.sh."
  echo "  --clean          Run 'make clean' before starting the build."
  echo "  --output <path>  Specify the output path for the build artifact."
  echo "  --help           Display this help message and exit."
  echo ""
  echo "Example:"
  echo "  $0                            # Build with default GCC (${DEFAULT_GCC_VERSION})"
  echo "  $0 --clean                    # Clean then build with default GCC"
  echo "  $0 --gcc 4.8 --clean VERBOSE=1 # Clean then build with GCC 4.8, verbose output"
  echo "  $0 ENABLE_DEBUG_MONITOR=1     # Build with default GCC and enable debug monitor"
}

# --- Main Execution ---
main() {
    local requested_gcc_version_user="$DEFAULT_GCC_VERSION"
    local run_clean=false
    local show_help=false
    local make_extra_args=()
    local output_path=""
    local os
    local cpu_count
    local make_cmd
    local gcc_env_cmd

    # --- Argument Parsing ---
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --gcc)
                if [[ -z "${2:-}" ]]; then
                    echo "Error: --gcc option requires a version argument." >&2
                    print_help >&2
                    exit 1
                fi
                requested_gcc_version_user="$2"
                shift 2
                ;;
            --clean)
                run_clean=true
                shift 1
                ;;
            --output)
                if [[ -z "${2:-}" ]]; then
                    echo "Error: --output option requires a path argument." >&2
                    print_help >&2
                    exit 1
                fi
                output_path="$2"
                shift 2
                ;;
            --help)
                show_help=true
                shift 1
                ;;
            --) # End of options
                shift
                make_extra_args+=("$@")
                break
                ;;
            -*)
                echo "Error: Unknown option: $1" >&2
                print_help >&2
                exit 1
                ;;
            *) # Start of make variable arguments
                make_extra_args+=("$@")
                break
                ;;
        esac
    done

    if [[ "$show_help" == true ]]; then
        print_help
        exit 0
    fi

    # --- Environment and Build Setup ---
    os=$(detect_os)
    cpu_count=$(get_cpu_count "$os")

    echo "Detected OS: $os" >&2
    echo "Using GCC version: $requested_gcc_version_user" >&2
    echo "Using $cpu_count parallel jobs for make." >&2
    [[ "$run_clean" == true ]] && echo "Will run 'make clean' first." >&2

    # Get the PATH export command from gcc.sh
    gcc_env_cmd="$("$SCRIPT_DIR/gcc.sh" --gcc "$requested_gcc_version_user" --env)"
    if [[ $? -ne 0 || -z "$gcc_env_cmd" ]]; then
        echo "Error: Failed to get PATH from gcc.sh for version $requested_gcc_version_user" >&2
        exit 1
    fi

    # Construct the final make command
    # We need eval here to correctly handle potential spaces or special chars
    # in make_extra_args if they were quoted on the command line.
    make_cmd=(make "-j${cpu_count}" ${BASE_MAKE_ARGS})
    make_cmd+=(${make_extra_args[@]+"${make_extra_args[@]}"}) # Append extra args

    # --- Execution ---
    # Use a subshell to isolate the PATH modification
    (
        # Set the PATH for this subshell
        eval "$gcc_env_cmd"

        echo "GH debug: $PATH"

        # Change to project root relative to the script's location
        cd "$PROJECT_ROOT"

        if [[ "$run_clean" == true ]]; then
            echo "Running: make clean" >&2
            if ! make clean; then
                echo "Error: 'make clean' failed." >&2
                exit 1
            fi
            echo "'make clean' finished." >&2
        fi

        echo "Running: ${make_cmd[*]}" >&2
        # Use "${make_cmd[@]}" to handle args with spaces correctly
        if ! "${make_cmd[@]}"; then
             echo "Error: Build failed." >&2
             exit 1
        fi
        echo "Build finished successfully." >&2

        # --- Handle Output Copy ---
        if [[ -n "$output_path" ]]; then
            local source_file="$PROJECT_ROOT/LPC1768/main.bin"
            local dest_path
            local dest_dir

            if [[ ! -f "$source_file" ]]; then
                echo "Error: Build artifact not found at $source_file" >&2
                exit 1
            fi

            # Determine absolute path for output_path if it's relative
            if [[ "$output_path" != /* ]]; then
                output_path="$ORIGINAL_PWD/$output_path"
            fi

            # Check if output_path is a directory
            if [[ -d "$output_path" ]]; then
                dest_path="$output_path/main.bin"
                dest_dir="$output_path"
            else
                # Assume output_path includes the filename
                dest_path="$output_path"
                dest_dir=$(dirname "$output_path")
            fi

            # Check if destination directory exists
            if [[ ! -d "$dest_dir" ]]; then
                echo "Error: Destination directory $dest_dir does not exist." >&2
                exit 1
            fi

             # Perform the copy
             echo "Copying $source_file to $dest_path" >&2
             if ! cp "$source_file" "$dest_path"; then
                 echo "Error: Failed to copy build artifact to $dest_path" >&2
                 exit 1
             fi
             echo "Successfully copied artifact to $dest_path" >&2
        fi
    )
    exit $? # Exit with the status code of the subshell (and thus the make command)
}

# Ensure the script runs from the project root contextually for make
# but execute the command itself from the original directory if needed (like gcc.sh does)
# However, for make, it's better to run it from the project root.
# main "$@" # Call main with all script arguments
# The subshell above handles changing directory, so just call main.
main "$@"

