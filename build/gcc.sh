#!/usr/bin/env bash

set -euo pipefail

# Get the directory where the script resides
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." &> /dev/null && pwd)"
ORIGINAL_PWD="$PWD"

# --- Configuration ---
DEFAULT_GCC_VERSION="14.2"

# Function to map GCC version to directory name (bash 3.2 compatible)
get_gcc_dir_name() {
    local version="$1"
    case "$version" in
        "4-8")
            echo "gcc-arm-none-eabi-4.8"
            ;;
        "14-2")
            echo "gcc-arm-none-eabi-14.2"
            ;;
        *)
            echo ""
            ;;
    esac
}

# Function to check if a GCC version is supported (bash 3.2 compatible)
is_supported_gcc_version() {
    local version="$1"
    case "$version" in
        "4-8"|"14-2")
            return 0
            ;;
        *)
            return 1
            ;;
    esac
}

# Function to get all supported GCC versions (bash 3.2 compatible)
get_supported_gcc_versions() {
    echo "4-8 14-2"
}

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

# Usage: verify_hash tool hash_type expected_hash file_path
verify_hash() {
    local tool="$1"
    local hash_type="$2" # md5 or sha256
    local expected_hash="$3"
    local file_path="$4"
    local calculated_hash

    echo "Verifying ${hash_type} hash for $(basename "$file_path")..." >&2 # Use basename, output to stderr
    if [[ "$tool" == "md5sum" ]] || [[ "$tool" == "sha256sum" ]]; then
        calculated_hash=$($tool "$file_path" | awk '{ print $1 }')
    else
        echo "Error: Unsupported hash tool '$tool'. Expected 'md5sum' or 'sha256sum'." >&2
        exit 1
    fi

    if [[ "$calculated_hash" == "$expected_hash" ]]; then
        echo "Hash verified successfully." >&2 # Output to stderr
    else
        echo "Error: Hash mismatch for $(basename "$file_path")!" >&2 # Use basename
        echo "Expected: $expected_hash" >&2
        echo "Calculated: $calculated_hash" >&2
        rm -f "$file_path" # Clean up failed download
        exit 1
    fi
}

# Usage: download_and_unpack url hash_type hash gcc_version_internal archive_name hash_tool
download_and_unpack() {
    local url="$1"
    local hash_type="$2"
    local hash="$3"
    local gcc_version_internal="$4"
    local archive_name="$5"
    local hash_tool="$6"
    local temp_file
    local target_dir
    local base_path

    base_path="${TOOLCHAIN_DIR:-$PROJECT_ROOT}"
    target_dir="${base_path}/$(get_gcc_dir_name "$gcc_version_internal")"

    temp_file=$(mktemp)
    trap 'rm -f "$temp_file"' EXIT # Ensure temp file is cleaned up

    echo "Downloading $archive_name from $url..." >&2
    if ! curl -L -o "$temp_file" "$url"; then
        echo "Error: Failed to download $url" >&2
        exit 1
    fi

    verify_hash "$hash_tool" "$hash_type" "$hash" "$temp_file"

    # Calculate relative path for user message
    local display_path
    if [[ "$base_path" == "$PROJECT_ROOT" ]]; then
      # If using project root, show relative path from there
      display_path="./$(get_gcc_dir_name "$gcc_version_internal")"
    else
      # Otherwise, show the full absolute path
      display_path="$target_dir"
    fi

    echo "Unpacking $archive_name to $display_path..." >&2 # Use display_path
    mkdir -p "$target_dir"
    # strip-components=1 removes the top-level directory from the archive
    if ! tar -xf "$temp_file" -C "$target_dir" --strip-components=1; then
        echo "Error: Failed to unpack $archive_name" >&2
        rm -rf "$target_dir" # Clean up partial unpack
        exit 1
    fi

    echo "Successfully downloaded and unpacked $archive_name to $display_path." >&2 # Use display_path

    # Ensure binaries are executable
    local target_bin_dir="$target_dir/bin"
    if [[ -d "$target_bin_dir" ]]; then
        echo "Setting execute permissions for files in $target_bin_dir..." >&2
        chmod -R +x "$target_bin_dir" || echo "Warning: Failed to set execute permissions in $target_bin_dir. Build might fail." >&2
    else
        echo "Warning: Expected bin directory not found at $target_bin_dir after unpacking." >&2
    fi

    rm -f "$temp_file"
    trap - EXIT # Clear the trap
}


# --- GCC Download Functions ---

download_gcc_4_8_darwin_x86_64() {
    local url="https://launchpad.net/gcc-arm-embedded/4.8/4.8-2014-q1-update/+download/gcc-arm-none-eabi-4_8-2014q1-20140314-mac.tar.bz2"
    local hash="5d34d95a53ba545f1585b9136cbb6805"
    local hash_type="md5"
    local archive_name="gcc-arm-none-eabi-4_8-2014q1-20140314-mac.tar.bz2"
    local hash_tool="md5sum"
    local gcc_version_internal="4-8"

    download_and_unpack "$url" "$hash_type" "$hash" "$gcc_version_internal" "$archive_name" "$hash_tool"
}

download_gcc_4_8_linux_x86_64() {
    local url="https://launchpad.net/gcc-arm-embedded/4.8/4.8-2014-q1-update/+download/gcc-arm-none-eabi-4_8-2014q1-20140314-linux.tar.bz2"
    local hash="72b0d06ae16b303c25fd70b2883d3950"
    local hash_type="md5"
    local archive_name="gcc-arm-none-eabi-4_8-2014q1-20140314-linux.tar.bz2"
    local hash_tool="md5sum"
    local gcc_version_internal="4-8"

    download_and_unpack "$url" "$hash_type" "$hash" "$gcc_version_internal" "$archive_name" "$hash_tool"
}

download_gcc_14_2_darwin_arm64() {
    local url="https://developer.arm.com/-/media/Files/downloads/gnu/14.2.rel1/binrel/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi.tar.xz"
    local hash="c7c78ffab9bebfce91d99d3c24da6bf4b81c01e16cf551eb2ff9f25b9e0a3818"
    local hash_type="sha256"
    local archive_name="arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi.tar.xz"
    local hash_tool="sha256sum"
    local gcc_version_internal="14-2"

    download_and_unpack "$url" "$hash_type" "$hash" "$gcc_version_internal" "$archive_name" "$hash_tool"
}

download_gcc_14_2_linux_x86_64() {
    local url="https://developer.arm.com/-/media/Files/downloads/gnu/14.2.rel1/binrel/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi.tar.xz"
    local hash="62a63b981fe391a9cbad7ef51b17e49aeaa3e7b0d029b36ca1e9c3b2a9b78823"
    local hash_type="sha256"
    local archive_name="arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi.tar.xz"
    local hash_tool="sha256sum"
    local gcc_version_internal="14-2"

    download_and_unpack "$url" "$hash_type" "$hash" "$gcc_version_internal" "$archive_name" "$hash_tool"
}

download_gcc_14_2_linux_aarch64() {
    local url="https://developer.arm.com/-/media/Files/downloads/gnu/14.2.rel1/binrel/arm-gnu-toolchain-14.2.rel1-aarch64-arm-none-eabi.tar.xz"
    local hash="87330bab085dd8749d4ed0ad633674b9dc48b237b61069e3b481abd364d0a684"
    local hash_type="sha256"
    local archive_name="arm-gnu-toolchain-14.2.rel1-aarch64-arm-none-eabi.tar.xz"
    local hash_tool="sha256sum"
    local gcc_version_internal="14-2"

    download_and_unpack "$url" "$hash_type" "$hash" "$gcc_version_internal" "$archive_name" "$hash_tool"
}

# --- Core Logic ---

# Usage: check_gcc [version]
# Checks if the specified GCC version (e.g., "4-8" or "14-2") exists.
# If not, downloads it based on the detected OS.
# If no version specified, uses DEFAULT_GCC_VERSION.
# Returns the absolute path to the bin directory of the GCC toolchain.
check_gcc() {
    local version="${1:-$DEFAULT_GCC_VERSION}"
    local gcc_dir_name
    local gcc_path
    local os
    local base_path # For displaying relative paths correctly

    # Translate internal version (dash) back to user version (dot) for messages
    local version_user="${version/-/.}"

    if is_supported_gcc_version "$version"; then
        gcc_dir_name="$(get_gcc_dir_name "$version")"
        base_path="${TOOLCHAIN_DIR:-$PROJECT_ROOT}" # Determine the base path used
        gcc_path="${base_path}/${gcc_dir_name}" # Construct full path
    else
        echo "Error: Unsupported GCC version requested: $version" >&2
        echo "Supported versions: 4.8, 14.2" >&2
        exit 1
    fi

    # Determine relative path for messages
    local display_path
    if [[ "$base_path" == "$PROJECT_ROOT" ]]; then
      # If using project root, show relative path from there
      display_path="./${gcc_dir_name}"
    else
      # Otherwise, show the full absolute path
      display_path="$gcc_path"
    fi


    if [[ ! -d "$gcc_path/bin" ]]; then
        echo "GCC toolchain version $version_user not detected at ${display_path}." >&2
        echo "Downloading GCC $version_user..." >&2
        os=$(detect_os)
        arch=$(uname -m)
        local download_func="download_gcc_${version/-/_}_${os}_${arch}" # e.g., download_gcc_14_2_linux_x86_64

        if declare -F "$download_func" > /dev/null; then
            "$download_func" # Call the appropriate download function
        else
            echo "Error: No download function found for GCC $version_user on $os/$arch ($download_func)" >&2
            exit 1
        fi

        # Verify again after download attempt
        if [[ ! -d "$gcc_path/bin" ]]; then
             echo "Error: GCC bin directory still not found after download attempt: $gcc_path/bin for version $version_user" >&2
             exit 1
        fi
        echo "GCC version $version_user installed successfully to ${display_path}." >&2
    else
        echo "Found GCC version $version_user at ${display_path}" >&2
    fi

    # Return the absolute path to the bin directory
    echo "$gcc_path/bin"
}

print_help() {
  echo "Usage: $0 [options] [command...]"
  echo ""
  echo "Downloads the specified ARM GCC toolchain (if needed) and runs the"
  echo "provided command with the toolchain's bin directory added to the PATH."
  echo ""
  echo "Options:"
  echo "  --gcc <version>  Specify GCC version (e.g., '4.8', '14.2')."
  echo "                   Defaults to ${DEFAULT_GCC_VERSION}."
  echo "                   Supported versions: 4.8, 14.2"
  echo "  --env            Output the modified PATH string for eval."
  echo "  --help           Display this help message and exit."
  echo ""
  echo "Example:"
  echo "  $0 make -j8 CNC=1           # Builds using default GCC (${DEFAULT_GCC_VERSION})"
  echo "  $0 --gcc 4.8 make clean     # Builds using GCC 4.8"
  echo "  eval \$($0 --env)            # Add default GCC to current shell's PATH"
  echo "  eval \$($0 --gcc 4.8 --env)  # Add GCC 4.8 to current shell's PATH"

}

# --- Main Execution ---
main() {
    local requested_gcc_version_user="$DEFAULT_GCC_VERSION"
    local requested_gcc_version_internal
    local output_env=false
    local show_help=false
    local gcc_bin_path
    local cmd_args=()

    # Parse arguments
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
            --env)
                output_env=true
                shift 1
                ;;
            --help)
                show_help=true
                shift 1
                ;;
            --) # End of options
                shift
                cmd_args+=("$@")
                break
                ;;
            -*)
                echo "Error: Unknown option: $1" >&2
                print_help >&2
                exit 1
                ;;
            *) # Start of command arguments
                cmd_args+=("$@")
                break
                ;;
        esac
    done

    if [[ "$show_help" == true ]]; then
        print_help
        exit 0
    fi

    # Translate user version (dot) to internal version (dash)
    requested_gcc_version_internal="${requested_gcc_version_user//./-}"

    gcc_bin_path=$(check_gcc "$requested_gcc_version_internal")

    if [[ -z "$gcc_bin_path" ]]; then
        echo "Error: Failed to determine GCC bin path." >&2
        exit 1
    fi

    if [[ "$output_env" == true ]]; then
        echo "export PATH=\"$gcc_bin_path:\$PATH\""
        exit 0
    fi

    if [[ ${#cmd_args[@]} -eq 0 ]]; then
        echo "Error: No command provided." >&2
        print_help >&2
        exit 1
    fi

    # Execute the command from the original directory, exporting PATH
    (cd "$ORIGINAL_PWD" && export PATH="$gcc_bin_path:$PATH" && "${cmd_args[@]}")
    exit $? # Exit with the same status code as the executed command
}

main "$@"
