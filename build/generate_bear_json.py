import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

# --- Configuration ---
TEMPLATE_FILE = Path("bear.json.template")
OUTPUT_FILE = Path("bear.json")
GCC_EXECUTABLE = "arm-none-eabi-gcc"
GPP_EXECUTABLE = "arm-none-eabi-g++"
NULL_DEVICE = "/dev/null" if sys.platform != "win32" else "nul"


# --- Helper Functions ---
def find_executable(name):
    """Finds an executable in the system PATH."""
    path = shutil.which(name)
    if not path:
        print(f"Error: Executable '{name}' not found in PATH.", file=sys.stderr)
        sys.exit(1)
    return Path(path).resolve()

def get_gpp_include_paths(gpp_path):
    """Runs g++ with verbose preprocessor output to find include paths."""
    cmd = [str(gpp_path), "-E", "-Wp,-v", "-xc++", NULL_DEVICE]
    print(f"Running: {' '.join(cmd)}", file=sys.stderr)
    try:
        # We capture stderr because that's where -Wp,-v prints the paths
        result = subprocess.run(cmd, capture_output=True, text=True, check=True, encoding='utf-8')
        output = result.stderr
    except subprocess.CalledProcessError as e:
        print(f"Error running g++ to find include paths:", file=sys.stderr)
        print(f"  Command: {' '.join(e.cmd)}", file=sys.stderr)
        print(f"  Return Code: {e.returncode}", file=sys.stderr)
        print(f"  Stderr: {e.stderr}", file=sys.stderr)
        print(f"  Stdout: {e.stdout}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Unexpected error running g++: {e}", file=sys.stderr)
        sys.exit(1)

    # Parse the output
    include_paths = []
    in_search_list = False
    for line in output.splitlines():
        if line.strip() == "#include <...> search starts here:":
            in_search_list = True
        elif line.strip() == "End of search list.":
            in_search_list = False
        elif in_search_list and line.startswith(" "):
            # Paths are usually indented
            path = line.strip()
            # Sometimes paths might be quoted, remove if necessary
            if path.startswith('"') and path.endswith('"'):
                path = path[1:-1]
            # Resolve to absolute paths for consistency
            include_paths.append(str(Path(path).resolve()))

    if not include_paths:
        print("Error: Could not parse include paths from g++ output.", file=sys.stderr)
        print("--- g++ stderr ---", file=sys.stderr)
        print(output, file=sys.stderr)
        print("------------------", file=sys.stderr)
        sys.exit(1)

    print("Detected include paths:", file=sys.stderr)
    for p in include_paths:
        print(f"  {p}", file=sys.stderr)
    return include_paths


def derive_paths(gcc_path, gpp_path):
    """Derives toolchain root and include paths from executable paths and compiler output."""
    gcc_bin_dir = gcc_path.parent
    gpp_bin_dir = gpp_path.parent

    if gcc_bin_dir != gpp_bin_dir:
        print(f"Warning: {GCC_EXECUTABLE} and {GPP_EXECUTABLE} seem to be in different directories.", file=sys.stderr)
        print(f"  GCC: {gcc_bin_dir}", file=sys.stderr)
        print(f"  G++: {gpp_bin_dir}", file=sys.stderr)
        print("Using GCC's parent directory to determine toolchain root.", file=sys.stderr)

    toolchain_root = gcc_bin_dir.parent
    print(f"Tentative Toolchain Root: {toolchain_root}") # May not be entirely accurate if mixed toolchains

    # Get paths directly from G++ output
    gpp_includes = get_gpp_include_paths(gpp_path)

    # --- Attempt to map extracted paths to template placeholders ---
    # This requires making educated guesses based on typical path structures.
    # It might need adjustments depending on the toolchain layout.
    paths = {
        "__ARM_NONE_EABI_GCC_PATH__": str(gcc_path),
        "__ARM_NONE_EABI_GPP_PATH__": str(gpp_path),
        "__GCC_INCLUDE_DIR__": None,         # General C includes
        "__GPP_SYS_INCLUDE_DIR_1__": None,    # Base C++ includes (e.g., .../c++/X.Y.Z)
        "__GPP_SYS_INCLUDE_DIR_2__": None,    # Target-specific C++ (e.g., .../X.Y.Z/arm-none-eabi/...)
        "__GPP_SYS_INCLUDE_DIR_3__": None,    # Backward compatibility C++ (e.g., .../X.Y.Z/backward)
        "__GPP_SYS_INCLUDE_DIR_4__": None,    # libgcc includes (e.g., .../lib/gcc/.../include)
        "__GPP_SYS_INCLUDE_DIR_5__": None,    # libgcc fixed includes (e.g., .../lib/gcc/.../include-fixed)
        "__GPP_SYS_INCLUDE_DIR_6__": None,    # Duplicate of __GCC_INCLUDE_DIR__ in original template
    }
    found_indices = set()

    # 1. Find general arm-none-eabi/include (often contains sys/stdio.h etc.)
    for i, p in enumerate(gpp_includes):
        if p.endswith(os.path.join("arm-none-eabi", "include")):
            paths["__GCC_INCLUDE_DIR__"] = p
            paths["__GPP_SYS_INCLUDE_DIR_6__"] = p
            found_indices.add(i)
            break

    # 2. Find include-fixed
    for i, p in enumerate(gpp_includes):
        if i not in found_indices and "include-fixed" in p:
            paths["__GPP_SYS_INCLUDE_DIR_5__"] = p
            found_indices.add(i)
            break

    # 3. Find basic libgcc include (must not be include-fixed)
    for i, p in enumerate(gpp_includes):
        if i not in found_indices and os.path.join("lib", "gcc") in p and "include" in os.path.basename(p) and "fixed" not in os.path.basename(p):
             paths["__GPP_SYS_INCLUDE_DIR_4__"] = p
             found_indices.add(i)
             break

    # 4. Find C++ backward include
    for i, p in enumerate(gpp_includes):
        if i not in found_indices and os.path.join("c++", "backward") in p or os.path.join("backward") == os.path.basename(p) :
             paths["__GPP_SYS_INCLUDE_DIR_3__"] = p
             found_indices.add(i)
             break

    # 5. Find target-specific C++ include (contains /c++/VERSION/TARGET_TRIPLET )
    for i, p in enumerate(gpp_includes):
         # Regex to find .../c++/X.Y.Z/arm-none-eabi/... pattern
         # This is complex because the target triplet can vary
         if i not in found_indices and re.search(r"c\+\+/[^/]+/" + re.escape("arm-none-eabi"), p):
             paths["__GPP_SYS_INCLUDE_DIR_2__"] = p
             found_indices.add(i)
             break

    # 6. Find base C++ include (contains /c++/VERSION but not backward or target)
    for i, p in enumerate(gpp_includes):
        if i not in found_indices and os.path.join("c++") in p:
            # Check it wasn't already matched as target-specific or backward
            is_target_specific = paths["__GPP_SYS_INCLUDE_DIR_2__"] and p.startswith(paths["__GPP_SYS_INCLUDE_DIR_2__"])
            is_backward = paths["__GPP_SYS_INCLUDE_DIR_3__"] and p.startswith(paths["__GPP_SYS_INCLUDE_DIR_3__"])
            if not is_target_specific and not is_backward:
                 paths["__GPP_SYS_INCLUDE_DIR_1__"] = p
                 found_indices.add(i)
                 break

    # --- Verification and Fallback ---
    missing_keys = [key for key, val in paths.items() if val is None and "PATH" not in key]
    if missing_keys:
        print("Warning: Could not automatically map all include paths to template placeholders:", file=sys.stderr)
        print(f"  Missing mappings for: {', '.join(missing_keys)}", file=sys.stderr)
        print("  Please check the generated bear.json carefully.", file=sys.stderr)
        # Attempt a simple fallback for the general C include if missing
        if paths["__GCC_INCLUDE_DIR__"] is None:
            fallback_c_include = toolchain_root / "arm-none-eabi" / "include"
            if fallback_c_include.exists():
                print(f"  Attempting fallback for __GCC_INCLUDE_DIR__: {fallback_c_include}", file=sys.stderr)
                paths["__GCC_INCLUDE_DIR__"] = str(fallback_c_include)
                paths["__GPP_SYS_INCLUDE_DIR_6__"] = str(fallback_c_include)
            else:
                 print(f"  Fallback path {fallback_c_include} does not exist.", file=sys.stderr)


    print("Mapped paths:", file=sys.stderr)
    for key, val in paths.items():
         print(f"  {key}: {val}", file=sys.stderr)


    # Final check for existence of mapped paths (excluding None)
    for key, p_str in paths.items():
        if "PATH" not in key and p_str is not None:
            p = Path(p_str)
            if not p.exists():
                 # Use repr for cleaner path output, especially on Windows
                 print(f"Warning: Mapped path for {key} does not exist: {p!r}", file=sys.stderr)

    return paths


# --- Main Execution ---
if __name__ == "__main__":
    # Ensure the script runs relative to the build directory
    script_dir = Path(__file__).parent.resolve()
    os.chdir(script_dir)
    print(f"Running in directory: {script_dir}")


    print(f"Generating {OUTPUT_FILE} from {TEMPLATE_FILE}...")

    if not TEMPLATE_FILE.exists():
        print(f"Error: Template file '{TEMPLATE_FILE}' not found.", file=sys.stderr)
        print(f"Current directory: {Path.cwd()}", file=sys.stderr)
        sys.exit(1)

    # Find executables
    gcc_exe_path = find_executable(GCC_EXECUTABLE)
    gpp_exe_path = find_executable(GPP_EXECUTABLE)

    # Derive paths using the new method
    replacements = derive_paths(gcc_exe_path, gpp_exe_path)

    # Read template
    try:
        template_content = TEMPLATE_FILE.read_text()
    except Exception as e:
        print(f"Error reading template file '{TEMPLATE_FILE}': {e}", file=sys.stderr)
        sys.exit(1)

    # Perform substitutions
    output_content = template_content
    for placeholder, value in replacements.items():
        if value is None:
            print(f"Warning: No value found for placeholder {placeholder}. Leaving it unchanged in {OUTPUT_FILE}.", file=sys.stderr)
            continue # Skip replacing if no value was found/mapped

        # Escape backslashes for JSON compatibility (e.g., C:\path -> C:\\path)
        # Needed primarily for Windows paths.
        escaped_value = value.replace('\\', '\\\\')
        output_content = output_content.replace(placeholder, escaped_value)


    # Write output file
    try:
        OUTPUT_FILE.write_text(output_content)
        print(f"Successfully generated {OUTPUT_FILE}")
    except Exception as e:
        print(f"Error writing output file '{OUTPUT_FILE}': {e}", file=sys.stderr)
        sys.exit(1)

    # Optional: Make script executable (usually needed on Linux/macOS)
    if sys.platform != "win32":
        try:
            current_mode = os.stat(__file__).st_mode
            # Add execute permissions for user, group, and others if not already set
            new_mode = current_mode | 0o111
            if current_mode != new_mode:
                os.chmod(__file__, new_mode)
                print(f"Made script {Path(__file__).name} executable.")
        except OSError as e:
            print(f"Warning: Could not make script executable: {e}", file=sys.stderr) 