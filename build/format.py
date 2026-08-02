#!/usr/bin/env python3

import argparse
import difflib
import hashlib
import os
import platform
import subprocess
import sys
import tempfile
import urllib.request
import zipfile
from pathlib import Path


CLANG_FORMAT_VERSION = "22.1.8"
ASSETS = {
    ("Darwin", "arm64"): (
        "https://files.pythonhosted.org/packages/2e/55/539cc1036dae16659f50500ca34838cc5b16cd3e98e3faaf164186b98093/clang_format-22.1.8-py2.py3-none-macosx_11_0_arm64.whl",
        "d1147107222c0dda3e4869e9e8c4a79f9ed1de83819e5274de42b82adf3d2129",
    ),
    ("Darwin", "x86_64"): (
        "https://files.pythonhosted.org/packages/5d/d8/29b9db6098da1a011ca3f7560c3942fa81404dbbb4367c3bd1d5c435da3b/clang_format-22.1.8-py2.py3-none-macosx_10_9_x86_64.whl",
        "fc2ac5bd0ea41af49968fb69426207806d5f7016cb8f4bfbd44f4f1ffe8d53f2",
    ),
    ("Linux", "aarch64"): (
        "https://files.pythonhosted.org/packages/50/25/a9734da014eecc1f54c051ad643a28f2f6643dcc812ac59320e80e2b1a3b/clang_format-22.1.8-py2.py3-none-manylinux_2_26_aarch64.manylinux_2_28_aarch64.whl",
        "48c3b8dcfe9d4e964ced0e744e0f1f8ddc711bce92e50f6cab21e10f54857d08",
    ),
    ("Linux", "x86_64"): (
        "https://files.pythonhosted.org/packages/e5/88/b82c066fa807da4ca2518fecf79071361f6324b77375e5e92c059c0697fd/clang_format-22.1.8-py2.py3-none-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl",
        "b00cff6bfd1f1686f073a4fdf1cb937dbd58bf7510c659477805c03afdea0816",
    ),
    ("Windows", "AMD64"): (
        "https://files.pythonhosted.org/packages/08/60/c6783b3190a8f741107a44912a11c39c1a51e254e86a4c43cb0151cea0dd/clang_format-22.1.8-py2.py3-none-win_amd64.whl",
        "5fe6ad3e9399d589aff5ead432568a84cdcbbd621f1708340819efd74cbf8176",
    ),
    ("Windows", "ARM64"): (
        "https://files.pythonhosted.org/packages/43/ca/7e1fa4a6044c37c37356bb18fc938d2811754231e448aaffc192dc3774ce/clang_format-22.1.8-py2.py3-none-win_arm64.whl",
        "1fac18f32426c6fd7acde7087511bd80e2c549b2cd7477099582c216ae82fa63",
    ),
}
SOURCE_SUFFIXES = (".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx")


def download_clang_format(project_root: Path) -> Path:
    host = (platform.system(), platform.machine())
    asset = ASSETS.get(host)
    if asset is None:
        supported = ", ".join(f"{system}/{machine}" for system, machine in sorted(ASSETS))
        raise RuntimeError(f"unsupported host {host[0]}/{host[1]}; supported hosts: {supported}")

    executable_name = "clang-format.exe" if host[0] == "Windows" else "clang-format"
    executable = (
        project_root
        / ".scratch"
        / "clang-format"
        / CLANG_FORMAT_VERSION
        / f"{host[0]}-{host[1]}"
        / executable_name
    )
    if executable.is_file():
        return executable

    url, expected_hash = asset
    executable.parent.mkdir(parents=True, exist_ok=True)
    print(
        f"Downloading clang-format {CLANG_FORMAT_VERSION} for {host[0]}/{host[1]}...",
        file=sys.stderr,
    )
    with tempfile.TemporaryDirectory(dir=executable.parent) as temporary_directory:
        archive = Path(temporary_directory) / "clang-format.whl"
        urllib.request.urlretrieve(url, archive)
        actual_hash = hashlib.sha256(archive.read_bytes()).hexdigest()
        if actual_hash != expected_hash:
            raise RuntimeError(
                f"clang-format download checksum mismatch: expected {expected_hash}, got {actual_hash}"
            )

        member = f"clang_format/data/bin/{executable_name}"
        with zipfile.ZipFile(archive) as wheel:
            try:
                binary = wheel.read(member)
            except KeyError as error:
                raise RuntimeError(f"clang-format executable is missing from the downloaded wheel") from error

        temporary_executable = Path(temporary_directory) / executable_name
        temporary_executable.write_bytes(binary)
        if host[0] != "Windows":
            temporary_executable.chmod(0o755)
        os.replace(temporary_executable, executable)

    return executable


def tracked_sources(project_root: Path) -> list[Path]:
    result = subprocess.run(
        ["git", "-C", str(project_root), "ls-files", "-z"],
        check=True,
        stdout=subprocess.PIPE,
    )
    paths = []
    for raw_path in result.stdout.split(b"\0"):
        if not raw_path:
            continue
        relative_path = Path(os.fsdecode(raw_path))
        if relative_path.parts[0] == "attic" or relative_path.suffix.lower() not in SOURCE_SUFFIXES:
            continue
        paths.append(project_root / relative_path)
    return paths


def selected_sources(project_root: Path, arguments: list[str]) -> list[Path]:
    if not arguments:
        return tracked_sources(project_root)

    sources = []
    seen = set()
    for argument in arguments:
        source = Path(argument)
        if not source.is_absolute():
            source = Path.cwd() / source
        source = source.resolve()
        try:
            relative_path = source.relative_to(project_root)
        except ValueError as error:
            raise RuntimeError(f"source is outside the project: {argument}") from error
        if not source.is_file():
            raise RuntimeError(f"source file not found: {argument}")
        if relative_path.parts[0] == "attic":
            raise RuntimeError(f"attic sources are excluded: {argument}")
        if source.suffix.lower() not in SOURCE_SUFFIXES:
            raise RuntimeError(f"not a C/C++ source file: {argument}")
        if source not in seen:
            sources.append(source)
            seen.add(source)
    return sources


def check_sources(clang_format: Path, project_root: Path, sources: list[Path]) -> int:
    differences = False
    for source in sources:
        original = source.read_bytes()
        result = subprocess.run(
            [str(clang_format), "--style=file", str(source)],
            cwd=project_root,
            check=True,
            stdout=subprocess.PIPE,
        )
        if result.stdout == original:
            continue

        differences = True
        relative_path = source.relative_to(project_root).as_posix().encode()
        diff = b"".join(
            difflib.diff_bytes(
                difflib.unified_diff,
                original.splitlines(keepends=True),
                result.stdout.splitlines(keepends=True),
                fromfile=b"a/" + relative_path,
                tofile=b"b/" + relative_path,
            )
        )
        sys.stdout.buffer.write(diff)
        if diff and not diff.endswith(b"\n"):
            sys.stdout.buffer.write(b"\n")
    return 1 if differences else 0


def format_sources(clang_format: Path, project_root: Path, sources: list[Path]) -> None:
    print(
        f"Formatting {len(sources)} C/C++ files with clang-format {CLANG_FORMAT_VERSION}...",
        file=sys.stderr,
    )
    # Some legacy macro layouts need a second clang-format pass to stabilize.
    for _ in range(2):
        for offset in range(0, len(sources), 64):
            subprocess.run(
                [str(clang_format), "-i", "--style=file", *map(str, sources[offset : offset + 64])],
                cwd=project_root,
                check=True,
            )
    print("Formatting complete.", file=sys.stderr)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Download a pinned clang-format and check or format C/C++ sources."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    for command in ("check", "format"):
        command_parser = subparsers.add_parser(command)
        command_parser.add_argument("files", nargs="*", metavar="FILES")
    arguments = parser.parse_args()

    project_root = Path(__file__).resolve().parent.parent
    try:
        clang_format = download_clang_format(project_root)
        sources = selected_sources(project_root, arguments.files)
        if not sources:
            raise RuntimeError("no tracked C/C++ sources found")
        if arguments.command == "check":
            return check_sources(clang_format, project_root, sources)
        format_sources(clang_format, project_root, sources)
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
