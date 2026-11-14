#!/usr/bin/env python3
"""Require new G/M/console command IDs in a PR diff to exist in docs/commands.json.

This is a conservative scan of added lines. It looks for handler-style matches
such as gcode->m == 576, SimpleShell table rows, and cmd == \"play\". It does
not try to prove the JSON is complete or that parameters are documented.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
COMMANDS_JSON = REPO_ROOT / "docs" / "commands.json"

IGNORE_LINE_RE = re.compile(
    r"script_queue\.push|snprintf\s*\(|printf\s*\(|printfcmd\s*\(|"
    r"PacketMessage\s*\(|dispatch_gcode\s*\(|Gcode\s+\w+\s*\(\s*\"|"
    r"message\.message\s*="
)

M_EQ_RE = re.compile(r"gcode->m\s*==\s*(\d+)")
G_EQ_RE = re.compile(r"gcode->g\s*==\s*(\d+)")
SUBCODE_RE = re.compile(r"(?:gcode->)?subcode\s*==\s*(\d+)")
CMD_EQ_RE = re.compile(r'\bcmd\s*==\s*"([a-z][a-z0-9_-]*)"')
SHELL_TABLE_RE = re.compile(r'\{\s*"([a-zA-Z?][a-zA-Z0-9_-]*)"\s*,')
DOLLAR_CASE_RE = re.compile(r"case\s+'([A-Za-z#])'\s*:")
CASE_NUM_RE = re.compile(r"case\s+(\d+)\s*:")
CONFIG_M_RE = re.compile(r"input_(?:on|off)_command\s+M(\d+)\b")
COMMENT_START_ID_RE = re.compile(r"^\s*([GM]\d+(?:\.\d+)?)")
SWITCH_G_RE = re.compile(r"switch\s*\(\s*gcode->g")
SWITCH_M_RE = re.compile(r"switch\s*\(\s*gcode->m")

SKIP_SHELL_NAMES = {
    "ok",
    "error",
    "laser",  # handled by Laser module; not a documented console command
}

# IDs that show up in handler diffs but are not public commands to document.
ALLOWLIST = set()

SOURCE_PREFIXES = (
    "src/",
    "src/config.default",
    "src/config2.default",
)


def run_git(args: list[str]) -> str:
    return subprocess.check_output(
        ["git", *args],
        cwd=REPO_ROOT,
        text=True,
        stderr=subprocess.DEVNULL,
    )


def discover_diff_range() -> str | None:
    explicit = os_environ_get("COMMANDS_JSON_DIFF_RANGE")
    if explicit:
        return explicit

    event_path = os_environ_get("GITHUB_EVENT_PATH")
    if event_path:
        import json as _json

        try:
            payload = _json.loads(Path(event_path).read_text())
        except (OSError, ValueError):
            payload = {}
        base_ref = (payload.get("pull_request") or {}).get("base", {}).get("ref")
        base_sha = (payload.get("pull_request") or {}).get("base", {}).get("sha")
        if base_sha:
            try:
                run_git(["cat-file", "-e", f"{base_sha}^{{commit}}"])
                return f"{base_sha}...HEAD"
            except subprocess.CalledProcessError:
                pass
        if base_ref:
            return f"origin/{base_ref}...HEAD"

    for candidate in ("upstream/Dev", "origin/Dev"):
        try:
            merge_base = run_git(["merge-base", "HEAD", candidate]).strip()
        except subprocess.CalledProcessError:
            continue
        if merge_base:
            return f"{merge_base}...HEAD"
    return None


def os_environ_get(name: str) -> str | None:
    import os

    value = os.environ.get(name)
    return value if value else None


def changed_source_files(diff_range: str) -> list[str]:
    names = run_git(["diff", "--name-only", diff_range]).splitlines()
    return [n for n in names if n.startswith("src/")]


def parse_unified_diff(diff_text: str) -> list[tuple[str, list[tuple[str, str]]]]:
    hunks: list[tuple[str, list[tuple[str, str]]]] = []
    path = ""
    lines: list[tuple[str, str]] = []

    def flush() -> None:
        nonlocal lines
        if path and lines:
            hunks.append((path, lines))
        lines = []

    for raw in diff_text.splitlines():
        if raw.startswith("diff --git "):
            flush()
            path = ""
            continue
        if raw.startswith("+++ b/"):
            path = raw[6:]
            continue
        if raw.startswith("@@"):
            if lines:
                flush()
                # keep path for the next hunk of the same file
            continue
        if not path:
            continue
        if raw.startswith("+") and not raw.startswith("+++"):
            lines.append(("add", raw[1:]))
        elif raw.startswith("-") and not raw.startswith("---"):
            lines.append(("remove", raw[1:]))
        elif raw.startswith(" "):
            lines.append(("context", raw[1:]))
    flush()
    return hunks


def interesting_path(path: str) -> bool:
    return path.startswith(SOURCE_PREFIXES) or path in {
        "src/config.default",
        "src/config2.default",
    }


def should_ignore_line(line: str) -> bool:
    return bool(IGNORE_LINE_RE.search(line))


def comment_text(line: str) -> str:
    if "//" in line:
        return line.split("//", 1)[1]
    if "/*" in line:
        return line.split("/*", 1)[1]
    return ""


def handler_id_from_comment(line: str) -> str | None:
    comment = comment_text(line)
    if not comment:
        return None
    match = COMMENT_START_ID_RE.match(comment)
    return match.group(1) if match else None


def nearest_parent_code(previous_lines: list[str]) -> tuple[str, int] | None:
    for text in reversed(previous_lines[-8:]):
        match = M_EQ_RE.search(text)
        if match:
            return "M", int(match.group(1))
        match = G_EQ_RE.search(text)
        if match:
            return "G", int(match.group(1))
    return None


def extract_from_hunk(path: str, hunk_lines: list[tuple[str, str]]) -> set[str]:
    found: set[str] = set()
    visible_so_far: list[str] = []
    current_switch = None
    dollar_switch = False

    for kind, line in hunk_lines:
        if kind != "remove":
            visible_so_far.append(line)
            if SWITCH_G_RE.search(line):
                current_switch = "g"
            elif SWITCH_M_RE.search(line):
                current_switch = "m"
            if "possible_command[0] == '$'" in line or "grbl compatible command" in line:
                dollar_switch = True

        if kind != "add":
            continue
        if should_ignore_line(line):
            continue

        for n in M_EQ_RE.findall(line):
            found.add(f"M{n}")
        for n in G_EQ_RE.findall(line):
            found.add(f"G{n}")

        for n in SUBCODE_RE.findall(line):
            sub = int(n)
            if sub <= 0:
                continue
            parent = nearest_parent_code(visible_so_far)
            if parent:
                letter, num = parent
                found.add(f"{letter}{num}")
                found.add(f"{letter}{num}.{sub}")

        handler_id = handler_id_from_comment(line)
        if handler_id:
            found.add(handler_id)

        if path.endswith("SimpleShell.cpp"):
            for name in SHELL_TABLE_RE.findall(line):
                if name not in SKIP_SHELL_NAMES:
                    found.add(name)
            if dollar_switch:
                for letter in DOLLAR_CASE_RE.findall(line):
                    found.add(f"${letter}")

        if path.endswith(("SimpleShell.cpp", "Player.cpp")):
            for name in CMD_EQ_RE.findall(line):
                if name not in SKIP_SHELL_NAMES:
                    found.add(name)

        if path.endswith(("config.default", "config2.default")):
            for n in CONFIG_M_RE.findall(line):
                found.add(f"M{n}")

        if path.endswith(("Robot.cpp", "GcodeDispatch.cpp")):
            for n in CASE_NUM_RE.findall(line):
                if handler_id:
                    continue
                if current_switch == "g":
                    found.add(f"G{n}")
                elif current_switch == "m":
                    found.add(f"M{n}")

    return found


def extract_from_diff(diff_text: str) -> set[str]:
    found: set[str] = set()
    for path, hunk in parse_unified_diff(diff_text):
        if interesting_path(path):
            found |= extract_from_hunk(path, hunk)
    return found - ALLOWLIST


def load_documented() -> set[str]:
    data = json.loads(COMMANDS_JSON.read_text())
    commands = data.get("commands")
    if not isinstance(commands, dict):
        raise ValueError("docs/commands.json is missing a commands object")
    return set(commands.keys())


def check(diff_range: str) -> int:
    if not changed_source_files(diff_range):
        print("No files changed in src/. Skipping commands.json check.")
        return 0

    if not COMMANDS_JSON.is_file():
        print("docs/commands.json is missing.")
        print()
        print("New commands need a matching key in docs/commands.json.")
        return 1

    try:
        documented = load_documented()
    except ValueError as exc:
        print(f"docs/commands.json is not valid: {exc}")
        return 1

    diff_text = run_git(["diff", "-U8", diff_range, "--", "src"])
    discovered = extract_from_diff(diff_text)
    missing = sorted(discovered - documented)

    if not discovered:
        print("No new G/M/console command IDs found in the src/ diff.")
        print(f"(Checked diff range: {diff_range})")
        return 0

    if not missing:
        print(f"Documented {len(discovered)} command ID(s) added in this PR.")
        print(f"(Checked diff range: {diff_range})")
        return 0

    print("Missing commands.json entries for new command IDs")
    print()
    print("These command IDs appear in the src/ diff but are not keys in docs/commands.json:")
    for ident in missing:
        print(f"  - {ident}")
    print()
    print("Add a matching key under commands in docs/commands.json with a short")
    print("description and any parameters (required, description, default).")
    print(f"(Checked diff range: {diff_range})")
    return 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--diff-range", help="Git three-dot range to scan")
    args = parser.parse_args()

    diff_range = args.diff_range or discover_diff_range()
    if not diff_range:
        print("Could not determine a base revision. Skipping commands.json check.")
        return 0
    return check(diff_range)


if __name__ == "__main__":
    sys.exit(main())
