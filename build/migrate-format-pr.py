#!/usr/bin/env python3

"""Migrate an open PR across the repository-wide clang-format commit."""

import argparse
import json
import shlex
import subprocess
import sys
from pathlib import Path


FORMAT_COMMIT_SUBJECT = "format sources"
SOURCE_SUFFIXES = (".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx")


def run(
    arguments: list[str],
    *,
    cwd: Path,
    capture_output: bool = False,
) -> subprocess.CompletedProcess[str]:
    print(f"+ {shlex.join(arguments)}", file=sys.stderr)
    return subprocess.run(
        arguments,
        cwd=cwd,
        check=True,
        text=True,
        stdout=subprocess.PIPE if capture_output else None,
    )


def output(arguments: list[str], *, cwd: Path) -> str:
    return run(arguments, cwd=cwd, capture_output=True).stdout.strip()


def git(arguments: list[str], *, cwd: Path, capture_output: bool = False) -> subprocess.CompletedProcess[str]:
    return run(["git", *arguments], cwd=cwd, capture_output=capture_output)


def git_output(arguments: list[str], *, cwd: Path) -> str:
    return output(["git", *arguments], cwd=cwd)


def require_clean_tracked_files(project_root: Path) -> None:
    status = git_output(["status", "--porcelain", "--untracked-files=no"], cwd=project_root)
    if status:
        raise RuntimeError("tracked files contain local changes; commit or stash them before migrating a PR")


def find_format_commit(project_root: Path, dev_ref: str) -> str:
    history = git_output(
        ["log", "--first-parent", "--format=%H%x00%s", dev_ref],
        cwd=project_root,
    )
    for entry in history.splitlines():
        commit, subject = entry.split("\0", 1)
        if subject == FORMAT_COMMIT_SUBJECT:
            return commit
    raise RuntimeError(
        f"could not find a first-parent commit named {FORMAT_COMMIT_SUBJECT!r} on {dev_ref}; "
        "pass it explicitly with --format-commit"
    )


def validate_format_commit(project_root: Path, format_commit: str) -> tuple[str, str]:
    format_commit = git_output(["rev-parse", "--verify", f"{format_commit}^{{commit}}"], cwd=project_root)
    pre_format_commit = git_output(["rev-parse", "--verify", f"{format_commit}^"], cwd=project_root)
    git(["cat-file", "-e", f"{pre_format_commit}:build/format.py"], cwd=project_root)

    changed_files = git_output(
        ["diff-tree", "--no-commit-id", "--name-only", "-r", format_commit],
        cwd=project_root,
    ).splitlines()
    unexpected = [path for path in changed_files if not path.lower().endswith(SOURCE_SUFFIXES)]
    if not changed_files or unexpected:
        details = ", ".join(unexpected[:5]) or "no changed files"
        raise RuntimeError(f"{format_commit} does not look like a source-only formatting commit: {details}")
    return format_commit, pre_format_commit


def pr_details(project_root: Path, pr: str, repository: str | None) -> dict[str, object]:
    command = [
        "gh",
        "pr",
        "view",
        pr,
        "--json",
        "number,state,baseRefName,headRefName,url",
    ]
    if repository:
        command.extend(["--repo", repository])
    return json.loads(output(command, cwd=project_root))


def checkout_pr(project_root: Path, pr: str, repository: str | None) -> None:
    command = ["gh", "pr", "checkout", pr]
    if repository:
        command.extend(["--repo", repository])
    run(command, cwd=project_root)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Make a PR mergeable across the one-time repository-wide formatting commit."
    )
    parser.add_argument("pr", help="PR number or URL")
    parser.add_argument("--remote", default="upstream", help="remote containing the base branch (default: upstream)")
    parser.add_argument("--base", default="Dev", help="formatted base branch (default: Dev)")
    parser.add_argument("--repo", help="GitHub OWNER/REPO used by gh")
    parser.add_argument("--format-commit", help="override automatic discovery of the 'format sources' commit")
    parser.add_argument("--push", action="store_true", help="push the migrated PR branch after verification")
    parser.add_argument("--dry-run", action="store_true", help="inspect the PR and migration commits only")
    arguments = parser.parse_args()

    try:
        project_root = Path(
            output(["git", "rev-parse", "--show-toplevel"], cwd=Path.cwd())
        )
        require_clean_tracked_files(project_root)

        git(["fetch", arguments.remote, arguments.base], cwd=project_root)
        dev_ref = f"{arguments.remote}/{arguments.base}"
        candidate = arguments.format_commit or find_format_commit(project_root, dev_ref)
        format_commit, pre_format_commit = validate_format_commit(project_root, candidate)

        details = pr_details(project_root, arguments.pr, arguments.repo)
        if details["state"] != "OPEN":
            raise RuntimeError(f"PR #{details['number']} is {str(details['state']).lower()}, not open")
        if details["baseRefName"] != arguments.base:
            raise RuntimeError(
                f"PR #{details['number']} targets {details['baseRefName']}, not {arguments.base}"
            )

        print(f"PR:              {details['url']}")
        print(f"PR branch:       {details['headRefName']}")
        print(f"Pre-format base: {pre_format_commit}")
        print(f"Format commit:   {format_commit}")
        if arguments.dry_run:
            return 0

        checkout_pr(project_root, arguments.pr, arguments.repo)
        require_clean_tracked_files(project_root)

        git(["merge", "--no-edit", pre_format_commit], cwd=project_root)
        ancestor_check = subprocess.run(
            ["git", "merge-base", "--is-ancestor", format_commit, "HEAD"],
            cwd=project_root,
        )
        if ancestor_check.returncode != 0:
            git(
                [
                    "merge",
                    "-s",
                    "ours",
                    format_commit,
                    "-m",
                    "Bridge repository-wide formatting migration",
                ],
                cwd=project_root,
            )

        run([sys.executable, "build/format.py", "format"], cwd=project_root)
        git(["add", "-u"], cwd=project_root)
        staged_changes = subprocess.run(
            ["git", "diff", "--cached", "--quiet"],
            cwd=project_root,
        ).returncode
        if staged_changes:
            git(["commit", "-m", "Apply repository formatting"], cwd=project_root)

        run([sys.executable, "build/format.py", "check"], cwd=project_root)
        git(["merge-base", "--is-ancestor", format_commit, "HEAD"], cwd=project_root)

        if arguments.push:
            git(["push"], cwd=project_root)
        else:
            print("Migration verified locally. Review the diff, then run: git push")
        return 0
    except (KeyError, json.JSONDecodeError, OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
