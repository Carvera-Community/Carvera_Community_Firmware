#!/usr/bin/env bash
# .github/scripts/run-checks.sh
set -euo pipefail

# Enumerates the check-*.sh scripts and executes each one, reporting the status
# back as a github check.

ROOT="$(dirname "$0")"
OWNER="${GITHUB_REPOSITORY%/*}"
REPO="${GITHUB_REPOSITORY#*/}"
HEAD_SHA="$GITHUB_SHA"


FAILED=0

for script in "$ROOT"/check-*.sh; do
  name="$(basename "$script" .sh)"     # e.g. "check-version"
  echo "→ Running $name..."

  # Run the check script; capture all output
  if output=$( "$script" 2>&1 ); then
    # Success case
    echo "   ✓ $name passed."
    conclusion="success"
    # Extract first line for title, rest for summary
    title=$(echo "$output" | head -n 1)
    summary=$(echo "$output" | tail -n +2)
    # Default title if no output
    if [[ -z "$title" ]]; then
        title="✓ $name passed"
    fi
    # Ensure summary is not empty if there was only one line
    if [[ "$(echo "$output" | wc -l)" -le 1 ]]; then
        summary="Check passed without additional details."
    fi

    # Create success check run
    gh api --method POST "/repos/$OWNER/$REPO/check-runs" \
      -f name="$name" \
      -f head_sha="$HEAD_SHA" \
      -f status=completed \
      -f conclusion="$conclusion" \
      -f "output[title]=$title" \
      -f "output[summary]=$summary" > /dev/null # Suppress stdout
  else
    # Failure case
    exit_code=$?
    echo "   ✘ $name failed (exit code $exit_code). Creating check-run…"
    FAILED=1
    conclusion="failure"
    # Extract first line for title, rest for summary
    title=$(echo "$output" | head -n 1)
    summary=$(echo "$output" | tail -n +2)
    # Default title if no output
    if [[ -z "$title" ]]; then
        title="❌ $name failed"
    fi
    # Ensure summary is not empty if there was only one line
    if [[ "$(echo "$output" | wc -l)" -le 1 ]]; then
        summary="Check failed without additional details."
    fi

    # Create failure check run
    gh api --method POST "/repos/$OWNER/$REPO/check-runs" \
      -f name="$name" \
      -f head_sha="$HEAD_SHA" \
      -f status=completed \
      -f conclusion="$conclusion" \
      -f "output[title]=$title" \
      -f "output[summary]=$summary" > /dev/null # Suppress stdout
  fi
done

if [[ $FAILED -eq 1 ]]; then
  echo "🚨 One or more checks failed."
else
  echo "🎉 All checks passed."
fi

exit $FAILED
