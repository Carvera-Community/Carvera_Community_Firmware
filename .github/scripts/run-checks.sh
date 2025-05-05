#!/usr/bin/env bash
# .github/scripts/run-checks.sh
set -euo pipefail

# Enumerates the check-*.sh scripts and executes each one, reporting the status
# back as a github check.

ROOT="$(dirname "$0")"
OWNER="${GITHUB_REPOSITORY%/*}"
REPO="${GITHUB_REPOSITORY#*/}"
# Use the PR head SHA if provided (during pull_request events), otherwise use GITHUB_SHA
HEAD_SHA="${PR_HEAD_SHA:-$GITHUB_SHA}"


FAILED=0
passed_checks=""
failed_checks=""

for script in "$ROOT"/check-*.sh; do
  # Check if the file exists and is executable before trying to run it
  if [[ ! -f "$script" || ! -x "$script" ]]; then
    echo "Skipping non-executable or non-existent file: $script"
    continue
  fi

  name="$(basename "$script" .sh)"     # e.g. "check-version"
  echo "→ Running $name..."

  # Run the check script; capture all output
  if output=$( "$script" 2>&1 ); then
    # Success case
    echo "   ✓ $name passed."
    conclusion="success"
    passed_checks+="- ✅ $name\n"
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
      -f "output[summary]=$summary"
  else
    # Failure case
    exit_code=$?
    echo "   ✘ $name failed (exit code $exit_code). Creating check-run…"
    FAILED=1
    conclusion="failure"
    failed_checks+="- ❌ $name (exit code $exit_code)\n"
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
      -f "output[summary]=$summary"
  fi
done

# --- Generate Job Summary --- 
summary_content="## Check Run Summary\n\n"

if [[ -n "$failed_checks" ]]; then
  summary_content+="### Failed Checks 🚨\n$failed_checks\n"
fi

if [[ -n "$passed_checks" ]]; then
  summary_content+="### Passed Checks ✅\n$passed_checks\n"
fi

if [[ -z "$passed_checks" && -z "$failed_checks" ]]; then
  summary_content+="No checks were executed.\n"
elif [[ -z "$failed_checks" ]]; then
  summary_content+="**All checks passed!** 🎉\n"
fi

# Append to the GitHub Step Summary file
# Use printf for better handling of backslashes and newlines
printf "%b" "$summary_content" >> "$GITHUB_STEP_SUMMARY"

# --- Exit with appropriate code ---
if [[ $FAILED -eq 1 ]]; then
  echo "🚨 One or more checks failed."
else
  echo "🎉 All checks passed."
fi

exit $FAILED
