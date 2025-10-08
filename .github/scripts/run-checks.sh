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

# Check if this is a fork PR (where we might have limited API permissions)
IS_FORK_PR=false
CAN_CREATE_CHECK_RUNS=true

if [[ "${GITHUB_EVENT_NAME:-}" == "pull_request" ]]; then
  # Check if the head repo is different from the base repo
  if command -v jq >/dev/null 2>&1 && [[ -f "${GITHUB_EVENT_PATH:-}" ]]; then
    HEAD_REPO=$(jq -r '.pull_request.head.repo.full_name // empty' < "$GITHUB_EVENT_PATH" 2>/dev/null || echo "")
    BASE_REPO=$(jq -r '.pull_request.base.repo.full_name // empty' < "$GITHUB_EVENT_PATH" 2>/dev/null || echo "")
    if [[ -n "$HEAD_REPO" && -n "$BASE_REPO" && "$HEAD_REPO" != "$BASE_REPO" ]]; then
      IS_FORK_PR=true
      echo "ℹ️  Detected fork PR: $HEAD_REPO -> $BASE_REPO"
      echo "ℹ️  Check runs may not be created due to limited permissions, but checks will still run locally."
    fi
  fi
fi

# Test if we can create check runs (for fork PRs, this will likely fail)
if [[ "$IS_FORK_PR" == "true" ]]; then
  # For fork PRs, test API permissions by trying to create a test check run
  if ! gh api --method POST "/repos/$OWNER/$REPO/check-runs" \
    -f name="permission-test" \
    -f head_sha="$HEAD_SHA" \
    -f status=completed \
    -f conclusion=success \
    -f "output[title]=Permission Test" \
    -f "output[summary]=Testing API permissions" 2>/dev/null >/dev/null; then
    CAN_CREATE_CHECK_RUNS=false
    echo "ℹ️  Cannot create check runs due to fork PR permissions. Results will be logged locally only."
  else
    echo "ℹ️  API permissions available. Check runs will be created."
  fi
fi

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

    # Create success check run (if permissions allow)
    if [[ "$CAN_CREATE_CHECK_RUNS" == "true" ]]; then
      if ! gh api --method POST "/repos/$OWNER/$REPO/check-runs" \
        -f name="$name" \
        -f head_sha="$HEAD_SHA" \
        -f status=completed \
        -f conclusion="$conclusion" \
        -f "output[title]=$title" \
        -f "output[summary]=$summary" 2>/dev/null; then
        echo "   ⚠️  Failed to create check run (API error). Check passed locally."
        CAN_CREATE_CHECK_RUNS=false  # Disable future attempts
      fi
    else
      echo "   ℹ️  Check passed locally (check runs disabled due to permissions)."
    fi
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

    # Create failure check run (if permissions allow)
    if [[ "$CAN_CREATE_CHECK_RUNS" == "true" ]]; then
      if ! gh api --method POST "/repos/$OWNER/$REPO/check-runs" \
        -f name="$name" \
        -f head_sha="$HEAD_SHA" \
        -f status=completed \
        -f conclusion="$conclusion" \
        -f "output[title]=$title" \
        -f "output[summary]=$summary" 2>/dev/null; then
        echo "   ⚠️  Failed to create check run (API error). Check failed locally."
        CAN_CREATE_CHECK_RUNS=false  # Disable future attempts
      fi
    else
      echo "   ℹ️  Check failed locally (check runs disabled due to permissions)."
    fi
  fi
done

# --- Generate Job Summary --- 
summary_content="## Check Run Summary\n\n"

# Add context about check run creation status
if [[ "$IS_FORK_PR" == "true" ]]; then
  if [[ "$CAN_CREATE_CHECK_RUNS" == "true" ]]; then
    summary_content+="### Status ℹ️\n**Fork PR detected** - Check runs created successfully.\n\n"
  else
    summary_content+="### Status ℹ️\n**Fork PR detected** - Check runs could not be created due to limited permissions. Results logged locally only.\n\n"
  fi
fi

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
  if [[ "$IS_FORK_PR" == "true" && "$CAN_CREATE_CHECK_RUNS" == "false" ]]; then
    echo "ℹ️  Results logged locally due to fork PR permissions."
  fi
else
  echo "🎉 All checks passed."
  if [[ "$IS_FORK_PR" == "true" && "$CAN_CREATE_CHECK_RUNS" == "false" ]]; then
    echo "ℹ️  Results logged locally due to fork PR permissions."
  fi
fi

exit $FAILED
