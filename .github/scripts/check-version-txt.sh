#!/usr/bin/env bash
set -euo pipefail

# Simple check to make sure version.txt has some kind of edit history for the
# current PR, but only if files in the src/ directory have changed.

# Extract the PR base branch name from the event payload
BASE_REF=$(jq -r .pull_request.base.ref < "$GITHUB_EVENT_PATH")

if [[ -z "$BASE_REF" ]]; then
  echo "Could not determine base ref from GITHUB_EVENT_PATH. Skipping check."
  exit 0 
fi

DIFF_RANGE="origin/$BASE_REF...HEAD"

# First check if any files in src/ directory have changed
if ! git diff --name-only "$DIFF_RANGE" | grep -q "^src/"; then
  echo "ℹ️  No files changed in src/ directory. Skipping version.txt check."
  exit 0
fi

echo "ℹ️  Files changed in src/ directory. Checking version.txt update..."

# List changed files compared to the base branch
# Use three-dot diff to see changes on HEAD since diverging from origin/$BASE_REF
if git diff --name-only "$DIFF_RANGE" | grep -qx version.txt; then
  # PASS: version.txt was updated
  echo "✅ version.txt was updated in this PR."
  echo "(Checked diff range: $DIFF_RANGE)"
  exit 0
fi

# FAIL: emit a Markdown summary on stdout
cat <<EOF
⚠️ Missing version.txt update

Since you've modified files in the src/ directory, version.txt must be updated with a brief summary of your changes.

Please add a summary into version.txt before merging.

(Checked diff range: $DIFF_RANGE)
EOF

exit 1