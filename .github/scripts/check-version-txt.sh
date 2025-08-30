#!/usr/bin/env bash
set -euo pipefail

# Simple check to make sure version.txt has some kind of edit history for the
# current PR.

# Extract the PR base branch name from the event payload
BASE_REF=$(jq -r .pull_request.base.ref < "$GITHUB_EVENT_PATH")

if [[ -z "$BASE_REF" ]]; then
  echo "Could not determine base ref from GITHUB_EVENT_PATH. Skipping check."
  # Or exit 1 depending on desired behavior when ref is missing
  exit 0 
fi

DIFF_RANGE="origin/$BASE_REF...HEAD"

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

version.txt must be updated with a brief summary of your changes.

Please add a summary into version.txt before merging.

(Checked diff range: $DIFF_RANGE)
EOF

exit 1