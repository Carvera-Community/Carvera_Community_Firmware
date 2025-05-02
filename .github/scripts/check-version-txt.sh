#!/usr/bin/env bash
set -euo pipefail

# Simple check to make sure version.txt has some kind of edit history for the
# current PR.

# Extract the PR base SHA from the event payload
BASE_SHA=$(jq -r .pull_request.base.sha < "$GITHUB_EVENT_PATH")

# list changed files between base…head
if git diff --name-only "$BASE_SHA" HEAD | grep -qx version.txt; then
  # PASS: version.txt was updated
  exit 0
fi

# FAIL: emit a Markdown summary on stdout
cat <<'EOF'
⚠️ Missing version.txt update

version.txt must be updated with a brief summary of your changes.

Please add a summary into version.txt before merging.
EOF

exit 1