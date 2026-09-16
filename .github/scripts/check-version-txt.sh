#!/usr/bin/env bash
set -euo pipefail

# Simple check to make sure version.txt has some kind of edit history for the
# current PR, but only if files in the src/ directory have changed.

# Extract the PR base branch and repository from the event payload
BASE_REF=$(jq -r .pull_request.base.ref < "$GITHUB_EVENT_PATH")
BASE_REPO=$(jq -r .pull_request.base.repo.clone_url < "$GITHUB_EVENT_PATH")

if [[ -z "$BASE_REF" || "$BASE_REF" == "null" ]]; then
  echo "Could not determine base ref from GITHUB_EVENT_PATH. Skipping check."
  exit 0
fi

# The workflow checks out the PR head repository, so on a fork the base branch
# is usually absent from origin/. Fetch it from the base repository directly.
# Without this the diff below fails, the error is swallowed by the "no src
# changes" branch, and the check passes without having checked anything.
if ! git fetch --no-tags --quiet "$BASE_REPO" \
    "+refs/heads/$BASE_REF:refs/remotes/base/$BASE_REF"; then
  echo "❌ Could not fetch base branch $BASE_REF"
  echo ""
  echo "The version.txt check needs the base branch to diff against, and"
  echo "fetching it from $BASE_REPO failed."
  exit 1
fi

# Three-dot diff to see changes on HEAD since diverging from the base branch
DIFF_RANGE="refs/remotes/base/$BASE_REF...HEAD"
CHANGED=$(git diff --name-only "$DIFF_RANGE")

if ! grep -q "^src/" <<< "$CHANGED"; then
  echo "ℹ️  No files changed in src/ directory. Skipping version.txt check."
  exit 0
fi

echo "ℹ️  Files changed in src/ directory. Checking version.txt update..."

if grep -qx version.txt <<< "$CHANGED"; then
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
