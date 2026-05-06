#!/usr/bin/env bash
set -euo pipefail

# Sets the VERSION variable in the action for the different types of github
# events. Not to be confused with version.txt.

ref=${GITHUB_REF:-}
sha=${GITHUB_SHA:0:7}

if [[ $ref == refs/tags/* ]]; then
    version=${ref#refs/tags/}
elif [[ $ref == refs/heads/* ]]; then
    version=${ref#refs/heads/}-$sha
elif [[ $ref == refs/pull/*/merge ]]; then
    num=${ref#refs/pull/}
    num=${num%%/*}
    version=pr-$num-$sha
else
    version=${ref##*/}-$sha
fi

echo "VERSION=$version" >>"$GITHUB_OUTPUT"
