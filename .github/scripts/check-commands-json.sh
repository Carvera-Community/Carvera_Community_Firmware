#!/usr/bin/env bash
set -euo pipefail

# Fail a PR when newly added G/M/console command IDs are missing from
# docs/commands.json. See check-commands-json.py for what is scanned.

ROOT="$(cd "$(dirname "$0")" && pwd)"
exec python3 "$ROOT/check-commands-json.py"
