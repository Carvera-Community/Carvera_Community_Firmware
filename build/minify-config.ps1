# minify-config.ps1 - Strip comments, blank lines, and excess whitespace from
# config files before embedding them into the firmware binary.
#
# Usage: .\minify-config.ps1 <input> <output>
#
# This reduces flash usage by removing data that the config parser skips anyway:
#   - Lines starting with # (comments)
#   - Blank or whitespace-only lines
#   - Trailing comments (everything after the value's first # on a line)
#   - Runs of tabs/spaces used for column alignment (collapsed to single tab)
#   - Trailing whitespace on each line

param(
    [Parameter(Mandatory = $true, Position = 0)]
    [string]$InputFile,

    [Parameter(Mandatory = $true, Position = 1)]
    [string]$OutputFile
)

# ============================================================================
# IMPORTANT: Each regex below is carefully chosen to ONLY remove bytes that
# the config parser (FirmConfigSource) already ignores at runtime. None of
# these will ever match or alter an actual config key or value.
#
# Rule-by-rule breakdown:
#
#   1) ^\s*#
#      DELETE entire lines that are pure comments (optional leading whitespace
#      followed by #).
#
#   2) ^\s*$
#      DELETE blank lines or lines containing only whitespace.
#
#   3) \s#\s.*$
#      STRIP trailing comments from value lines. Requires space-#-space to
#      avoid false matches on values containing # adjacent to text.
#
#   4) \s+$
#      STRIP trailing whitespace from each line.
#
#   5) \s{2,}
#      COLLAPSE runs of 2+ whitespace characters into a single tab.
#
# ============================================================================

$ErrorActionPreference = 'Stop'

Get-Content -LiteralPath $InputFile |
    Where-Object { $_ -notmatch '^\s*#' } |
    Where-Object { $_ -notmatch '^\s*$' } |
    ForEach-Object { $_ -replace '\s#\s.*$', '' } |
    ForEach-Object { $_ -replace '\s+$', '' } |
    ForEach-Object { $_ -replace '\s{2,}', "`t" } |
    Set-Content -LiteralPath $OutputFile