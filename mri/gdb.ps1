#!/usr/bin/env pwsh
<##
    mri/gdb.ps1
    Helper script to launch arm‑none‑eabi‑gdb against a Carvera running MRI on Windows.
    
    Behaviour:
      1. Locate a suitable arm‑none‑eabi‑gdb (prefers newest version).
      2. Determine a serial port if none supplied.
      3. Select a sensible default baud rate.
      4. Invoke GDB with init.gdb and set $PORT within GDB without disturbing quoting.
##>

[CmdletBinding(PositionalBinding=$false)]
Param(
    [Parameter(Position=0)]
    [string]$Port,

    [Parameter(Position=1)]
    [int]$Baud = 115200
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
$ScriptDir   = $PSScriptRoot
$ProjectRoot = Split-Path $ScriptDir -Parent

# ---------------------------------------------------------------------------
# Locate a suitable arm‑none‑eabi‑gdb
# ---------------------------------------------------------------------------
$candidates = @()

# 1) Already in PATH?
try {
    $cmd = Get-Command arm-none-eabi-gdb.exe -ErrorAction SilentlyContinue
    if ($cmd) { $candidates += $cmd.Path }
} catch {}

# 2) Search inside the project tree for */bin/arm-none-eabi-gdb.exe
Get-ChildItem -Path $ProjectRoot -Filter arm-none-eabi-gdb.exe -Recurse -ErrorAction SilentlyContinue |
    ForEach-Object { $candidates += $_.FullName }

if (-not $candidates) {
    Write-Error "arm-none-eabi-gdb not found in PATH or project tree."
    exit 1
}

function Get-Version {
    param([string]$Exe)
    try {
        $out = & $Exe -v 2>&1 | Select-Object -First 1
    } catch {
        return [version]'0.0.0'
    }
    $match = [regex]::Match($out, '\d+(\.\d+)+')
    if ($match.Success) { return [version]$match.Value }
    return [version]'0.0.0'
}

$Gdb = $candidates | Sort-Object @{ Expression = { Get-Version $_ } ; Descending = $true } | Select-Object -First 1

if (-not $Gdb) {
    Write-Error "Failed to select a usable arm-none-eabi-gdb."
    exit 1
}

# ---------------------------------------------------------------------------
# Verify ELF binary exists
# ---------------------------------------------------------------------------
$ElfFile = Join-Path $ProjectRoot 'LPC1768\main.elf'
if (-not (Test-Path $ElfFile)) {
    Write-Error "Firmware ELF binary not found at $ElfFile. Build the firmware first."
    exit 1
}

# ---------------------------------------------------------------------------
# Select serial port (if not provided)
# ---------------------------------------------------------------------------
if (-not $Port) {
    # Try to detect a likely serial port. Prefer USB serial devices.
    try {
        $serial = Get-CimInstance Win32_SerialPort -ErrorAction SilentlyContinue |
            Sort-Object Name |
            Select-Object -First 1
        if ($serial) { $Port = $serial.DeviceID }
    } catch {}
}

if (-not $Port) {
    Write-Error "Unable to determine a serial port automatically. Please provide one."
    Write-Host "Usage: .\\gdb.ps1 <serial_port> [baud]"
    exit 1
}

# ---------------------------------------------------------------------------
# Launch GDB
# ---------------------------------------------------------------------------
$initGdb = Join-Path $ScriptDir 'init.gdb'
& $Gdb -b $Baud -x $initGdb -ex "target remote $Port" $ElfFile
exit $LASTEXITCODE 