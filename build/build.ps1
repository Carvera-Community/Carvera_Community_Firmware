<#
.SYNOPSIS
Runs the firmware build process using the specified ARM GCC toolchain via gcc.ps1.

.DESCRIPTION
This script orchestrates the firmware build.
It determines the number of CPU cores for parallel make jobs.
It invokes ./build/gcc.ps1 to ensure the correct GCC toolchain is available
and in the PATH, then executes the 'make' command.

It passes standard arguments (AXIS=5 PAXIS=3 CNC=1) and any additional
key=value pairs or make targets provided directly to the make command.

.PARAMETER GccVersion
Specifies the GCC version to use (e.g., '4.8', '14.2').
Defaults to '14.2'. Passed directly to gcc.ps1.

.PARAMETER Clean
If specified, runs 'make clean' using the selected GCC toolchain before
starting the main build.

.PARAMETER OutputPath
Specifies the path where the final `main.bin` artifact should be copied.
If the path is an existing directory, the artifact will be copied into it as `main.bin`.
If the path points to a non-existent file in an existing directory, the artifact will be copied to that path.
The destination directory must exist.

.PARAMETER Slow
runs the build command single threaded to prevent issues with multiple threads
writing error messages over eachother

.PARAMETER Help
Displays this help message and exits.

.PARAMETER MakeExtraArgs
Any remaining arguments after options are treated as additional arguments
(like targets or variable assignments, e.g., VERBOSE=1)
to be passed to the make command.

.EXAMPLE
# Build with default GCC (14.2)
./build/build.ps1

.EXAMPLE
# Clean then build with default GCC
./build/build.ps1 --Clean

.EXAMPLE
# Clean then build with GCC 4.8, verbose output
./build/build.ps1 --GccVersion 4.8 --Clean VERBOSE=1

.EXAMPLE
# Build with default GCC and enable debug monitor
./build/build.ps1 ENABLE_DEBUG_MONITOR=1

.NOTES
Relies on ./build/gcc.ps1 being present in the same directory.
Requires PowerShell 5.1 or later.
#>
[CmdletBinding(SupportsShouldProcess=$true)]
param(
    # Capture make-specific args (targets, VAR=VAL)
    [Parameter(Mandatory=$false, ValueFromRemainingArguments=$true, Position=0)]
    [string[]]$MakeExtraArgs,

    # Optional parameters for build script itself
    [Parameter(Mandatory=$false)]
    [string]$GccVersion = '14.2',

    [Parameter(Mandatory=$false)]
    [switch]$Clean,

    [Parameter(Mandatory=$false)]
    [string]$OutputPath,

    [Parameter(Mandatory=$false)]
    [switch]$Help,

    [Parameter(Mandatory=$false)]
    [switch]$Slow
)

# --- Script Setup ---
$ErrorActionPreference = "Stop"

# Get the directory where the script resides and the project root
$ScriptDir = Split-Path -Parent $PSCommandPath
$ProjectRoot = (Resolve-Path (Join-Path $ScriptDir "..")).Path
$GccScriptPath = Join-Path $ScriptDir "gcc.ps1"

# --- Configuration ---
$DefaultGccVersion = '14.2' # Match the default in gcc.ps1
$BaseMakeArgs = "AXIS=5 PAXIS=3 CNC=1" # Standard args for this firmware

# --- Helper Functions ---

function Show-Help {
    Get-Help $PSCommandPath
}

# --- Main Execution ---

if ($Help) {
    Show-Help
    exit 0
}

# Determine GCC version to use
$requestedGccVersion = if ($PSBoundParameters.ContainsKey('GccVersion')) { $GccVersion } else { $DefaultGccVersion }

# Determine CPU count for parallelism - empty on Windows
if ($PSVersionTable.PSEdition -eq "Desktop" -or 
    ($PSVersionTable.PSVersion.Major -ge 6 -and $PSVersionTable.Platform -eq "Win32NT") -or
    [System.Environment]::OSVersion.Platform -eq "Win32NT") {
    # Running on Windows
    $cpuCount = ""
    # Check if running on Windows first before using $IsWindows
    $script:IsWindows = $true
    Write-Host "Running on Windows, processor count not set because it's buggy." -ForegroundColor Cyan
} else {
    # Running on non-Windows system
    $script:IsWindows = $false
    $cpuCount = [Environment]::ProcessorCount
    Write-Host "Using $cpuCount parallel jobs for make." -ForegroundColor Cyan
}
if ($Slow){
    $cpuCount = "1"
}
Write-Host "Using GCC version: $requestedGccVersion" -ForegroundColor Cyan
Write-Host "Using $cpuCount parallel jobs for make." -ForegroundColor Cyan
if ($Clean) {
    Write-Host "Will run 'make clean' first." -ForegroundColor Cyan
}

# Ensure we are in the project root, as 'make' expects this
Set-Location $ProjectRoot

$exitCode = 0

# --- Run Clean Step (if requested) ---
if ($Clean) {
    $cleanCommandArgs = @("make", "clean")
    Write-Host "Running: $($cleanCommandArgs -join ' ') (via gcc.ps1)..." -ForegroundColor Green
    try {
        & $GccScriptPath -GccVersion $requestedGccVersion -CommandToRun $cleanCommandArgs
        $exitCode = $LASTEXITCODE
        if ($exitCode -ne 0) {
            throw "'make clean' failed with exit code $exitCode."
        }
        Write-Host "'make clean' finished successfully." -ForegroundColor Green
    } catch {
        Write-Error "Failed during clean step: $($_.Exception.Message)"
        exit 1 # Exit immediately on clean failure
    }
}

# --- Run Build Step ---
# Construct the final make command arguments
$makeArgsList = New-Object System.Collections.Generic.List[string]
$makeArgsList.Add("make")
$makeArgsList.Add("-j$cpuCount")
$makeArgsList.AddRange($BaseMakeArgs.Split(' '))
if ($MakeExtraArgs.Length -gt 0) {
    $makeArgsList.AddRange($MakeExtraArgs)
}
$makeCommandArgs = $makeArgsList.ToArray()

# Construct arguments to pass to gcc.ps1
Write-Host "Running: $($makeCommandArgs -join ' ') (via gcc.ps1)..." -ForegroundColor Green
try {
    & $GccScriptPath -GccVersion $requestedGccVersion -CommandToRun $makeCommandArgs
    $exitCode = $LASTEXITCODE
    if ($exitCode -ne 0) {
        throw "Build failed with exit code $exitCode."
    }
    Write-Host "Build finished successfully." -ForegroundColor Green

    # --- Handle Output Copy --- 
    if ($exitCode -eq 0 -and $PSBoundParameters.ContainsKey('OutputPath') -and -not [string]::IsNullOrWhiteSpace($OutputPath)) {
        $sourceFile = Join-Path $ProjectRoot "LPC1768/main.bin"
        if (-not (Test-Path $sourceFile -PathType Leaf)) {
            Write-Error "Build artifact not found at $sourceFile"
            exit 1
        }

        # Resolve OutputPath to an absolute path
        $resolvedOutputPath = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($OutputPath)

        $destPath = ""
        $destDir = ""

        # Check if resolvedOutputPath is a directory
        if (Test-Path $resolvedOutputPath -PathType Container) {
            $destDir = $resolvedOutputPath
            $destPath = Join-Path $destDir "main.bin"
        } else {
            # Assume it includes the filename
            $destPath = $resolvedOutputPath
            $destDir = Split-Path -Parent $destPath
        }

        # Check if destination directory exists
        if (-not (Test-Path $destDir -PathType Container)) {
            Write-Error "Destination directory $destDir does not exist."
            exit 1
        }

        # Perform the copy
        Write-Host "Copying $sourceFile to $destPath" -ForegroundColor Cyan
        try {
            Copy-Item -Path $sourceFile -Destination $destPath -Force -ErrorAction Stop
            Write-Host "Successfully copied artifact to $destPath" -ForegroundColor Green
        } catch {
            Write-Error "Failed to copy build artifact to $destPath`: $($_.Exception.Message)"
            exit 1
        }
    }

} catch {
    Write-Error "Failed during build step: $($_.Exception.Message)"
    # Exit code already captured, just report error
}

# Exit with the final exit code from the build (or clean if build didn't run/succeeded)
exit $exitCode 