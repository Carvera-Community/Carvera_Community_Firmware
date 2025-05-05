<#
.SYNOPSIS
Downloads the specified ARM GCC toolchain (if needed) and optionally runs a
command with the toolchain's bin directory added to the PATH.

.DESCRIPTION
This script manages ARM GCC toolchain versions for the project on Windows.
It checks if a specified version (defaulting to 14.2) is present. If not,
it downloads the official toolchain archive, verifies its integrity using
MD5 or SHA256, and unpacks it into the project structure.

It can then either:
- Execute a provided command (and arguments) with the selected toolchain's
  'bin' directory prepended to the PATH environment variable for that process.
- Output a PowerShell command string (using --env) that the calling shell
  can use (via Invoke-Expression) to modify its own PATH.

.PARAMETER GccVersion
Specifies the GCC version to use (e.g., '4.8', '14.2').
Defaults to '14.2'. Supported versions: 4.8, 14.2.

.PARAMETER Env
If specified, the script outputs a PowerShell command string to update the
calling shell's PATH, instead of executing a command.

.PARAMETER Help
Displays this help message and exits.

.PARAMETER CommandToRun
Any remaining arguments after options are treated as the command and its
arguments to be executed with the selected GCC toolchain in the PATH.

.EXAMPLE
# Build using default GCC (14.2)
./build/gcc.ps1 make CNC=1 AXIS=5 PAXIS=3

.EXAMPLE
# Build using GCC 4.8 after running 'make clean'
./build/gcc.ps1 --gcc 4.8 make clean

.EXAMPLE
# Add default GCC (14.2) to the current shell's PATH
Invoke-Expression (& ./build/gcc.ps1 --env)

.EXAMPLE
# Add GCC 4.8 to the current shell's PATH
Invoke-Expression (& ./build/gcc.ps1 --gcc 4.8 --env)

.NOTES
Requires PowerShell 5.1 or later for Expand-Archive and Get-FileHash.
The script assumes it is located in the 'build' directory of the project.
#>
[CmdletBinding(SupportsShouldProcess = $true)]
param(
    # --- Standard Named Parameters First ---
    [Parameter(Mandatory = $false)]
    [ValidateSet('4.8', '14.2')]
    [string]$GccVersion,

    [Parameter(Mandatory = $false)]
    [switch]$Env,

    [Parameter(Mandatory = $false)]
    [switch]$Help,

    # --- Capture Remaining Arguments Last ---
    [Parameter(Mandatory = $false, ValueFromRemainingArguments = $true, Position = 1)]
    [string[]]$CommandToRun
)

# --- Script Setup ---
$ErrorActionPreference = "Stop"
# $ProgressPreference = 'SilentlyContinue' # Suppress Invoke-WebRequest progress bar -- REMOVED to show progress

# Get the directory where the script resides and the project root
$ScriptDir = Split-Path -Parent $PSCommandPath # Use PSCommandPath for reliability
$ProjectRoot = (Resolve-Path (Join-Path $ScriptDir "..")).Path
$OriginalPwd = $PWD.Path # Store original working directory

# --- Configuration ---
$DefaultGccVersion = '14.2'
$GccConfigurations = @{
    '4.8'  = @{
        DirName     = 'gcc-arm-none-eabi-4.8'
        Url         = 'https://launchpad.net/gcc-arm-embedded/4.8/4.8-2014-q1-update/+download/gcc-arm-none-eabi-4_8-2014q1-20140314-win32.zip'
        Hash        = '09c19b3248863074f5498a88f31bee16'
        HashAlgo    = 'MD5'
        ArchiveName = 'gcc-arm-none-eabi-4_8-2014q1-20140314-win32.zip'
    }
    '14.2' = @{
        DirName     = 'gcc-arm-none-eabi-14.2'
        Url         = 'https://developer.arm.com/-/media/Files/downloads/gnu/14.2.rel1/binrel/arm-gnu-toolchain-14.2.rel1-mingw-w64-x86_64-arm-none-eabi.zip'
        Hash        = 'f074615953f76036e9a51b87f6577fdb4ed8e77d3322a6f68214e92e7859888f' # SHA256 hash for the mingw zip
        HashAlgo    = 'SHA256'
        ArchiveName = 'arm-gnu-toolchain-14.2.rel1-mingw-w64-x86_64-arm-none-eabi.zip'
    }
}

# Configuration for additional tools to check/download
$ToolConfigurations = @{
    'make' = @{
        DirName     = 'make-4.4.1' # Target directory name
        Url         = 'https://sourceforge.net/projects/ezwinports/files/make-4.4.1-without-guile-w32-bin.zip/download'
        Hash        = 'fb66a02b530f7466f6222ce53c0b602c5288e601547a034e4156a512dd895ee7'
        HashAlgo    = 'SHA256'
        ArchiveName = 'make-4.4.1-without-guile-w32-bin.zip'
    }
    # Add other tools here if needed in the future
}

# --- Helper Functions ---

function Show-Help {
    # Get the comment-based help content and display it using the script's path
    Get-Help $PSCommandPath
}

function Test-FileHash {
    param(
        [Parameter(Mandatory = $true)][string]$FilePath,
        [Parameter(Mandatory = $true)][string]$ExpectedHash,
        [Parameter(Mandatory = $true)][string]$Algorithm # MD5 or SHA256
    )
    Write-Host "Verifying $Algorithm hash for $(Split-Path $FilePath -Leaf)..."
    try {
        $calculatedHash = (Get-FileHash -Algorithm $Algorithm $FilePath).Hash
        if ($calculatedHash -eq $ExpectedHash) {
            Write-Host "Hash verified successfully."
            return $true
        }
        else {
            Write-Error "Hash mismatch for $(Split-Path $FilePath -Leaf)! Expected: $ExpectedHash, Calculated: $calculatedHash"
            return $false
        }
    }
    catch {
        Write-Error "Failed to calculate $Algorithm hash for $FilePath. Error: $($_.Exception.Message)"
        return $false
    }
}

# --- Consolidated Tool Installation Function ---
function Install-Tool {
    param(
        [Parameter(Mandatory = $true)][string]$ToolName, # Informational name, e.g., 'GCC 14.2', 'Make 4.4.1'
        [Parameter(Mandatory = $true)][hashtable]$Config, # The specific config block for the tool
        [Parameter(Mandatory = $true)][string]$ProjectRoot,
        [Parameter(Mandatory = $false)][string]$ToolchainBaseDir # Optional override (TOOLCHAIN_DIR)
    )

    # Validate mandatory config fields
    $requiredKeys = @('DirName', 'Url', 'Hash', 'HashAlgo', 'ArchiveName')
    foreach ($key in $requiredKeys) {
        if (-not $Config.ContainsKey($key) -or [string]::IsNullOrEmpty($Config[$key])) {
            throw "Configuration for '$ToolName' is missing or has empty value for required key: $key"
        }
    }

    # Determine base installation path
    $basePath = $ToolchainBaseDir
    if (-not $basePath) {
        $basePath = $ProjectRoot
    }
    $installPath = Join-Path $basePath $Config.DirName
    $toolBinPath = Join-Path $installPath "bin"

    # Calculate relative path for messages
    $separator = [System.IO.Path]::DirectorySeparatorChar
    $displayPath = ""
    if ($basePath -eq $ProjectRoot) {
        $displayPath = $installPath -replace [regex]::Escape($ProjectRoot + $separator), ("." + $separator)
    }
    else {
        $displayPath = $installPath # Show full path if TOOLCHAIN_DIR is used
    }

    # Check if tool bin directory already exists
    Write-Host "Checking for $ToolName at $displayPath..."
    if (Test-Path $toolBinPath -PathType Container) {
        Write-Host "Found $ToolName toolchain at $displayPath"
        # Optional: Add deeper verification here if needed (e.g., check specific file presence/version/hash)
        return $toolBinPath
    }

    Write-Host "$ToolName not detected at $displayPath."
    Write-Host "Downloading and unpacking $ToolName..."

    # Use config values directly
    $url = $Config.Url
    $hash = $Config.Hash
    $hashAlgo = $Config.HashAlgo
    $archiveName = $Config.ArchiveName

    # --- Download and Unpack Logic ---
    $tempDir = [System.IO.Path]::GetTempPath()
    if ([string]::IsNullOrEmpty($tempDir) -or -not (Test-Path $tempDir -PathType Container)) {
        throw "Could not determine or access the system temporary path: '$tempDir'."
    }
    $tempFilePath = Join-Path $tempDir $archiveName
    # Use a more unique temp extraction dir name
    $tempExtractDir = Join-Path $tempDir "$($Config.DirName)_$($hash.Substring(0,8))_extract"

    try {
        Write-Host "Downloading $archiveName from $url using curl.exe..."

        # Verify curl.exe is available
        $curlCmd = Get-Command curl.exe -ErrorAction SilentlyContinue
        if ($null -eq $curlCmd) {
            throw "curl.exe not found in PATH. It is required for downloading tools."
        }

        # Use curl.exe to download, following redirects (-L), failing on server errors (--fail), 
        # showing errors (-S) but allowing default progress meter (removed -s), and outputting to the temp file (-o)
        # Use quotes around paths and URL for safety
        & curl.exe -L -S --fail -o "$tempFilePath" "$url"
        $curlExitCode = $LASTEXITCODE

        # Check curl's exit code
        if ($curlExitCode -ne 0) {
            throw "curl.exe failed to download '$url' (Exit code: $curlExitCode). Check the URL and network connection."
        }

        # Verify downloaded file exists (curl might create an empty file on some errors even with --fail)
        if (-not (Test-Path $tempFilePath) -or (Get-Item $tempFilePath).Length -eq 0) {
            throw "Temporary file '$tempFilePath' not found or is empty after curl download attempt."
        }

        if (-not (Test-FileHash -FilePath $tempFilePath -ExpectedHash $hash -Algorithm $hashAlgo)) {
            throw "Hash verification failed for $archiveName."
        }

        Write-Host "Unpacking $archiveName to temporary location..."
        # Ensure target installation directory exists and is empty before final move
        if (Test-Path $installPath) {
            Write-Warning "Target directory '$installPath' already exists. Removing before extraction."
            Remove-Item $installPath -Recurse -Force -ErrorAction Stop # Stop if removal fails
        }
        # We create the final dir later, *after* extraction, before the move

        # Ensure clean temp extraction dir
        if (Test-Path $tempExtractDir) { Remove-Item $tempExtractDir -Recurse -Force }
        New-Item -ItemType Directory -Path $tempExtractDir -Force | Out-Null

        # Unpack to temporary directory
        Expand-Archive -Path $tempFilePath -DestinationPath $tempExtractDir -Force

        # Determine source directory within the extracted content (assuming bin is always at the root)
        $sourceDirToMove = $null
        $binPathInExtractRoot = Join-Path $tempExtractDir 'bin'

        if (Test-Path $binPathInExtractRoot -PathType Container) {
            Write-Verbose "Found 'bin' directory directly in archive root."
            $sourceDirToMove = $tempExtractDir # Source is the root of extraction
        }
        else {
            # Updated error message
            $subItems = Get-ChildItem -Path $tempExtractDir # Get items for error message context
            $existingNames = if ($null -ne $subItems) { ($subItems | Select-Object -ExpandProperty Name) -join ', ' } else { 'none' }
            throw "Archive '$archiveName' structure unclear. Expected 'bin' directory at the root of the unpacked archive. Contents found: $existingNames"
        }

        # This check might be redundant now, but keep as safety
        if ([string]::IsNullOrEmpty($sourceDirToMove)) {
            # This case should theoretically be caught by the logic above, but check just in case.
            throw "Failed to determine source directory path from extracted archive '$archiveName'."
        }

        # Ensure final target directory exists before moving
        if (-not (Test-Path $installPath)) {
            New-Item -ItemType Directory -Path $installPath -Force | Out-Null
        }

        # Move contents to the final target directory
        Write-Host "Installing $ToolName to $displayPath..."
        # Use -ErrorAction Stop to ensure move failures halt the script
        Move-Item -Path (Join-Path $sourceDirToMove '*') -Destination $installPath -Force -ErrorAction Stop

        # Final verification
        if (-not (Test-Path $toolBinPath -PathType Container)) {
            throw "Tool bin directory '$toolBinPath' still not found after download and installation attempt."
        }
        Write-Host "Successfully installed $ToolName to $displayPath."

        # Rename files in bin directories to add .exe (User-provided block)
        Get-ChildItem -Path $installPath -Recurse -Directory -Filter 'bin' | 
        ForEach-Object {
            Get-ChildItem $_.FullName -File |
            Where-Object { $_.Extension -ne '.exe' } | 
            Rename-Item -NewName { $_.Name + '.exe' } -ErrorAction SilentlyContinue
        }

        return $toolBinPath

    }
    catch {
        $errMsg = "Operation failed installing $ToolName (Install-Tool): $($_.Exception.Message)"
        if ($_.InvocationInfo) {
            $scriptName = $_.InvocationInfo.ScriptName
            $lineNum = $_.InvocationInfo.ScriptLineNumber
            $errMsg += (' ({0}:{1})' -f $scriptName, $lineNum)
        }
        Write-Error $errMsg
        # Clean up target dir on failure if it exists and might be incomplete
        if (Test-Path $installPath) {
            Write-Warning "Removing potentially incomplete directory due to error: $installPath"
            Remove-Item $installPath -Recurse -Force -ErrorAction SilentlyContinue
        }
        throw # Re-throw
    }
    finally {
        # Clean up temp files/dirs
        if (Test-Path $tempFilePath) { Remove-Item $tempFilePath -Force -ErrorAction SilentlyContinue }
        if (Test-Path $tempExtractDir) { Remove-Item $tempExtractDir -Recurse -Force -ErrorAction SilentlyContinue }
    }
}

# --- Main Execution ---

if ($Help) {
    Show-Help
    exit 0
}

# Set default GCC version if not provided by the user
$requestedGccVersion = $DefaultGccVersion # Start with default
if ($PSBoundParameters.ContainsKey('GccVersion')) {
    # Validate user-provided version against available configurations
    if (-not $GccConfigurations.ContainsKey($GccVersion)) {
        Write-Error "Unsupported GCC version requested: $GccVersion. Supported: $($GccConfigurations.Keys -join ', ')"
        exit 1
    }
    $requestedGccVersion = $GccVersion # Use user-provided value
}

$gccBinPath = $null
$makeBinPath = $null

try {
    # Get configurations
    $gccConfig = $GccConfigurations[$requestedGccVersion]
    $makeConfig = $ToolConfigurations['make']
    if (-not $gccConfig) { throw "Internal error: Could not find configuration for GCC version $requestedGccVersion." }
    if (-not $makeConfig) { throw "Internal error: Could not find configuration for tool 'make'." }

    # Ensure GCC is present and get its bin path
    $gccToolName = "GCC $requestedGccVersion"
    $gccBinPath = Install-Tool -ToolName $gccToolName -Config $gccConfig -ProjectRoot $ProjectRoot -ToolchainBaseDir $env:TOOLCHAIN_DIR

    # Ensure 'make' is present and get its bin path
    $makeToolName = "Make $($makeConfig.DirName)" # Use DirName for version info
    $makeBinPath = Install-Tool -ToolName $makeToolName -Config $makeConfig -ProjectRoot $ProjectRoot -ToolchainBaseDir $env:TOOLCHAIN_DIR

    # Validate paths returned
    if (-not $gccBinPath -or -not (Test-Path $gccBinPath -PathType Container)) {
        throw "Failed to determine or validate GCC bin path ('$gccBinPath') for version $requestedGccVersion after Install-Tool call."
    }
    if (-not $makeBinPath -or -not (Test-Path $makeBinPath -PathType Container)) {
        throw "Failed to determine or validate 'make' bin path ('$makeBinPath') after Install-Tool call."
    }

}
catch {
    # Print more details from the caught exception
    Write-Error "Failed during tool setup. Error: $($_.Exception.Message)"
    exit 1
}

# Output environment modification command if requested
if ($Env) {
    # Output the command to prepend the Make bin path and GCC bin path to the PATH
    $pathSeparator = [System.IO.Path]::PathSeparator
    # Prepend Make first, then GCC
    $newPath = "{0}{1}{2}{3}{4}" -f $makeBinPath, $pathSeparator, $gccBinPath, $pathSeparator, $env:PATH
    Write-Output ('$env:PATH="{0}"' -f $newPath)
    exit 0
}

# Check if a command was provided
if ($CommandToRun.Length -eq 0) {
    Write-Error "No command provided to execute."
    Show-Help
    exit 1
}

# Execute the command with the modified PATH
Write-Host "Executing command with $makeToolName and $gccToolName in PATH: $($CommandToRun -join ' ')"
$originalPath = $env:PATH
$pathSeparator = [System.IO.Path]::PathSeparator
# Prepend Make first, then GCC
$env:PATH = "$makeBinPath$pathSeparator$gccBinPath$pathSeparator$originalPath"

$exitCode = 0

try {
    # Execute the command from the original directory
    # Check if $OriginalPwd exists before setting location
    if (Test-Path $OriginalPwd -PathType Container) {
        Set-Location $OriginalPwd
    }
    else {
        Write-Warning "Original working directory '$OriginalPwd' not found. Staying in current directory ('$($PWD.Path)')."
    }

    # Use the call operator '&' and pass arguments individually
    Write-Host "Running command: $($CommandToRun[0]) $($CommandToRun[1..$($CommandToRun.Length - 1)] -join ' ')"
    & $CommandToRun[0] $CommandToRun[1..($CommandToRun.Length - 1)]
    $exitCode = $LASTEXITCODE
}
catch {
    Write-Error "Command execution failed: $($_.Exception.Message)"
    $exitCode = 1 # Indicate failure
}
finally {
    # Restore original PATH
    $env:PATH = $originalPath
    # Return to original location if it existed and we changed it
    if ((Test-Path $OriginalPwd -PathType Container) -and ($PWD.Path -ne $OriginalPwd)) {
        Set-Location $OriginalPwd -ErrorAction SilentlyContinue
    }
}

exit $exitCode 