<#
.SYNOPSIS
Builds the LPC1768 firmware with native CMake support.

.DESCRIPTION
Uses gcc.ps1 to acquire and select the ARM GCC toolchain, prefers Ninja as the
CMake generator, and falls back to Unix Makefiles. The default build is Release
with AXIS=5 and PAXIS=3.

.PARAMETER GccVersion
ARM GCC version managed by gcc.ps1. Defaults to 14.2.

.PARAMETER Clean
Cleans the selected CMake build tree before building.

.PARAMETER OutputPath
Copies firmware.bin to an existing directory or to a file in an existing directory.

.PARAMETER Debug
Builds the Debug profile with MRI support.

.PARAMETER Release
Builds the Release profile. This is the default.

.PARAMETER Slow
Uses one build job so diagnostics do not interleave.

.PARAMETER CMakeExtraArgs
Trailing NAME=value or -DNAME=value CMake cache settings.

.EXAMPLE
./build/build-cmake.ps1 -Clean

.EXAMPLE
./build/build-cmake.ps1 -Debug VERSION=my-debug-build
#>
$ErrorActionPreference = 'Stop'
$GccVersion = '14.2'
$Clean = $false
$OutputPath = ''
$Debug = $false
$Release = $false
$Slow = $false
$Help = $false
$CMakeExtraArgs = New-Object System.Collections.Generic.List[string]

for ($Index = 0; $Index -lt $args.Count; $Index++) {
    $Argument = $args[$Index]
    if ($Argument -in @('-GccVersion', '--gcc')) {
        if (++$Index -ge $args.Count) { throw "$Argument requires a version." }
        $GccVersion = $args[$Index]
    }
    elseif ($Argument -in @('-OutputPath', '--output')) {
        if (++$Index -ge $args.Count) { throw "$Argument requires a path." }
        $OutputPath = $args[$Index]
    }
    elseif ($Argument -in @('-Clean', '--clean')) { $Clean = $true }
    elseif ($Argument -in @('-Debug', '--debug')) { $Debug = $true }
    elseif ($Argument -in @('-Release', '--release')) { $Release = $true }
    elseif ($Argument -in @('-Slow', '--slow')) { $Slow = $true }
    elseif ($Argument -in @('-Help', '--help', '-h')) { $Help = $true }
    elseif ($Argument -match '^-D[A-Za-z_][A-Za-z0-9_]*=.*$' -or
            $Argument -match '^[A-Za-z_][A-Za-z0-9_]*=.*$') {
        $CMakeExtraArgs.Add($Argument)
    }
    else {
        throw "Unknown option or malformed cache setting '$Argument'."
    }
}

if ($GccVersion -notin @('4.8', '14.2')) {
    throw "Unsupported GCC version '$GccVersion'. Choose 4.8 or 14.2."
}

$ScriptDir = Split-Path -Parent $PSCommandPath
$ProjectRoot = (Resolve-Path (Join-Path $ScriptDir '..')).Path
$IsWindowsHost = [System.Environment]::OSVersion.Platform -eq [System.PlatformID]::Win32NT
$GccScriptPath = Join-Path $ScriptDir $(if ($IsWindowsHost) { 'gcc.ps1' } else { 'gcc.sh' })
$OriginalLocation = Get-Location
$OriginalPath = $env:PATH

function Invoke-CheckedCommand {
    param(
        [string]$Command,
        [string[]]$Arguments
    )

    Write-Host "Running: $Command $($Arguments -join ' ')" -ForegroundColor Green
    & $Command @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed with exit code $LASTEXITCODE`: $Command"
    }
}

if ($Help) {
    @"
Usage: ./build/build-cmake.ps1 [options] [NAME=value | -DNAME=value ...]

Options:
  -GccVersion <version>  Select GCC 4.8 or 14.2 (default: 14.2).
  -Clean                 Clean the selected build tree before building.
  -OutputPath <path>     Copy firmware.bin to a directory or file path.
  -Debug                 Build Debug firmware with MRI support.
  -Release               Build Release firmware (default).
  -Slow                  Build with one parallel job.
  -Help                  Show this help and exit.
"@
    exit 0
}
if ($Debug -and $Release) {
    Write-Error 'Choose either -Debug or -Release, not both.'
    exit 1
}

$BuildType = if ($Debug) { 'Debug' } else { 'Release' }
$BuildDir = Join-Path $ProjectRoot "build/cmake/gcc-$GccVersion/$BuildType"
$Artifact = Join-Path $BuildDir 'LPC1768/firmware.bin'

try {
    if ($IsWindowsHost) {
        $EnvCommand = & $GccScriptPath -GccVersion $GccVersion -Env
    }
    else {
        $EnvCommand = & $GccScriptPath --gcc $GccVersion --env
    }
    if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace(($EnvCommand -join ''))) {
        throw "Failed to prepare GCC $GccVersion."
    }
    if ($IsWindowsHost) {
        Invoke-Expression ($EnvCommand -join [Environment]::NewLine)
    }
    else {
        $EnvText = ($EnvCommand -join [Environment]::NewLine).Trim()
        if ($EnvText -notmatch '^export PATH="(.+):\$PATH"$') {
            throw "Unexpected environment output from $GccScriptPath."
        }
        $env:PATH = "$($Matches[1])$([System.IO.Path]::PathSeparator)$env:PATH"
    }

    $GeneratorArgs = @()
    if (-not (Test-Path (Join-Path $BuildDir 'CMakeCache.txt') -PathType Leaf)) {
        if (Get-Command ninja -ErrorAction SilentlyContinue) {
            $GeneratorArgs = @('-G', 'Ninja')
            Write-Host 'Using Ninja generator.' -ForegroundColor Cyan
        }
        elseif (Get-Command make -ErrorAction SilentlyContinue) {
            $MakeCommand = (Get-Command make).Source
            if ($IsWindowsHost) {
                $GeneratorArgs = @('-G', 'MinGW Makefiles', "-DCMAKE_MAKE_PROGRAM=$MakeCommand")
                Write-Host 'Ninja not found; using MinGW Makefiles generator.' -ForegroundColor Cyan
            }
            else {
                $GeneratorArgs = @('-G', 'Unix Makefiles')
                Write-Host 'Ninja not found; using Unix Makefiles generator.' -ForegroundColor Cyan
            }
        }
        else {
            throw 'Neither ninja nor make is available for CMake.'
        }
    }

    $TranslatedArgs = New-Object System.Collections.Generic.List[string]
    foreach ($Argument in $CMakeExtraArgs) {
        if ($Argument -match '^-D[A-Za-z_][A-Za-z0-9_]*=.*$') {
            $TranslatedArgs.Add($Argument)
        }
        elseif ($Argument -match '^[A-Za-z_][A-Za-z0-9_]*=.*$') {
            $TranslatedArgs.Add("-D$Argument")
        }
        else {
            throw "Expected NAME=value or -DNAME=value, got '$Argument'."
        }
    }

    $ConfigureArgs = @(
        '-S', $ProjectRoot,
        '-B', $BuildDir
    ) + $GeneratorArgs + @(
        "-DCMAKE_TOOLCHAIN_FILE=$(Join-Path $ProjectRoot 'cmake/arm-none-eabi-toolchain.cmake')",
        "-DCMAKE_BUILD_TYPE=$BuildType",
        '-DAXIS=5',
        '-DPAXIS=3'
    ) + $TranslatedArgs.ToArray()

    Invoke-CheckedCommand -Command 'cmake' -Arguments $ConfigureArgs
    if ($Clean) {
        Invoke-CheckedCommand -Command 'cmake' -Arguments @('--build', $BuildDir, '--target', 'clean')
    }

    $JobCount = if ($Slow) { 1 } else { [Environment]::ProcessorCount }
    Invoke-CheckedCommand -Command 'cmake' -Arguments @('--build', $BuildDir, '--parallel', $JobCount)

    if (-not (Test-Path $Artifact -PathType Leaf)) {
        throw "Build artifact not found at $Artifact."
    }

    if (-not [string]::IsNullOrWhiteSpace($OutputPath)) {
        $ResolvedOutputPath = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($OutputPath)
        if (Test-Path $ResolvedOutputPath -PathType Container) {
            $Destination = Join-Path $ResolvedOutputPath 'firmware.bin'
            $DestinationDir = $ResolvedOutputPath
        }
        else {
            $Destination = $ResolvedOutputPath
            $DestinationDir = Split-Path -Parent $Destination
        }
        if (-not (Test-Path $DestinationDir -PathType Container)) {
            throw "Destination directory '$DestinationDir' does not exist."
        }
        Copy-Item -Path $Artifact -Destination $Destination -Force
        Write-Host "Copied firmware.bin to $Destination." -ForegroundColor Cyan
    }

    Write-Host "CMake build finished: $Artifact" -ForegroundColor Green
}
catch {
    Write-Error $_.Exception.Message
    exit 1
}
finally {
    $env:PATH = $OriginalPath
    Set-Location $OriginalLocation
}
