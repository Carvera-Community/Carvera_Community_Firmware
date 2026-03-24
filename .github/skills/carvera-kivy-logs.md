---
description: Locate and analyze Kivy logs from Carvera Controller application for debugging firmware behavior. Use this skill when user asks to check logs, debug G-code execution, or analyze firmware output.
---

# Carvera Kivy Logs Analysis Skill

## Log File Locations

### Primary Log Directory
```
C:\Users\Matth\.kivy\logs
```

### Filename Pattern
```
kivy_YY-MM-DD_N.txt
```
- **YY-MM-DD**: Date in 2-digit year, month, day format (e.g., `26-03-13` for March 13, 2026)
- **N**: Session number starting from 0 (increments each time app starts)

### Examples
- `kivy_26-03-13_0.txt` - First session on March 13, 2026
- `kivy_26-03-13_1.txt` - Second session on March 13, 2026
- `kivy_26-03-14_0.txt` - First session on March 14, 2026

## Finding the Latest Log

### PowerShell Commands

**Get most recent log file:**
```powershell
Get-ChildItem C:\Users\Matth\.kivy\logs\kivy_*.txt | 
    Sort-Object LastWriteTime -Descending | 
    Select-Object -First 1 FullName, LastWriteTime
```

**Get latest log for today:**
```powershell
$today = Get-Date -Format "yy-MM-dd"
Get-ChildItem C:\Users\Matth\.kivy\logs\kivy_${today}_*.txt | 
    Sort-Object Name -Descending | 
    Select-Object -First 1
```

**Get latest log for specific date:**
```powershell
# Example: March 13, 2026
Get-ChildItem C:\Users\Matth\.kivy\logs\kivy_26-03-13_*.txt | 
    Sort-Object Name -Descending | 
    Select-Object -First 1
```

## Log Structure

### Session Lifecycle
1. **Startup**: Application initialization, GL context, resource loading
2. **Connection**: `MDI Recieved: Connected to machine!`
3. **Version Check**: `MDI Recieved: version = [firmware-version]`
4. **Config Download**: `MDI Sent: download /sd/config.txt`
5. **Runtime**: Continuous logging of commands and responses
6. **Shutdown**: `Base: Leaving application in progress...`

### Log Levels
- `[DEBUG  ]` - Detailed debugging information (verbose)
- `[INFO   ]` - Informational messages (standard operations)
- `[WARNING]` - Warning messages (non-critical issues)
- `[ERROR  ]` - Error messages (failures)

## Finding G-Code Test Runs

### Key Marker Line
Look for lines matching this pattern:
```
MDI Sent: download /sd/gcodes/[test_filename].cnc
```

**Example:**
```
[INFO   ] MDI Sent: download /sd/gcodes/TEST_D_word_simple_square_in_square.cnc
```

### PowerShell Search for Test Runs
```powershell
# Find all G-code downloads in latest log
$log = Get-ChildItem C:\Users\Matth\.kivy\logs\kivy_*.txt | Sort-Object LastWriteTime -Descending | Select-Object -First 1
Get-Content $log.FullName | Select-String "MDI Sent: download /sd/gcodes/"

# Get context around test run (20 lines before and after)
$log = Get-ChildItem C:\Users\Matth\.kivy\logs\kivy_*.txt | Sort-Object LastWriteTime -Descending | Select-Object -First 1
Get-Content $log.FullName | Select-String "MDI Sent: download /sd/gcodes/" -Context 20,100
```

### Test Run Debug Output Structure
After the download line, you'll see:
1. **Download confirmation**: `Info: download success: /sd/gcodes/[filename].cnc`
2. **File info**: `File size [bytes]`
3. **G-code execution**: Line-by-line processing with debug output
4. **Completion**: `Done ATC` or play completion message

## Firmware Debug Syntax

### MDI Communication
- **Sent to machine**: `MDI Sent: [command]`
- **Received from machine**: `MDI Recieved: [response]`

**Examples:**
```
[INFO   ] MDI Sent: version
[INFO   ] MDI Recieved: version = cutter-compensation-v2-acf2067
```

### Cutter Compensation Debug Output

#### Phase 2 Messages (Dual Buffer Implementation)
Prefix: `>>PHASE2:`

**Activation:**
```
>>PHASE2: G41/G42 called - clearing buffer (count was 0)
>>PHASE2: Compensation activated - type=LEFT (G41), radius=10.000
```

**Deactivation:**
```
>>PHASE2: G40 activated - FLUSHING mode (will output remaining X buffered commands)
```

#### Passthrough Messages
Lines that bypass compensation buffering:
```
>>PASSTHROUGH: [gcode] (comp=OFF, no buffering)
>>PASSTHROUGH: G90  (comp=OFF, no buffering)
```

#### Buffered Processing
Lines processed through compensation buffer:
```
>>PROCESS_BUFFERED: [g-number] '[gcode]'
>>PROCESS_BUFFERED: G1 'G1 X50.000 Y0.000'
```

#### Buffer State Messages
```
>>PHASE2: Buffering G1 X50.000 Y0.000 (count=1/3)
>>PHASE2: Computing offset for point [X,Y]
>>PHASE2: Outputting compensated: G1 X[new_x] Y[new_y]
```

## Analyzing Firmware Behavior

### Verification Checklist

**1. Firmware Version**
```powershell
# Check firmware version
$log = Get-ChildItem C:\Users\Matth\.kivy\logs\kivy_*.txt | Sort-Object LastWriteTime -Descending | Select-Object -First 1
Get-Content $log.FullName | Select-String "version = " | Select-Object -First 3
```

**Expected output:**
```
version = cutter-compensation-v2-acf2067  # Dual buffer implementation
version = cutter-compensation-v2-8001590  # Phase 2 previous version
```

**2. G41/G42 Activation**
Look for:
```
>>PHASE2: Compensation activated - type=[LEFT/RIGHT] (G41/G42), radius=[value]
```

**3. Buffer Operation**
Verify 3-move buffering:
```
>>PHASE2: Buffering G1... (count=1/3)
>>PHASE2: Buffering G1... (count=2/3)
>>PHASE2: Buffering G1... (count=3/3)
>>PHASE2: Computing offset...
```

**4. Compensated Output**
Check for properly offset coordinates:
```
>>PHASE2: Outputting compensated: G1 X[offset_x] Y[offset_y]
```

**5. G40 Deactivation**
Verify clean shutdown:
```
>>PHASE2: G40 activated - FLUSHING mode
```

### Common Debug Patterns

**Extract all Phase 2 debug messages:**
```powershell
$log = Get-ChildItem C:\Users\Matth\.kivy\logs\kivy_*.txt | Sort-Object LastWriteTime -Descending | Select-Object -First 1
Get-Content $log.FullName | Select-String ">>PHASE2:"
```

**Extract all compensation-related messages:**
```powershell
Get-Content $log.FullName | Select-String "(>>PHASE2:|>>PASSTHROUGH:|>>PROCESS_BUFFERED:)"
```

**Get last 100 lines (recent activity):**
```powershell
Get-Content $log.FullName | Select-Object -Last 100
```

**Search for errors:**
```powershell
Get-Content $log.FullName | Select-String "error|Error|ERROR" -Context 5
```

## Troubleshooting Guide

### Problem: Can't Find Latest Log
```powershell
# List all logs sorted by date
Get-ChildItem C:\Users\Matth\.kivy\logs\kivy_*.txt | 
    Sort-Object LastWriteTime -Descending | 
    Format-Table Name, LastWriteTime, Length -AutoSize
```

### Problem: Log Too Large to Read
```powershell
# Extract just the test run section
$log = Get-ChildItem C:\Users\Matth\.kivy\logs\kivy_*.txt | Sort-Object LastWriteTime -Descending | Select-Object -First 1

# Find line number of test start
$lineNum = (Get-Content $log.FullName | Select-String "MDI Sent: download /sd/gcodes/TEST_" | Select-Object -Last 1).LineNumber

# Read from that line to end
Get-Content $log.FullName | Select-Object -Skip ($lineNum - 20) | Select-Object -First 200
```

### Problem: Need to Compare Two Test Runs
```powershell
# Extract Phase 2 output from two logs
$log1 = "C:\Users\Matth\.kivy\logs\kivy_26-03-13_0.txt"
$log2 = "C:\Users\Matth\.kivy\logs\kivy_26-03-13_1.txt"

Write-Host "=== LOG 1 ===" -ForegroundColor Green
Get-Content $log1 | Select-String ">>PHASE2:" | Select-Object -Last 20

Write-Host "`n=== LOG 2 ===" -ForegroundColor Green
Get-Content $log2 | Select-String ">>PHASE2:" | Select-Object -Last 20
```

## Quick Reference Commands

### Most Common Operations

**View latest log live (tail -f equivalent):**
```powershell
$log = Get-ChildItem C:\Users\Matth\.kivy\logs\kivy_*.txt | Sort-Object LastWriteTime -Descending | Select-Object -First 1
Get-Content $log.FullName -Tail 50 -Wait
```

**Search for specific G-code test:**
```powershell
$log = Get-ChildItem C:\Users\Matth\.kivy\logs\kivy_*.txt | Sort-Object LastWriteTime -Descending | Select-Object -First 1
Get-Content $log.FullName | Select-String "TEST_D_word" -Context 0,50
```

**Extract firmware version and date:**
```powershell
$log = Get-ChildItem C:\Users\Matth\.kivy\logs\kivy_*.txt | Sort-Object LastWriteTime -Descending | Select-Object -First 1
Write-Host "Log: $($log.Name) - Modified: $($log.LastWriteTime)"
Get-Content $log.FullName | Select-String "version = " | Select-Object -First 1
```

## Integration with Firmware Development

### After Flashing New Firmware
1. Start Carvera Controller (creates new log session)
2. Note the session number increase
3. Run test G-code program
4. Check log for:
   - Firmware version matches expected build
   - Phase 2 debug output appears
   - Buffer operation is correct
   - Compensated coordinates are calculated

### Debug Workflow
1. **Identify log**: Find latest session
2. **Locate test**: Search for G-code download
3. **Extract output**: Get Phase 2 messages
4. **Verify behavior**: Check buffer, computation, output
5. **Compare**: Diff against previous firmware version

## Related Documentation
- Firmware build process: `.github/skills/carvera-build.md`
- Dual buffer design: `DUAL_BUFFER_REFACTOR_DESIGN.md`
- Cutter compensation implementation: `cutter_comp_documentation/CUTTER_COMPENSATION_IMPLEMENTATION.md`
