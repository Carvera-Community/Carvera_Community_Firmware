---
description: Build and deploy Carvera CNC firmware with proper naming conventions and post-build procedures. Use this skill when user asks to build, compile, or deploy firmware.
---

# Carvera Firmware Build Skill

## Build Commands

### PowerShell Build (Preferred Method)
```powershell
cd C:\Users\Matth\Desktop\Personal\equipment_and_tools\Carvera_CNC\Carvera_Community_Firmware
.\build\build.ps1 -Clean VERSION=2.1.0c TOOLSETTER=1 VERBOSE=1
```

### Parameterized PowerShell Build (When Version Changes)
```powershell
cd C:\Users\Matth\Desktop\Personal\equipment_and_tools\Carvera_CNC\Carvera_Community_Firmware
.\build\build.ps1 -Clean VERSION=<firmware_version> TOOLSETTER=1 VERBOSE=1
```

### Alternative: Make Build
```bash
cd C:\Users\Matth\Desktop\Personal\equipment_and_tools\Carvera_CNC\Carvera_Community_Firmware
.\BuildShell.cmd
make clean
make all AXIS=5 PAXIS=3 CNC=1
```

## Build Parameters
- **Target Platform:** LPC1768 ARM Cortex-M3
- **Compiler:** GCC ARM None EABI 14.2.1
- **Build Flags:** AXIS=5 PAXIS=3 CNC=1 (5-axis CNC configuration)
- **Expected Size:** ~446KB text + 708 bytes data + 17KB bss = ~464KB total

## Output Locations
- **Primary Output:** `LPC1768/main.bin`
- **Hex File:** `LPC1768/main.hex`
- **ELF File:** `LPC1768/main.elf`
- **Disassembly:** `LPC1768/main.disasm`

## Post-Build Procedure

### 1. Verify Build Success
Check for all of the following at end of output:
- "Build finished successfully."
- No linker or compiler errors (warnings are acceptable unless user asks to resolve all warnings)
- Output artifacts exist: `LPC1768/main.bin`, `LPC1768/main.hex`, `LPC1768/main.elf`

### 2. Copy Binary to FirmwareBin Folder with Descriptive Name
```powershell
# Pattern: firmware_[Feature]_[Date].bin
# Copy to FirmwareBin folder where all previous builds are stored
# Examples:
Copy-Item LPC1768\main.bin -Destination FirmwareBin\firmware_DualBuffer_20260313.bin
Copy-Item LPC1768\main.bin -Destination FirmwareBin\firmware_Exp_CutComp.bin
```

### 3. Deployment to Carvera
1. **Backup current firmware:**
   - Copy existing `firmware.bin` from SD card to safe location
   - Backup `config.txt` file as well

2. **Copy from FirmwareBin folder to SD card:**
   - Navigate to `FirmwareBin` folder
   - Select desired firmware binary (e.g., `firmware_DualBuffer_20260313.bin`)
   - Rename to `firmware.bin`
   - Copy to SD card root
   - Safely eject SD card

3. **Flash to Carvera:**
   - Insert SD card into Carvera
   - Power cycle the machine
   - Firmware will auto-update on boot
   - Verify version/features after boot

## Troubleshooting

### Common Build Errors
- **"unrecognized option"**: Use `-Clean` not `--Clean` (PowerShell syntax)
- **Missing includes**: Verify `gcc-arm-none-eabi-14.2/` directory exists
- **Linker errors**: Run with `-Clean` flag to force full rebuild

### Build Script Syntax
- ✅ Correct (preferred): `.\build\build.ps1 -Clean VERSION=2.1.0c TOOLSETTER=1 VERBOSE=1`
- ✅ Correct (versioned): `.\build\build.ps1 -Clean VERSION=<firmware_version> TOOLSETTER=1 VERBOSE=1`
- ❌ Wrong: `.\build\build.ps1 --Clean`

## Current Implementation Status

### Dual Buffer Refactor (Completed)
- **Status:** ✅ Compilation successful
- **Files Modified:**
  - `src/modules/robot/CompensationPreprocessor.h`
  - `src/modules/robot/CompensationPreprocessor.cpp`
- **Architecture:** Phase-locked dual buffer with stored Gcode pointers
- **Binary Size:** 446,452 bytes (firmware ready for testing)

### Key Features
- 3-move lookahead buffer for corner detection
- Perpendicular offset calculation (G41 LEFT / G42 RIGHT)
- Gcode reconstruction from compensated coordinates
- Memory-safe (proper cleanup in destructor and clear())

## Testing Checklist

### Pre-Deployment
- [ ] Build completes without errors
- [ ] Binary size is reasonable (~446KB)
- [ ] Backup current firmware
- [ ] Backup config.txt

### Post-Deployment
- [ ] Machine boots successfully
- [ ] Console shows firmware version
- [ ] Test G40/G41/G42 commands
- [ ] Verify motion with simple G1 moves
- [ ] Check debug output for compensation values

## Version Naming Convention

Use descriptive names for firmware binaries:
- `firmware_DualBuffer_YYYYMMDD.bin` - Dual buffer refactor builds
- `firmware_Exp_CutComp.bin` - Experimental cutter compensation
- `firmware_Stable_YYYYMMDD.bin` - Stable release candidates
- `firmware_Debug_[Feature].bin` - Debug builds with specific features

## Documentation References
- Full implementation details: `DUAL_BUFFER_REFACTOR_DESIGN.md`
- Cutter compensation theory: `cutter_comp_documentation/CUTTER_COMPENSATION_IMPLEMENTATION.md`
- Developer guide: `DEVELOPER.md`
