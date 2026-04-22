# Carvera Community Firmware Analysis

## Architecture Overview

The Carvera Community Firmware is based on Smoothie firmware, which is an open-source CNC control system. The firmware is written in C++ and follows a modular architecture. This flavor of smoothieware is designed specifically for the Carvera series of CNC machines from Makera. 

### Key Components:

1. **Kernel** - Central control system that:
    - Acts as the main coordinator between all firmware components
    - Manages system initialization and runtime operations
    - Controls global configuration settings and parameters such as steps/mm, max speeds, and boundaries
      - Configuration management is handled through a combination of:
        - Static configuration files (e.g., JSON, XML) that define machine parameters
        - Dynamic runtime adjustments via G-code commands
        - User interfaces for manual configuration
    - Key responsibilities include:
      - Command queue management
      - Real-time scheduling of operations
      - Module communication and synchronization
      - System state management
      - Error handling and recovery
    - Leverages global variables for:
      - Machine state (current position, speed, acceleration)
      - System configuration (steps/mm, max speeds, boundaries)
      - Tool parameters (current tool, offsets)
      - Operation flags (is_homed, is_busy, error_state)
    - Interfaces with modules through:
      - Standard command interfaces
      - Event callbacks such as `on_tool_change`, `on_probe_complete`
      - Shared memory spaces for real-time data exchange such as tool offsets and probe results
      - Message queues such as `tool_change_queue`, `probe_result_queue`
2. **Modules** - Functional units like:
   - ATC (Automatic Tool Changer)
   - Robot/Motion Control
   - Spindle Control
   - Tool Management
   - Z-Probe functionality

## Event System Architecture

The firmware uses a **publish-subscribe event system** where modules register for specific events and the kernel broadcasts events to all registered handlers.

### Event Broadcasting Flow

When G-code is received, it flows through this pipeline:

```
SD Card File / Serial Input / MDI
    ↓
Player::on_main_loop() [reads file line by line]
    ↓
ON_CONSOLE_LINE_RECEIVED event
    ↓
GcodeDispatch::on_console_line_received()
    - Parses raw text into Gcode object
    - Validates syntax
    - Handles special cases (G53, M28/M29, M112)
    - Executes some commands directly (M115, M500-504)
    - Sends "ok" responses
    ↓
ON_GCODE_RECEIVED event (broadcasts to ALL registered modules)
    ↓
    ├→ Robot::on_gcode_received()           [motion, compensation, WCS]
    ├→ SimpleShell::on_gcode_received()     [file commands]
    ├→ Player::on_gcode_received()          [playback control]
    ├→ Spindle::on_gcode_received()         [M3/M4/M5]
    ├→ ATCHandler::on_gcode_received()      [tool change, calibration]
    ├→ Laser::on_gcode_received()           [laser control]
    └→ [other modules...]
```

**Key Points:**
- **NOT direct function calls** - it's an event broadcast to multiple subscribers
- **Sequential processing** - handlers are called one after another in registration order
- **Each module filters** - checks `gcode->has_g`, `gcode->g`, `gcode->has_m`, `gcode->m` to determine if relevant
- **Shared Gcode object** - all handlers receive a pointer to the same structured object

### Gcode Object Structure

The `Gcode` class (`Gcode.h`) is a **structured object** that represents a parsed G-code command:

```cpp
class Gcode {
    public:
        unsigned int m;              // M-code number (if has_m)
        unsigned int g;              // G-code number (if has_g)
        unsigned int line;           // Line number in file
        
        struct {
            bool add_nl:1;           // Add newline after ok
            bool has_m:1;            // Command has M-code
            bool has_g:1;            // Command has G-code
            bool stripped:1;         // Parameters stripped
            bool is_error:1;         // Error flag
            uint8_t subcode:3;       // Subcode (e.g., M600.1)
        };
        
        StreamOutput* stream;        // Output stream for responses
        string txt_after_ok;         // Text to append to "ok"
        
        // Methods
        bool has_letter(char letter);           // Check if parameter exists
        float get_value(char letter);           // Get parameter value
        int get_int(char letter);               // Get integer value
        const char* get_command();              // Get original command string
};
```

**Capabilities:**
- Handles ANY letter parameter (A-Z) with numeric values
- Supports subcodes (e.g., M600.1, M98.1)
- Examples:
  - `G41 D10` → `has_g=true, g=41, has_letter('D')=true, get_value('D')=10.0`
  - `G1 X10 Y20 F100` → `has_g=true, g=1, has_letter('X')=true, get_value('X')=10.0`
  - `M3 S5000` → `has_m=true, m=3, has_letter('S')=true, get_value('S')=5000.0`

### Module Event Handlers

Each module registers for events and processes relevant G-codes:

#### Robot Module (Robot.cpp)
**Handles:**
- **Motion:** G0 (rapid), G1 (linear), G2/G3 (arcs)
- **Dwell:** G4
- **WCS Management:** G10 (set WCS offsets), G54-G59 (select WCS)
- **Plane Selection:** G17 (XY), G18 (XZ), G19 (YZ)
- **Units:** G20 (inches), G21 (mm)
- **Cutter Compensation:** G40 (off), G41 (left), G42 (right)
- **Positioning Mode:** G90 (absolute), G91 (relative), G92 (set position)

#### SimpleShell Module (SimpleShell.cpp)
**Handles:**
- M20 (list SD card files)
- M30 (delete file)
- M331 (vacuum mode toggle)

#### Player Module (Player.cpp)
**Handles:**
- M1 (optional stop)
- M21 (initialize SD card)
- M23 (select file)
- M24 (start/resume playback)
- M25 (pause playback)
- M26 (reset print)
- M27 (report print progress)
- M32 (select and start file)
- M97 (goto line number)
- M98.1 (call macro/subprogram)
- M99 (return from macro)
- M600 (suspend print)

#### ATCHandler Module (ATCHandler.cpp)
**Handles:**
- M494.1 (manual tool pickup)
- M494.2 (manual tool drop)
- M494.3 (manual probe pickup)
- M495 (set path origin)
- M496.1 (calibrate anchor 1)
- M496.2 (calibrate anchor 2)
- M498 (tool calibration)
- M730 (calibrate on anchor 2)

#### Spindle Module
**Handles:**
- M3 (spindle on clockwise)
- M4 (spindle on counter-clockwise)
- M5 (spindle off)
- M957 (set spindle parameters)

### Robot::on_gcode_received() Detailed Flow

When Robot receives a G-code event, it processes through the cutter compensation system:

```cpp
void Robot::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    
    // 1. Debug: Show incoming command
    gcode->stream->printf(">>BUFFER: %s\n", gcode->get_command());
    
    // 2. Buffer the command in compensation preprocessor
    if (compensation_preprocessor->buffer_gcode(gcode)) {
        
        // 3. Try to retrieve processed output
        Gcode* output = compensation_preprocessor->get_compensated_gcode();
        
        // 4. Process ALL available outputs
        while (output != nullptr) {
            gcode->stream->printf(">>OUTPUT: %s\n", output->get_command());
            
            // Execute the command (contains switch with case 41/42)
            process_buffered_command(output);
            
            // Free memory (we own this object now)
            delete output;
            
            // Try to get next output
            output = compensation_preprocessor->get_compensated_gcode();
        }
        
        return;  // Successfully buffered
    }
    
    // Fallback: buffer full, process directly
    process_buffered_command(gcode);
}
```

**Why the while loop?**

The compensation preprocessor uses a **circular buffer with lookahead**. One input command may produce:
- **Nothing (nullptr):** Still building lookahead (need 3 moves for compensation)
- **One command:** Non-motion commands pass through immediately
- **Multiple commands:** When flushing buffer (e.g., G40), outputs many commands

**Example Scenarios:**

**Scenario 1: Non-motion command (G90)**
```
Input: G90
buffer_gcode(G90) → true, passes through immediately
get_compensated_gcode() → returns G90
  Loop iteration 1: process G90, delete it
  get_compensated_gcode() → returns nullptr
  Exit loop
```

**Scenario 2: Building lookahead (1st move after G41)**
```
Input: G1 X10 Y10
buffer_gcode(G1 X10 Y10) → true, buffer_count = 1
get_compensated_gcode() → returns nullptr (need 3 moves)
  Loop never executes
```

**Scenario 3: Lookahead complete (3rd move)**
```
Input: G1 X50 Y50
buffer_gcode(G1 X50 Y50) → true, buffer_count = 3, calculates lead-in
get_compensated_gcode() → returns first compensated move
  Loop iteration 1: process move, delete it
  get_compensated_gcode() → returns nullptr (only 1 ready)
  Exit loop
```

**Scenario 4: G40 flush**
```
Input: G40
buffer_gcode(G40) → true, sets is_flushing flag
get_compensated_gcode() → flushes all 3 buffered moves
  Loop iteration 1: process move 1, delete it
  Loop iteration 2: process move 2, delete it  
  Loop iteration 3: process move 3, delete it
  get_compensated_gcode() → returns nullptr
  Exit loop
Then G40 handler disables compensation
```

## Program Flow

1. **Command Input**
   - G-code commands are received by the system
   - Commands are parsed and validated
   - Commands are queued in the conveyor system

2. **Command Processing**
   - Kernel coordinates between different modules
   - Movement commands are translated to stepper motor signals
   - Tool changes and probing operations are handled by specialized modules

3. **Execution**
   - Stepper motors execute movement
   - Tool changes are performed
   - Status updates are provided back to the control system

## ATCHandler.cpp Analysis

The ATCHandler (Automatic Tool Changer Handler) is responsible for managing tool changes and calibration operations.

### Key Functions:

1. **Tool Change Operations**
   - `fill_change_scripts()` - Manages complete tool change sequence
   - `fill_drop_scripts()` - Handles tool dropping
   - `fill_pick_scripts()` - Handles tool picking
   - `fill_manual_drop_scripts()` - Manual tool drop operations
   - `fill_manual_pickup_scripts()` - Manual tool pickup operations

2. **Calibration Functions**
   - `fill_cali_scripts()` - Primary calibration routine that:
     - Controls probe laser
     - Manages tool clamping
     - Performs Z-axis calibration
     - Handles tool length and diameter measurement
     - Controls spindle direction during diameter calibration
     - Manages safe movements

### Script Queue System

The ATCHandler uses a queue-based system where G-code commands are pushed into a script queue function called `push_script()`:

```cpp
this->push_script("M494.1");  // Example command push

### M851/M852 Fan Control Commands

The fan control commands M851/M852 are processed through multiple files:

1. **Command Processing Flow**:
   - Commands enter through GcodeDispatch
   - Processed by Switch module for fan control
   - Maps to specific PWM pins on the LPC1768

2. **Hardware Implementation**:
   - Fan control uses PWM output pins
   - For Carvera-specific hardware:
     - EXT port control pin defaults to P2.4
     - Uses hardware PWM capability of LPC1768
     - PWM configuration handled through Pin.cpp hardware_pwm() function

3. **Pin Configuration**:
   ```cpp
   // Fan control pin definition (P2.4)
   #define spindle_pwm_pin_checksum CHECKSUM("pwm_pin")
   // Default pin configuration in config
   this->pwm_pin = dummy_pin->hardware_pwm(); // P2.4

      
   This shows that the EXT port control pin is P2.4 on the LPC1768 microcontroller.