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