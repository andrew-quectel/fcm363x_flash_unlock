# FCM363X Flash Unlock Tool

## Project Overview

This project is a specialized XMC Flash unlock tool designed for FCM363X modules. It addresses various protection lock issues in XMC XM25QH64D Flash chips.

## Build Instructions

1. **Development Environment**: MCUXpresso for VS Code
2. **Build Configuration**: Select `ram_debug` or `ram_release` configuration

## Usage

### Method 1: JLink Command Line Flashing

1. After compilation, locate `freertos_hello.bin` in `armgcc/ram_debug/` or `armgcc/ram_release/` directory
2. Connect JLink to target board
3. Execute the following JLink commands:

```bash
# Load firmware to SRAM
loadbin freertos_hello.bin 0x20000000

# Set program counter to Reset_Handler (address may vary with compilation)
setpc 0x200012fc

# Start execution
go
```

### Method 2: MCUXpresso IDE Debug

1. Import project into MCUXpresso IDE
2. Select ram_debug configuration
3. Connect debugger and start debug session
4. Run the program

## Output Example

```
Starting Flash JEDEC ID and Status Register reading...
Flash JEDEC ID: 0x204017

--- Flash Information ---
Vendor ID: 0x20 (XMC, Supported)
Memory Type: 0x40
Capacity ID: 0x17 (8 MB)
-------------------------

--- Status Registers ---
Status Register 1 (0x05): 0x40 [SEC=1]
Status Register 2 (0x35): 0x43 [SRP1=1-HW_PROTECTED] [CMP=1]
...

Flash protection detected. Attempting to unlock...
*** CRITICAL: Hardware Write Protection Detected! ***
Attempting volatile unlock...
Attempting non-volatile unlock...
Hardware unlock sequences completed.
Flash unlock successful!
```
