#!/usr/bin/env python3
"""
Map File Memory Layout Analyzer

This script analyzes map files to extract critical memory layout information,
including section sizes and important symbol addresses. It can compare two builds
to identify memory layout changes that might cause heap/stack collisions.

Usage:
    ./map_analyzer.py <map_file>
    ./map_analyzer.py --compare <map_file_a> <map_file_b>
"""

import argparse
import re
import sys
from collections import defaultdict

# Critical symbols to look for - including potential variations
CRITICAL_SYMBOLS = [
    "__bss_end__", "_end", "_ebss", "end", 
    "__StackTop", "_estack", "STACK_TOP", 
    "g_maximumHeapAddress", "_maxHeapAddr",
    "__AHB_block_start", "__AHB_dyn_start", "__AHB_end",
    "MemoryPool::alloc", "MemoryPool::dealloc", "_sbrk", 
    "doesHeapCollideWithStack", "_HeapCollideStackCheck"
]

# Important sections to analyze
IMPORTANT_SECTIONS = [".text", ".data", ".bss", ".heap", ".stack", ".stack_dummy", ".AHBSRAM"]

# Regex to capture object file contributions within a section
# Example: AHBSRAM        0x2007c200      0x2e0 ../LPC1768/./main.o
obj_contribution_pattern = re.compile(r'^\s*[\w.]+\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)\s+(.*\.o)\s*$')

def parse_map_file(map_file):
    """Parse a map file to extract section sizes and symbol addresses."""
    result = {
        "sections": {},
        "symbols": {},
        "ahbsram_details": [] # Store detailed AHBSRAM contributions
    }
    
    try:
        with open(map_file, 'r') as f:
            map_content = f.read()
    except FileNotFoundError:
        print(f"Error: Map file {map_file} not found.")
        sys.exit(1)
    except Exception as e:
        print(f"Error reading map file: {e}")
        sys.exit(1)
    
    # Extract memory sections - try multiple patterns for different map file formats
    
    # Pattern 1: Look for standard section headers in the memory map
    for section_name in IMPORTANT_SECTIONS:
        section_pattern = rf'\.{section_name[1:] if section_name.startswith(".") else section_name}\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)'
        section_match = re.search(section_pattern, map_content)
        if section_match:
            try:
                addr, size = section_match.groups()
                result["sections"][section_name] = {
                    "start": int(addr, 0),
                    "size": int(size, 0)
                }
            except (ValueError, IndexError):
                pass
                
    # Pattern 2: Look for Memory Configuration section
    memory_section = re.search(r'Memory Configuration.*?\n\n(.*?)\n\n', 
                               map_content, re.DOTALL)
    if memory_section:
        mem_content = memory_section.group(1)
        for line in mem_content.split('\n'):
            if line.strip():
                # Try to find sections by name
                for section_name in IMPORTANT_SECTIONS:
                    section_str = section_name[1:] if section_name.startswith('.') else section_name
                    if section_str in line:
                        # Extract hex values from the line
                        hex_values = re.findall(r'0x[0-9a-fA-F]+', line)
                        if len(hex_values) >= 2:
                            try:
                                start_addr = int(hex_values[0], 0)
                                size = int(hex_values[1], 0)
                                result["sections"][section_name] = {
                                    "start": start_addr,
                                    "size": size
                                }
                            except (ValueError, IndexError):
                                pass
    
    # Look for the Linker script and memory map section
    linker_section = re.search(r'Linker script and memory map.*?\n\n(.*)', 
                               map_content, re.DOTALL)
    if linker_section:
        section_content = linker_section.group(1)
        
        # Extract section information
        # Pattern to find symbols defined ONLY by address and name within a section
        # Example: 0x2007d4e0                xbuff
        in_section_symbol_pattern = r'^\s*(0x[0-9a-fA-F]+)\s+([\w:]+)\s*$'
        current_section = None
        section_size = 0
        section_addr = 0
        current_ahb_contribution_index = -1
        
        for line in section_content.split('\n'):
            # Check for section header
            section_match = re.match(r'^(\.[\w._]+)\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)', line)
            if section_match:
                section_name, addr, size = section_match.groups()
                if section_name in IMPORTANT_SECTIONS:
                    try:
                        result["sections"][section_name] = {
                            "start": int(addr, 0),
                            "size": int(size, 0)
                        }
                    except ValueError:
                        pass
                current_section = section_name
                current_ahb_contribution_index = -1 # Reset contribution tracking when section changes
                continue
            
            # If we are inside the .AHBSRAM section, look for symbols defined purely by address and name
            if current_section == ".AHBSRAM":
                # First, check if this line defines an object file contribution
                contribution_match = obj_contribution_pattern.match(line)
                if contribution_match:
                    addr_str, size_str, file_name = contribution_match.groups()
                    try:
                        contribution = {
                            "file_name": file_name.strip(),
                            "start_addr": int(addr_str, 0),
                            "size": int(size_str, 0),
                            "symbols": {}
                        }
                        result["ahbsram_details"].append(contribution)
                        current_ahb_contribution_index = len(result["ahbsram_details"]) - 1
                    except ValueError:
                        pass # Ignore if addresses/size aren't valid hex
                    continue # Processed contribution line

                # If not a contribution line, check if it's a symbol within the current contribution
                symbol_match = re.match(in_section_symbol_pattern, line)
                if symbol_match:
                    addr_str, symbol = symbol_match.groups()
                    # Check if the symbol itself looks like a hex number (likely an error)
                    if not re.match(r'^0x[0-9a-fA-F]+$', symbol):
                        # Avoid matching linker directives or keywords
                        if symbol.upper() not in ['PROVIDE']:
                            # Add symbol to the *current* contribution if we are tracking one
                            if current_ahb_contribution_index >= 0:
                                try:
                                    current_contribution = result["ahbsram_details"][current_ahb_contribution_index]
                                    current_contribution["symbols"][symbol] = int(addr_str, 0)
                                except (ValueError, IndexError):
                                    pass # Ignore errors
                    continue # Found a potential symbol line, move to next line
                else:
                    # If the line is neither a contribution nor a symbol, reset tracking for safety
                    current_ahb_contribution_index = -1
            
            # Look for symbols within sections - try multiple patterns
            
            # Pattern 1: Standard format with symbol at end (GCC ARM)
            symbol_match = re.search(r'\s*(0x[0-9a-fA-F]+)\s+([a-zA-Z0-9_:]+(?:::[a-zA-Z0-9_]+)*)', line)
            if symbol_match and len(symbol_match.groups()) == 2:
                addr, symbol = symbol_match.groups()
                if symbol in CRITICAL_SYMBOLS or any(crit in symbol for crit in CRITICAL_SYMBOLS):
                    try:
                        result["symbols"][symbol] = int(addr, 0)
                    except ValueError:
                        pass
                    continue
            
            # Pattern 2: Format with symbol in middle of line
            # Example: " .bss._end          0x20007960        0x0 ./build/arm-none-eabi/debug/libc.a(lib_a-sbrkr.o)"
            for symbol in CRITICAL_SYMBOLS:
                if symbol in line:
                    addr_match = re.search(r'0x[0-9a-fA-F]+', line)
                    if addr_match:
                        try:
                            result["symbols"][symbol] = int(addr_match.group(0), 0)
                        except ValueError:
                            pass
    
    # Sometimes symbols are in a separate section
    symbol_table = re.search(r'SYMBOL TABLE:.*?\n(.*?)(?:\n\n|\Z)', 
                           map_content, re.DOTALL)
    if symbol_table:
        for line in symbol_table.group(1).split('\n'):
            parts = line.strip().split()
            if len(parts) >= 2:
                addr_match = re.match(r'0x[0-9a-fA-F]+', parts[0])
                if addr_match:
                    addr = parts[0]
                    symbol = parts[-1]  # Symbol name is typically the last part
                    if symbol in CRITICAL_SYMBOLS or any(crit in symbol for crit in CRITICAL_SYMBOLS):
                        try:
                            result["symbols"][symbol] = int(addr, 0)
                        except ValueError:
                            pass
    
    # Look for global symbols table if present
    global_symbols = re.search(r'Cross Reference Table.*?\n\n(.*?)(?:\n\n|\Z)', 
                              map_content, re.DOTALL)
    if global_symbols:
        for line in global_symbols.group(1).split('\n'):
            for symbol in CRITICAL_SYMBOLS:
                if symbol in line:
                    addr_match = re.search(r'0x[0-9a-fA-F]+', line)
                    if addr_match:
                        try:
                            result["symbols"][symbol] = int(addr_match.group(0), 0)
                        except ValueError:
                            pass
    
    return result

def print_result(file_name, result):
    """Print the analysis result in a readable format."""
    print(f"\n=== Analysis of {file_name} ===")
    
    # Print section sizes
    print("\nSection Sizes:")
    print(f"{'Section':<15} {'Start Address':<14} {'Size':<12} {'End Address':<14}")
    print("-" * 60)
    for section, info in sorted(result["sections"].items()):
        end_addr = info["start"] + info["size"]
        print(f"{section:<15} {info['start']:#010x} {info['size']:<12} {end_addr:#010x}")
    
    # Print symbol addresses
    print("\nCritical Symbol Addresses:")
    print(f"{'Symbol':<30} {'Address':<14}")
    print("-" * 45)
    for symbol, addr in sorted(result["symbols"].items(), key=lambda x: x[1]):
        print(f"{symbol:<30} {addr:#010x}")
    
    # Calculate heap space if possible
    if "__bss_end__" in result["symbols"] and "__StackTop" in result["symbols"]:
        heap_space = result["symbols"]["__StackTop"] - result["symbols"]["__bss_end__"]
        print(f"\nEstimated Available Heap Space: {heap_space} bytes ({heap_space/1024:.2f} KB)")
    # AHBSRAM region summary
    if ".AHBSRAM" in result.get("sections", {}):
        ahb = result["sections"][".AHBSRAM"]
        ahb_start = ahb["start"]
        ahb_size = ahb["size"]
        ahb_end = ahb_start + ahb_size
        print("\nAHBSRAM Region:")
        print(f" Start  : {ahb_start:#010x} (Static Start: __AHB_block_start)")
        print(f" Size   : {ahb_size} bytes")
        print(f" End    : {ahb_end:#010x} (Dynamic Start: __AHB_dyn_start)")

        # Print detailed contributions
        details = result.get("ahbsram_details", [])
        if details:
            print("\n Statically allocated contributions in .AHBSRAM:")
            for contrib in details:
                 print(f"  Contribution: {contrib['file_name']} (Start: {contrib['start_addr']:#010x}, Size: {contrib['size']} bytes)")
                 if contrib["symbols"]:
                     print(f"    Symbols:")
                     for sym, addr in sorted(contrib["symbols"].items(), key=lambda x: x[1]):
                         print(f"      {sym:<26} {addr:#010x}")
        else:
            print("\n No detailed static contributions found within .AHBSRAM section in map.")

        # Print overall AHB end if available
        if "__AHB_end" in result.get("symbols", {}):
             print(f" Pool End : {result['symbols']['__AHB_end']:#010x} (__AHB_end)")
             pool_size = result['symbols']['__AHB_end'] - ahb_end
             print(f" Pool Size: {pool_size} bytes ({pool_size/1024:.2f} KB) <-- Area for MemoryPool")

def compare_builds(build_a, build_b):
    """Compare two builds and print the differences."""
    print("\n=== COMPARISON RESULTS ===")
    
    # Compare sections
    print("\nSection Size Comparison:")
    print(f"{'Section':<15} {'Build A':<12} {'Build B':<12} {'Difference':<12} {'% Change':<10}")
    print("-" * 60)
    
    for section in sorted(set(build_a["sections"].keys()).union(build_b["sections"].keys())):
        size_a = build_a["sections"].get(section, {}).get("size", 0)
        size_b = build_b["sections"].get(section, {}).get("size", 0)
        diff = size_b - size_a
        if size_a > 0:
            percent = (diff / size_a) * 100
            percent_str = f"{percent:.2f}%"
        else:
            percent_str = "N/A"
        
        print(f"{section:<15} {size_a:<12} {size_b:<12} {diff:<+12} {percent_str:<10}")
    
    # Compare symbols
    print("\nCritical Symbol Address Comparison:")
    print(f"{'Symbol':<30} {'Build A':<12} {'Build B':<12} {'Difference':<12}")
    print("-" * 70)
    
    for symbol in sorted(set(build_a["symbols"].keys()).union(build_b["symbols"].keys())):
        addr_a = build_a["symbols"].get(symbol, 0)
        addr_b = build_b["symbols"].get(symbol, 0)
        diff = addr_b - addr_a
        
        status = ""
        if symbol == "__bss_end__" or symbol == "_end":
            if diff > 0:
                status = " [WARN: Higher end of BSS reduces heap space]"
        elif symbol == "__StackTop":
            if diff < 0:
                status = " [WARN: Lower stack top reduces heap space]"
        
        print(f"{symbol:<30} {addr_a:#010x} {addr_b:#010x} {diff:<+12}{status}")
    
    # Calculate available heap space (if symbols are available)
    if "__bss_end__" in build_a["symbols"] and "__StackTop" in build_a["symbols"] and \
       "__bss_end__" in build_b["symbols"] and "__StackTop" in build_b["symbols"]:
        heap_space_a = build_a["symbols"]["__StackTop"] - build_a["symbols"]["__bss_end__"]
        heap_space_b = build_b["symbols"]["__StackTop"] - build_b["symbols"]["__bss_end__"]
        diff = heap_space_b - heap_space_a
        print(f"\nEstimated Available Heap Space:")
        print(f"Build A: {heap_space_a} bytes ({heap_space_a/1024:.2f} KB)")
        print(f"Build B: {heap_space_b} bytes ({heap_space_b/1024:.2f} KB)")
        print(f"Difference: {diff:+} bytes ({diff/1024:.2f} KB)")
        
        if diff < 0:
            print("\n[WARNING] Available heap space decreased in Build B!")
            print("This could explain heap/stack collision issues.")

def main():
    parser = argparse.ArgumentParser(description="Analyze map files for memory layout")
    parser.add_argument("map_file", nargs="?", help="Path to a map file to analyze")
    parser.add_argument("--compare", "-c", nargs=2, metavar=("MAP_A", "MAP_B"), 
                      help="Compare two map files")
    
    args = parser.parse_args()
    
    try:
        if args.compare:
            print(f"Comparing map files: {args.compare[0]} and {args.compare[1]}")
            build_a = parse_map_file(args.compare[0])
            build_b = parse_map_file(args.compare[1])
            print_result(args.compare[0], build_a)
            print_result(args.compare[1], build_b)
            compare_builds(build_a, build_b)
        elif args.map_file:
            result = parse_map_file(args.map_file)
            print_result(args.map_file, result)
        else:
            parser.print_help()
            sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        print(f"Stack trace: {sys.exc_info()}")
        sys.exit(1)

if __name__ == "__main__":
    main()