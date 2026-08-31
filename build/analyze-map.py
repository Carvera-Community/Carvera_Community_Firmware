#!/usr/bin/env python3
"""Print or compare the SRAM layout recorded in GNU ld map files."""

import argparse
import re
import sys
from pathlib import Path


SECTIONS = (".data", ".bss", ".heap", ".stack_dummy", ".AHBSRAM")
SYMBOLS = (
    "__bss_start__",
    "__bss_end__",
    "__MainHeapStart",
    "__MainHeapEnd",
    "__StackLimit",
    "__StackTop",
    "__GeneralAHBStart",
    "__GeneralAHBEnd",
    "__AHBSRAM_start",
    "__AHBSRAM_end",
)


def parse_symbol(text: str, name: str):
    patterns = (
        rf"(0x[0-9a-fA-F]+)\s+PROVIDE\s*\(\s*{name}\s*=",
        rf"(0x[0-9a-fA-F]+)\s+{name}\b",
        rf"{name}\s*=\s*(0x[0-9a-fA-F]+)",
    )
    for pattern in patterns:
        match = re.search(pattern, text)
        if match:
            return int(match.group(1), 0)
    return None


def parse_map(path: Path):
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        raise ValueError(f"cannot read {path}: {exc}") from exc

    sections = {}
    for name in SECTIONS:
        match = re.search(
            rf"^{re.escape(name)}\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)",
            text,
            re.MULTILINE,
        )
        if match:
            sections[name] = (int(match.group(1), 0), int(match.group(2), 0))

    symbols = {}
    for name in SYMBOLS:
        address = parse_symbol(text, name)
        if address is not None:
            symbols[name] = address
    return {"sections": sections, "symbols": symbols}


def region(symbols, start_name, end_name):
    start = symbols.get(start_name)
    end = symbols.get(end_name)
    if start is None or end is None or end < start:
        return None
    return start, end, end - start


def region_size(layout, label, start_name, end_name):
    bounds = region(layout["symbols"], start_name, end_name)
    if bounds:
        return bounds[2]
    if label == "AHB reserved" and ".AHBSRAM" in layout["sections"]:
        return layout["sections"][".AHBSRAM"][1]
    return 0


def print_layout(path: Path, layout):
    print(f"\n=== {path} ===")
    print("\nSections")
    print(f"{'name':<16} {'start':>12} {'bytes':>10} {'end':>12}")
    for name, (start, size) in layout["sections"].items():
        print(f"{name:<16} {start:#010x} {size:10d} {start + size:#010x}")

    print("\nSRAM regions")
    print(f"{'name':<16} {'start':>12} {'bytes':>10} {'end':>12}")
    symbols = layout["symbols"]
    regions = (
        ("main heap", "__MainHeapStart", "__MainHeapEnd"),
        ("AHB heap", "__GeneralAHBStart", "__GeneralAHBEnd"),
        ("AHB reserved", "__AHBSRAM_start", "__AHBSRAM_end"),
        ("stack", "__StackLimit", "__StackTop"),
    )
    for label, start_name, end_name in regions:
        bounds = region(symbols, start_name, end_name)
        if label == "AHB reserved" and bounds is None and ".AHBSRAM" in layout["sections"]:
            start, size = layout["sections"][".AHBSRAM"]
            bounds = start, start + size, size
        if bounds:
            start, end, size = bounds
            print(f"{label:<16} {start:#010x} {size:10d} {end:#010x}")

    main_heap = region(symbols, "__MainHeapStart", "__MainHeapEnd")
    ahb_heap = region(symbols, "__GeneralAHBStart", "__GeneralAHBEnd")
    if main_heap and ahb_heap:
        raw = main_heap[2] + ahb_heap[2]
        print(f"\nUnified heap: {raw} raw bytes; {raw - 16} heap_5 bytes after region sentinels")


def compare(first_path: Path, first, second_path: Path, second):
    print("\n=== comparison (second - first) ===")
    names = sorted(set(first["sections"]) | set(second["sections"]))
    print(f"{'section':<16} {'first':>10} {'second':>10} {'change':>10}")
    for name in names:
        first_size = first["sections"].get(name, (0, 0))[1]
        second_size = second["sections"].get(name, (0, 0))[1]
        print(f"{name:<16} {first_size:10d} {second_size:10d} {second_size - first_size:+10d}")

    print(f"\n{'region':<16} {'first':>10} {'second':>10} {'change':>10}")
    pairs = (
        ("main heap", "__MainHeapStart", "__MainHeapEnd"),
        ("AHB heap", "__GeneralAHBStart", "__GeneralAHBEnd"),
        ("AHB reserved", "__AHBSRAM_start", "__AHBSRAM_end"),
        ("stack", "__StackLimit", "__StackTop"),
    )
    for label, start_name, end_name in pairs:
        first_size = region_size(first, label, start_name, end_name)
        second_size = region_size(second, label, start_name, end_name)
        print(f"{label:<16} {first_size:10d} {second_size:10d} {second_size - first_size:+10d}")

    print(f"\nfirst: {first_path}\nsecond: {second_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compare", action="store_true", help="compare two map files")
    parser.add_argument("map_files", nargs="+", type=Path)
    args = parser.parse_args()

    expected = 2 if args.compare else 1
    if len(args.map_files) != expected:
        parser.error(f"expected {expected} map file{'s' if expected == 2 else ''}")

    try:
        first_path = args.map_files[0]
        first = parse_map(first_path)
        print_layout(first_path, first)
        if args.compare:
            second_path = args.map_files[1]
            second = parse_map(second_path)
            print_layout(second_path, second)
            compare(first_path, first, second_path, second)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
