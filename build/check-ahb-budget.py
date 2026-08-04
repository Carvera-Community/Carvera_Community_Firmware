#!/usr/bin/env python3
"""
Static boot memory budget check for known SD configs.

Checks two resources that the linker cannot see:

1. AHB MemoryPool — permanent runtime allocs (new(AHB), BlockQueue, cart grid,
   flex buffer).
2. Main SRAM heap vs config-cache window — especially
   flex_compensation_always_active, which triggers fopen (heap FIL_t ~548B) and
   std::function bind. Heap growth into the live config-cache region hard-resets
   (boot loop).

Usage:
    ./build/check-ahb-budget.py \\
        --map LPC1768/main.map \\
        --elf LPC1768/main.elf \\
        --configs-dir tests/TEST_memory_budget/configs

Exit codes:
    0  all configs fit with the configured margins
    1  one or more configs exceed a budget (or fatal parse error)
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

# AHB MemoryPool header is one uint32_t; payloads are 4-byte aligned.
POOL_HEADER = 4

# newlib-nano malloc: 8-byte header, 8-byte alignment (typical).
MALLOC_HEADER = 8

CONFIG_CACHE_CAPACITY = 350  # ConfigCache.h

# Sizes only needed when --elf/gdb is unavailable. Prefer live DWARF sizeof().
FALLBACK_SIZES = {
    "Block": 368,
    "ConfigValue": 26,
    "FILE": 104,
    "FIL_t": 548,
    "FATFileHandle": 32,
    "Switch": 92,
    "TemperatureControl": 148,
    "Pin": 8,
    "Pwm": 28,
    "Thermistor": 40,
}

# Temporary makers/pools: constructed then deleted after load_*; do not count.
TRANSIENT_CTOR_TYPES = {
    "SwitchPool",
    "TemperatureControlPool",
    "SpindleMaker",
    "ExtruderMaker",
}

# Extra types to sizeof() for config-driven / AHB.alloc math.
AUX_SIZEOF_TYPES = (
    "Block",
    "ConfigValue",
    "FILE",
    "FIL_t",
    "FATFileHandle",
    "Switch",
    "TemperatureControl",
    "Pin",
    "Pwm",
    "Thermistor",
    "CartGridStrategy",
    "PWMSpindleControl",
    "PIDPWMSpindleControl",
    "AnalogSpindleControl",
    "HuanyangSpindleControl",
    "VESCSpindleControl",
)

# Boot .cpp files whose active `new Type` / `new(AHB) Type` sites define the
# permanent module set. Types missing from the ELF (excluded by makefile) are
# dropped after sizeof() probe.
BOOT_CTOR_SOURCES = (
    "src/libs/Kernel.cpp",
    "src/main.cpp",
    "src/libs/Config.cpp",
)

# Code defaults when neither firm nor SD config sets a key.
CODE_DEFAULTS = {
    "planner_queue_size": 32,
    "leveling-strategy.rectangular-grid.enable": True,
    "leveling-strategy.rectangular-grid.size": 7,
    "leveling-strategy.rectangular-grid.flex_x_points": 30,
    "leveling-strategy.rectangular-grid.flex_compensation_always_active": False,
    "usb_msc.enable": True,
    "spindle.enable": True,
    "spindle.type": "pwm",
}

NEW_AHB_RE = re.compile(
    r"new\s*\(\s*AHB\s*\)\s*([A-Za-z_][A-Za-z0-9_:]*)\s*[\(;]"
)
# Allow `new Foo(`, `new Foo;`, and `new Foo)` (e.g. add_module(new WifiProvider)).
NEW_MAIN_RE = re.compile(
    r"(?<![\w:])new\s+([A-Za-z_][A-Za-z0-9_:]*)\s*[\(;)]"
)


@dataclass
class BootCtor:
    type_name: str
    region: str  # "ahb" or "main"
    source: str

# Extra main-heap bytes not modeled module-by-module (vectors, strings, I2C,
# InterruptIn, transient SD FIL_t during factory/config read, allocator waste).
# Calibrated so configs/1 (flex_compensation_always_active) exceeds the
# cache-live window on current builds, matching observed boot loops, while
# stock configs/2 still fits. Refine from on-device `mem -v` after init.
BOOT_HEAP_UNACCOUNTED = 5500

# Peak main-heap during flex autoload beyond FIL_t/FILE/FATFileHandle:
# std::function bind target + printf/%f scratch.
FLEX_AUTOLOAD_SLACK = 320


@dataclass
class AllocLine:
    name: str
    payload: int
    cost: int
    note: str = ""


@dataclass
class BudgetResult:
    label: str
    config_path: Path
    machine: str
    firm_default: Path
    pool_size: int
    margin: int
    ahb_lines: List[AllocLine] = field(default_factory=list)
    main_lines: List[AllocLine] = field(default_factory=list)
    cache_live_usable: int = 0
    full_heap_gap: int = 0
    always_active: bool = False
    deferred_loads: Dict[str, bool] = field(default_factory=dict)
    ahb_ok: bool = False
    main_ok: bool = False
    notes: List[str] = field(default_factory=list)

    @property
    def ahb_total(self) -> int:
        return sum(line.cost for line in self.ahb_lines)

    @property
    def main_total(self) -> int:
        return sum(line.cost for line in self.main_lines)

    @property
    def ok(self) -> bool:
        return self.ahb_ok and self.main_ok


def repo_root() -> Path:
    return Path(__file__).resolve().parent.parent


def align4(nbytes: int) -> int:
    return (nbytes + 3) & ~3


def align8(nbytes: int) -> int:
    return (nbytes + 7) & ~7


def pool_cost(nbytes: int) -> int:
    """Bytes consumed from the AHB pool for one successful alloc of nbytes."""
    return align4(nbytes) + POOL_HEADER


def malloc_cost(nbytes: int) -> int:
    """Estimated bytes consumed from the main heap for one malloc/new."""
    return align8(nbytes) + MALLOC_HEADER


def parse_config_file(path: Path) -> Dict[str, str]:
    """Parse Smoothie/Carvera ASCII config into key -> raw value string."""
    values: Dict[str, str] = {}
    text = path.read_text(encoding="utf-8", errors="replace")
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        m = re.match(r"^(\S+)\s+(\S+)", line)
        if not m:
            continue
        key, value = m.group(1), m.group(2)
        value = value.split("#", 1)[0]
        values[key] = value
    return values


def as_bool(value: Optional[str], default: bool) -> bool:
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def as_int(value: Optional[str], default: int) -> int:
    if value is None:
        return default
    try:
        return int(float(value))
    except ValueError:
        return default


def merge_configs(*layers: Dict[str, str]) -> Dict[str, str]:
    merged: Dict[str, str] = {}
    for layer in layers:
        merged.update(layer)
    return merged


def detect_machine(user_cfg: Dict[str, str], config_dir: Path) -> str:
    name = user_cfg.get("wifi.machine_name", "").upper()
    if "AIR" in name or "CA1" in name:
        return "carvera_air"
    if "C1" in name or name.startswith("CARVERA"):
        if "AIR" not in name:
            return "carvera"

    readme = config_dir / "README.md"
    if readme.exists():
        text = readme.read_text(encoding="utf-8", errors="replace").upper()
        if "CA1" in text or "AIR" in text:
            return "carvera_air"
        if "C1" in text:
            return "carvera"
    return "carvera"


def firm_default_path(root: Path, machine: str) -> Path:
    if machine == "carvera_air":
        return root / "src" / "config2.default"
    return root / "src" / "config.default"


def parse_map_symbols(map_path: Path, names: Iterable[str]) -> Dict[str, int]:
    text = map_path.read_text(encoding="utf-8", errors="replace")
    symbols: Dict[str, int] = {}
    for name in names:
        patterns = (
            rf"(0x[0-9a-fA-F]+)\s+PROVIDE\s*\(\s*{name}\s*=",
            rf"(0x[0-9a-fA-F]+)\s+{name}\b",
            rf"{name}\s*=\s*(0x[0-9a-fA-F]+)",
            rf"{name}\s+(0x[0-9a-fA-F]+)",
        )
        addr = None
        for pattern in patterns:
            m = re.search(pattern, text)
            if m:
                addr = int(m.group(1), 0)
                break
        if addr is None:
            raise ValueError(f"Symbol {name} not found in {map_path}")
        symbols[name] = addr
    return symbols


def parse_ahb_pool_size(map_path: Path) -> Tuple[int, int, int]:
    symbols = parse_map_symbols(map_path, ("__AHB_dyn_start", "__AHB_end"))
    dyn = symbols["__AHB_dyn_start"]
    end = symbols["__AHB_end"]
    if end <= dyn:
        raise ValueError(f"Invalid AHB pool bounds: dyn={dyn:#x} end={end:#x}")
    return end - dyn, dyn, end


def parse_main_ram_layout(map_path: Path) -> Tuple[int, int, int]:
    """
    Return (bss_end, stack_limit, stack_top).
    Config cache lives at stack_limit - capacity*sizeof(ConfigValue).
    """
    text = map_path.read_text(encoding="utf-8", errors="replace")
    # Prefer __StackLimit PROVIDE line; fall back to __StackTop - .stack_dummy.
    symbols: Dict[str, int] = {}
    for name in ("__bss_end__", "__end__", "__StackTop", "__StackLimit"):
        for pattern in (
            rf"(0x[0-9a-fA-F]+)\s+PROVIDE\s*\(\s*{name}\s*=",
            rf"(0x[0-9a-fA-F]+)\s+{name}\b",
            rf"{name}\s*=\s*(0x[0-9a-fA-F]+)",
        ):
            m = re.search(pattern, text)
            if m:
                symbols[name] = int(m.group(1), 0)
                break

    bss_end = symbols.get("__bss_end__") or symbols.get("__end__")
    stack_top = symbols.get("__StackTop")
    if bss_end is None or stack_top is None:
        raise ValueError("Could not find __bss_end__/__end__ and __StackTop in map")

    stack_limit = symbols.get("__StackLimit")
    if stack_limit is None:
        m = re.search(r"\.stack_dummy\s+0x[0-9a-fA-F]+\s+(0x[0-9a-fA-F]+)", text)
        if not m:
            raise ValueError("Could not find __StackLimit or .stack_dummy size")
        stack_limit = stack_top - int(m.group(1), 0)

    return bss_end, stack_limit, stack_top


def _gdb_runs(gdb: Path) -> bool:
    """True if gdb starts (filters out ARM toolchain gdb missing libncurses.so.5)."""
    try:
        proc = subprocess.run(
            [str(gdb), "--version"],
            capture_output=True,
            text=True,
            check=False,
            timeout=15,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    text = (proc.stdout or "") + (proc.stderr or "")
    if proc.returncode != 0:
        return False
    if "error while loading shared libraries" in text:
        return False
    return "gdb" in text.lower()


def find_gdb(explicit: Optional[Path] = None) -> Optional[Path]:
    """
    Pick a gdb that can load the firmware ELF and evaluate sizeof().

    Prefer a working binary: Arm's gdb often fails on Ubuntu 24.04+ (needs
    libncurses.so.5). Host `gdb-multiarch` / `gdb` are fine for DWARF sizeof.
    """
    candidates: List[Path] = []

    def add(path: Optional[Path]) -> None:
        if path is None:
            return
        path = Path(path)
        if path.exists() and path not in candidates:
            candidates.append(path)

    if explicit:
        # Honor --gdb strictly: caller asked for this binary.
        return Path(explicit) if Path(explicit).exists() else None

    env = os.environ.get("ARM_GDB") or os.environ.get("GDB")
    if env:
        add(Path(env))
    # Prefer host multiarch gdb on Linux (Arm toolchain gdb often needs
    # libncurses.so.5, removed on Ubuntu 24.04+).
    for name in ("gdb-multiarch", "arm-none-eabi-gdb", "gdb"):
        which = shutil.which(name)
        if which:
            add(Path(which))

    root = repo_root()
    toolchain_dir = os.environ.get("TOOLCHAIN_DIR")
    if toolchain_dir:
        td = Path(toolchain_dir)
        add(td / "bin" / "arm-none-eabi-gdb")
        add(td / "gcc-arm-none-eabi-14.2" / "bin" / "arm-none-eabi-gdb")
    add(root / "gcc-arm-none-eabi-14.2" / "bin" / "arm-none-eabi-gdb")
    add(root / "toolchain" / "14.2" / "bin" / "arm-none-eabi-gdb")
    add(root / "toolchain" / "14.2" / "gcc-arm-none-eabi-14.2" / "bin" / "arm-none-eabi-gdb")

    for candidate in candidates:
        if _gdb_runs(candidate):
            return candidate
    # Last resort: return first existing path so the probe error is informative.
    return candidates[0] if candidates else None


def strip_cpp_line_comment(line: str) -> str:
    in_str = False
    i = 0
    while i < len(line) - 1:
        ch = line[i]
        if ch == '"' and (i == 0 or line[i - 1] != "\\"):
            in_str = not in_str
        elif not in_str and ch == "/" and line[i + 1] == "/":
            return line[:i]
        i += 1
    return line


def discover_boot_ctors(root: Path) -> List[BootCtor]:
    """
    Collect permanent `new Type` / `new(AHB) Type` sites from boot sources.
    Skips commented-out lines and temporary maker/pool types.
    """
    found: List[BootCtor] = []
    seen = set()
    for rel in BOOT_CTOR_SOURCES:
        path = root / rel
        if not path.exists():
            continue
        for lineno, raw in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
            line = strip_cpp_line_comment(raw).strip()
            if not line or line.startswith("#"):
                continue
            for match in NEW_AHB_RE.finditer(line):
                type_name = match.group(1).split("::")[-1]
                if type_name in TRANSIENT_CTOR_TYPES:
                    continue
                key = ("ahb", type_name)
                if key in seen:
                    continue
                seen.add(key)
                found.append(BootCtor(type_name, "ahb", f"{rel}:{lineno}"))
            # Avoid double-counting `new(AHB) Foo` via the plain-new regex.
            line_no_ahb = NEW_AHB_RE.sub(" ", line)
            for match in NEW_MAIN_RE.finditer(line_no_ahb):
                type_name = match.group(1).split("::")[-1]
                if type_name in TRANSIENT_CTOR_TYPES:
                    continue
                # Skip placement-new helpers and obvious non-modules.
                if type_name in {"char", "int", "float", "uint8_t", "uint16_t", "uint32_t"}:
                    continue
                key = ("main", type_name)
                if key in seen:
                    continue
                seen.add(key)
                found.append(BootCtor(type_name, "main", f"{rel}:{lineno}"))

    return found


def spindle_type_name(cfg: Dict[str, str]) -> Optional[str]:
    if not as_bool(cfg.get("spindle.enable"), True):
        return None
    kind = cfg.get("spindle.type", CODE_DEFAULTS["spindle.type"]).strip().lower()
    vfd = cfg.get("spindle.vfd_type", "none").strip().lower()
    mapping = {
        "pwm": "PWMSpindleControl",
        "pid_pwm": "PIDPWMSpindleControl",
        "analog": "AnalogSpindleControl",
        "vesc": "VESCSpindleControl",
    }
    if kind == "modbus" and vfd == "huanyang":
        return "HuanyangSpindleControl"
    return mapping.get(kind)


def load_sizes_from_elf(elf: Path, gdb: Path, type_names: List[str]) -> Dict[str, int]:
    """Probe sizeof() for many types; missing DWARF types are skipped.

    Uses plain gdb `printf`/`p` commands (no Python scripting required — some
    arm-none-eabi-gdb builds ship without it).
    """
    if not type_names:
        return {}
    cmd = [str(gdb), "-batch", "-nx", "-ex", f"file {elf}"]
    for type_name in type_names:
        # Label then sizeof; missing types print "No symbol" and no $N value.
        cmd.extend(["-ex", f'printf "TRY {type_name}\\n"'])
        cmd.extend(["-ex", f"p/d sizeof({type_name})"])
    proc = subprocess.run(cmd, capture_output=True, text=True, check=False)

    sizes: Dict[str, int] = {}
    current: Optional[str] = None
    for line in (proc.stdout + "\n" + proc.stderr).splitlines():
        if line.startswith("TRY "):
            current = line[4:].strip()
            continue
        if current is None:
            continue
        if "No symbol" in line or "Attempt to use a type name" in line:
            current = None
            continue
        m = re.match(r"\$\d+\s*=\s*(\d+)\s*$", line.strip())
        if m:
            value = int(m.group(1))
            if current == "FATFileHandle" and value > 128:
                current = None
                continue
            sizes[current] = value
            current = None

    if not sizes:
        raise RuntimeError(f"gdb sizeof probe failed:\n{proc.stdout}\n{proc.stderr}")
    return sizes


def resolve_sizes(
    elf: Optional[Path],
    gdb_path: Optional[Path],
    extra_types: Iterable[str],
    *,
    require_dwarf: bool = False,
) -> Tuple[Dict[str, int], str]:
    sizes = dict(FALLBACK_SIZES)
    wanted = sorted(set(extra_types) | set(FALLBACK_SIZES) | set(AUX_SIZEOF_TYPES))
    if elf is None:
        if require_dwarf:
            raise RuntimeError("--elf was required but no ELF path is available")
        return sizes, "fallback (no --elf)"

    gdb = find_gdb(gdb_path)
    if gdb is None:
        msg = (
            "no usable gdb found (need gdb-multiarch, gdb, or a runnable "
            "arm-none-eabi-gdb for DWARF sizeof against --elf)"
        )
        if require_dwarf:
            raise RuntimeError(msg)
        return sizes, f"fallback ({msg})"

    if not _gdb_runs(gdb):
        msg = (
            f"{gdb} does not start (often missing libncurses.so.5 on Ubuntu 24.04+). "
            f"Install gdb-multiarch or pass --gdb /path/to/working/gdb"
        )
        if require_dwarf:
            raise RuntimeError(msg)
        return sizes, f"fallback ({msg})"

    try:
        probed = load_sizes_from_elf(elf, gdb, wanted)
        sizes.update(probed)
        # Without most boot types, the main-heap model is meaningless and can
        # false-pass. Require a reasonable hit rate when ELF probing is required.
        if require_dwarf and len(probed) < max(10, len(wanted) // 3):
            raise RuntimeError(
                f"DWARF sizeof probe too weak ({len(probed)}/{len(wanted)} types); "
                f"refusing to continue"
            )
        return sizes, f"DWARF via {gdb.name} ({len(probed)}/{len(wanted)} types)"
    except Exception as exc:  # noqa: BLE001
        if require_dwarf:
            raise
        return dict(FALLBACK_SIZES), f"fallback (gdb probe failed: {exc})"


def grid_dimensions(cfg: Dict[str, str]) -> Tuple[int, int]:
    size = as_int(
        cfg.get("leveling-strategy.rectangular-grid.size"),
        CODE_DEFAULTS["leveling-strategy.rectangular-grid.size"],
    )
    gx = as_int(cfg.get("leveling-strategy.rectangular-grid.grid_x_size"), size)
    gy = as_int(cfg.get("leveling-strategy.rectangular-grid.grid_y_size"), size)
    return gx, gy


def count_enabled_modules(cfg: Dict[str, str], family: str) -> int:
    """Count keys like switch.name.enable true / temperature_control.name.enable true."""
    suffix = ".enable"
    prefix = family + "."
    names = set()
    for key, value in cfg.items():
        if not key.startswith(prefix) or not key.endswith(suffix):
            continue
        if as_bool(value, False):
            names.add(key[len(prefix) : -len(suffix)])
    return len(names)


def _extract_braced_body(text: str, open_brace_index: int) -> Optional[str]:
    """Return the source inside a `{...}` starting at open_brace_index, brace-matched."""
    if open_brace_index < 0 or open_brace_index >= len(text) or text[open_brace_index] != "{":
        return None
    depth = 0
    for i in range(open_brace_index, len(text)):
        ch = text[i]
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return text[open_brace_index + 1 : i]
    return None


def _method_body(text: str, signature_re: str) -> Optional[str]:
    m = re.search(signature_re, text)
    if not m:
        return None
    # Signature match ends at `{`.
    return _extract_braced_body(text, m.end() - 1)


def _flex_compensation_is_deferred(root: Path) -> bool:
    """True when load_flex_compensation_data is not called from handleConfig."""
    path = root / "src" / "modules" / "tools" / "zprobe" / "CartGridStrategy.cpp"
    text = path.read_text(encoding="utf-8", errors="replace")
    body = _method_body(text, r"bool\s+CartGridStrategy::handleConfig\s*\([^)]*\)\s*\{")
    if body is None:
        return False
    return "load_flex_compensation_data" not in body


# Boot work that must run only after config_cache_clear() (main-heap safe).
# Add new entries here as more deferred loaders appear.
DEFERRED_LOAD_CHECKS = (
    ("flex_compensation", _flex_compensation_is_deferred),
)


def check_deferred_loads(root: Path) -> Dict[str, bool]:
    """Return {name: True} when that load is deferred past config_cache_clear()."""
    return {name: check(root) for name, check in DEFERRED_LOAD_CHECKS}


def estimate_ahb(
    cfg: Dict[str, str],
    pool_size: int,
    margin: int,
    sizes: Dict[str, int],
    boot_ctors: List[BootCtor],
) -> Tuple[List[AllocLine], bool]:
    lines: List[AllocLine] = []

    for ctor in boot_ctors:
        if ctor.region != "ahb":
            continue
        if ctor.type_name == "SerialConsole" and as_bool(cfg.get("disable_serial_console"), False):
            continue
        payload = sizes.get(ctor.type_name)
        if not payload:
            continue
        lines.append(
            AllocLine(
                ctor.type_name,
                payload,
                pool_cost(payload),
                note=f"new(AHB) @ {ctor.source}",
            )
        )

    queue = as_int(cfg.get("planner_queue_size"), CODE_DEFAULTS["planner_queue_size"])
    block_sz = sizes.get("Block", FALLBACK_SIZES["Block"])
    block_bytes = block_sz * queue
    lines.append(
        AllocLine(
            "BlockQueue",
            block_bytes,
            pool_cost(block_bytes),
            note=f"{queue} x Block({block_sz})",
        )
    )

    if as_bool(
        cfg.get("leveling-strategy.rectangular-grid.enable"),
        CODE_DEFAULTS["leveling-strategy.rectangular-grid.enable"],
    ):
        gx, gy = grid_dimensions(cfg)
        grid_bytes = gx * gy * 4
        lines.append(
            AllocLine("CartGrid", grid_bytes, pool_cost(grid_bytes), note=f"{gx}x{gy} floats")
        )
        flex_points = as_int(
            cfg.get("leveling-strategy.rectangular-grid.flex_x_points"),
            CODE_DEFAULTS["leveling-strategy.rectangular-grid.flex_x_points"],
        )
        flex_bytes = flex_points * 4
        lines.append(
            AllocLine(
                "FlexCompensation",
                flex_bytes,
                pool_cost(flex_bytes),
                note=f"{flex_points} floats (allocated even if inactive)",
            )
        )

    total = sum(line.cost for line in lines)
    return lines, (pool_size - total) >= margin


def estimate_main_heap(
    cfg: Dict[str, str],
    sizes: Dict[str, int],
    cache_live_usable: int,
    full_heap_gap: int,
    margin: int,
    always_active: bool,
    deferred_loads: Dict[str, bool],
    boot_ctors: List[BootCtor],
) -> Tuple[List[AllocLine], bool, List[str]]:
    lines: List[AllocLine] = []
    notes: List[str] = []
    skipped: List[str] = []

    for ctor in boot_ctors:
        if ctor.region != "main":
            continue
        if ctor.type_name == "MSCFileSystem" and not as_bool(cfg.get("usb_msc.enable"), True):
            continue
        payload = sizes.get(ctor.type_name)
        if not payload:
            skipped.append(ctor.type_name)
            continue
        lines.append(
            AllocLine(
                ctor.type_name,
                payload,
                malloc_cost(payload),
                note=f"new @ {ctor.source}",
            )
        )

    # Config-driven pools (count.enable keys → N instances).
    n_switch = count_enabled_modules(cfg, "switch")
    if n_switch and sizes.get("Switch"):
        per = sizes["Switch"] + sizes.get("Pin", 0) + sizes.get("Pwm", 0)
        lines.append(
            AllocLine(
                "Switches",
                per * n_switch,
                malloc_cost(per) * n_switch,
                note=f"{n_switch} enabled via switch.*.enable",
            )
        )

    n_temp = count_enabled_modules(cfg, "temperature_control")
    if n_temp and sizes.get("TemperatureControl"):
        per = sizes["TemperatureControl"] + sizes.get("Thermistor", 0)
        lines.append(
            AllocLine(
                "TemperatureControl",
                per * n_temp,
                malloc_cost(per) * n_temp,
                note=f"{n_temp} enabled via temperature_control.*.enable",
            )
        )

    if as_bool(
        cfg.get("leveling-strategy.rectangular-grid.enable"),
        CODE_DEFAULTS["leveling-strategy.rectangular-grid.enable"],
    ):
        payload = sizes.get("CartGridStrategy")
        if payload:
            lines.append(
                AllocLine(
                    "CartGridStrategy",
                    payload,
                    malloc_cost(payload),
                    note="leveling-strategy.rectangular-grid.enable",
                )
            )

    spindle = spindle_type_name(cfg)
    if spindle and sizes.get(spindle):
        lines.append(
            AllocLine(
                spindle,
                sizes[spindle],
                malloc_cost(sizes[spindle]),
                note=f"spindle.type={cfg.get('spindle.type', 'pwm')}",
            )
        )

    if skipped:
        notes.append(
            "skipped (not in ELF / excluded from build): " + ", ".join(sorted(set(skipped)))
        )

    lines.append(
        AllocLine(
            "UnaccountedBootHeap",
            BOOT_HEAP_UNACCOUNTED,
            BOOT_HEAP_UNACCOUNTED,
            note="vectors/strings/transient SD/allocator waste",
        )
    )

    flex_deferred = deferred_loads.get("flex_compensation", False)
    if always_active:
        fil = sizes.get("FIL_t", FALLBACK_SIZES["FIL_t"])
        file_sz = sizes.get("FILE", FALLBACK_SIZES["FILE"])
        handle = sizes.get("FATFileHandle", FALLBACK_SIZES["FATFileHandle"])
        peak = malloc_cost(fil) + malloc_cost(file_sz) + malloc_cost(handle) + FLEX_AUTOLOAD_SLACK
        lines.append(
            AllocLine(
                "FlexAutoloadPeak",
                peak,
                peak,
                note=f"FIL_t({fil})+FILE({file_sz})+handle+bind/printf",
            )
        )
        if flex_deferred:
            notes.append(
                "flex_compensation is deferred; still requiring autoload peak "
                "to fit in the cache-live window (boot-loop class)"
            )

    for name, is_deferred in deferred_loads.items():
        if not is_deferred:
            notes.append(
                f"FAIL structural: '{name}' must be deferred past config_cache_clear()"
            )

    total = sum(line.cost for line in lines)

    main_ok = total + margin <= cache_live_usable
    if not main_ok:
        notes.append(
            f"main heap {total}+margin {margin} = {total + margin} exceeds "
            f"cache-live usable {cache_live_usable} "
            f"(full gap after clear would be {full_heap_gap})"
        )
    if always_active and not flex_deferred:
        main_ok = False
    if any(not ok for ok in deferred_loads.values()):
        main_ok = False

    if always_active and flex_deferred and total + margin > full_heap_gap:
        main_ok = False
        notes.append(
            f"deferred autoload also exceeds full heap gap "
            f"({total}+{margin} > {full_heap_gap})"
        )

    return lines, main_ok, notes


def estimate_budget(
    label: str,
    config_path: Path,
    user_cfg: Dict[str, str],
    firm_cfg: Dict[str, str],
    firm_path: Path,
    machine: str,
    pool_size: int,
    margin: int,
    sizes: Dict[str, int],
    bss_end: int,
    stack_limit: int,
    deferred_loads: Dict[str, bool],
    boot_ctors: List[BootCtor],
) -> BudgetResult:
    cfg = merge_configs(
        {
            k: ("true" if v is True else "false" if v is False else str(v))
            for k, v in CODE_DEFAULTS.items()
        },
        firm_cfg,
        user_cfg,
    )

    cache_bytes = CONFIG_CACHE_CAPACITY * sizes["ConfigValue"]
    full_heap_gap = stack_limit - bss_end
    cache_live_usable = full_heap_gap - cache_bytes

    always_active = as_bool(
        cfg.get("leveling-strategy.rectangular-grid.flex_compensation_always_active"),
        False,
    )

    ahb_lines, ahb_ok = estimate_ahb(cfg, pool_size, margin, sizes, boot_ctors)
    main_lines, main_ok, notes = estimate_main_heap(
        cfg,
        sizes,
        cache_live_usable,
        full_heap_gap,
        margin,
        always_active,
        deferred_loads,
        boot_ctors,
    )

    return BudgetResult(
        label=label,
        config_path=config_path,
        machine=machine,
        firm_default=firm_path,
        pool_size=pool_size,
        margin=margin,
        ahb_lines=ahb_lines,
        main_lines=main_lines,
        cache_live_usable=cache_live_usable,
        full_heap_gap=full_heap_gap,
        always_active=always_active,
        deferred_loads=deferred_loads,
        ahb_ok=ahb_ok,
        main_ok=main_ok,
        notes=notes,
    )


def discover_configs(configs_dir: Path) -> List[Path]:
    paths = sorted(configs_dir.glob("*/config.txt"))
    if not paths:
        raise FileNotFoundError(f"No */config.txt under {configs_dir}")
    return paths


def print_section(title: str, lines: List[AllocLine], total: int, limit: int, margin: int, ok: bool) -> None:
    status = "PASS" if ok else "FAIL"
    print(f"  -- {title} [{status}] --")
    print(f"  {'item':<22} {'payload':>8} {'cost':>8}  note")
    print(f"  {'-'*22} {'-'*8} {'-'*8}  ----")
    for line in lines:
        print(f"  {line.name:<22} {line.payload:8d} {line.cost:8d}  {line.note}")
    free = limit - total
    print(f"  {'TOTAL':<22} {'':8} {total:8d}")
    print(f"  limit={limit}  free={free}  margin_required={margin}")


def print_result(result: BudgetResult, cache_bytes: int, bss_end: int, stack_limit: int) -> None:
    status = "PASS" if result.ok else "FAIL"
    print(f"\n=== {result.label} [{status}] ===")
    print(f"  config : {result.config_path}")
    print(f"  machine: {result.machine} (firm defaults: {result.firm_default.name})")
    print(
        f"  main RAM: bss_end={bss_end:#010x} stack_limit={stack_limit:#010x} "
        f"gap={result.full_heap_gap} cache={cache_bytes} "
        f"cache-live usable={result.cache_live_usable}"
    )
    deferred = [name for name, ok in result.deferred_loads.items() if ok]
    not_deferred = [name for name, ok in result.deferred_loads.items() if not ok]
    print(f"  flex_compensation_always_active={result.always_active}")
    print(f"  deferred loads: {', '.join(deferred) if deferred else '(none)'}")
    if not_deferred:
        print(f"  NOT deferred: {', '.join(not_deferred)}")
    print_section(
        "AHB permanent",
        result.ahb_lines,
        result.ahb_total,
        result.pool_size,
        result.margin,
        result.ahb_ok,
    )
    print_section(
        "Main heap vs config-cache window",
        result.main_lines,
        result.main_total,
        result.cache_live_usable,
        result.margin,
        result.main_ok,
    )
    for note in result.notes:
        print(f"  note: {note}")


def main(argv: Optional[Iterable[str]] = None) -> int:
    root = repo_root()
    parser = argparse.ArgumentParser(
        description="Check AHB + main-heap boot budgets for known configs"
    )
    parser.add_argument("--map", type=Path, default=root / "LPC1768" / "main.map")
    parser.add_argument("--elf", type=Path, default=None)
    parser.add_argument("--gdb", type=Path, default=None)
    parser.add_argument(
        "--configs-dir",
        type=Path,
        default=root / "tests" / "TEST_memory_budget" / "configs",
    )
    parser.add_argument(
        "--margin",
        type=int,
        default=512,
        help="Required free bytes on AHB and in the cache-live heap window",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)

    map_path = args.map if args.map.is_absolute() else root / args.map
    if not map_path.exists():
        print(f"ERROR: map file not found: {map_path}", file=sys.stderr)
        print("Build firmware first (./build/build.sh) so LPC1768/main.map exists.", file=sys.stderr)
        return 1

    elf_path = args.elf
    if elf_path is None:
        candidate = root / "LPC1768" / "main.elf"
        if candidate.exists():
            elf_path = candidate
    elif not elf_path.is_absolute():
        elf_path = root / elf_path

    configs_dir = args.configs_dir if args.configs_dir.is_absolute() else root / args.configs_dir
    if not configs_dir.is_dir():
        print(f"ERROR: configs dir not found: {configs_dir}", file=sys.stderr)
        return 1

    try:
        pool_size, dyn, end = parse_ahb_pool_size(map_path)
        bss_end, stack_limit, stack_top = parse_main_ram_layout(map_path)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    boot_ctors = discover_boot_ctors(root)
    ctor_types = [c.type_name for c in boot_ctors]
    # CI always passes --elf; require a real DWARF probe so we do not false-pass
    # on FALLBACK_SIZES alone.
    require_dwarf = elf_path is not None
    try:
        sizes, size_source = resolve_sizes(
            elf_path, args.gdb, ctor_types, require_dwarf=require_dwarf
        )
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    cache_bytes = CONFIG_CACHE_CAPACITY * sizes.get("ConfigValue", FALLBACK_SIZES["ConfigValue"])
    deferred_loads = check_deferred_loads(root)

    print("Boot memory budget check (AHB + main heap / config-cache)")
    print(f"  map   : {map_path}")
    print(f"  elf   : {elf_path if elf_path else '(none)'}")
    print(f"  sizes : {size_source}")
    print(
        f"  boot ctors: {sum(1 for c in boot_ctors if c.region=='ahb')} AHB + "
        f"{sum(1 for c in boot_ctors if c.region=='main')} main "
        f"(from {', '.join(BOOT_CTOR_SOURCES)})"
    )
    print(f"  AHB pool  : {pool_size} bytes  [{dyn:#010x} .. {end:#010x})")
    print(
        f"  main RAM  : bss_end={bss_end:#010x} stack_limit={stack_limit:#010x} "
        f"stack_top={stack_top:#010x}"
    )
    print(
        f"  config cache: {CONFIG_CACHE_CAPACITY} x ConfigValue({sizes.get('ConfigValue', FALLBACK_SIZES['ConfigValue'])}) "
        f"= {cache_bytes} bytes"
    )
    deferred_ok = [name for name, ok in deferred_loads.items() if ok]
    deferred_bad = [name for name, ok in deferred_loads.items() if not ok]
    print(f"  deferred loads: {', '.join(deferred_ok) if deferred_ok else '(none)'}")
    if deferred_bad:
        print(f"  NOT deferred: {', '.join(deferred_bad)}")
    print(f"  margin: {args.margin} bytes")

    results: List[BudgetResult] = []
    for config_path in discover_configs(configs_dir):
        label = config_path.parent.name
        user_cfg = parse_config_file(config_path)
        machine = detect_machine(user_cfg, config_path.parent)
        firm_path = firm_default_path(root, machine)
        if not firm_path.exists():
            print(f"ERROR: firm default missing: {firm_path}", file=sys.stderr)
            return 1
        firm_cfg = parse_config_file(firm_path)
        results.append(
            estimate_budget(
                label=label,
                config_path=config_path,
                user_cfg=user_cfg,
                firm_cfg=firm_cfg,
                firm_path=firm_path,
                machine=machine,
                pool_size=pool_size,
                margin=args.margin,
                sizes=sizes,
                bss_end=bss_end,
                stack_limit=stack_limit,
                deferred_loads=deferred_loads,
                boot_ctors=boot_ctors,
            )
        )

    for result in results:
        print_result(result, cache_bytes, bss_end, stack_limit)

    failed = [r for r in results if not r.ok]
    print()
    if failed:
        print(f"FAILED: {len(failed)}/{len(results)} config(s) exceed boot memory budget")
        return 1
    print(f"OK: {len(results)} config(s) fit boot memory budget")
    return 0


if __name__ == "__main__":
    sys.exit(main())
