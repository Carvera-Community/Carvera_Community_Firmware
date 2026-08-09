# TEST_memory_budget

Host-side check that known SD configs can boot without exhausting:

1. **AHB MemoryPool** — permanent `new(AHB)` / grid / flex buffer / planner queue
2. **Main heap vs config-cache window** — especially
   `flex_compensation_always_active`, which opens
   `/sd/flex_compensation.dat` (`FIL_t` ≈ 548B sector buffer on the heap) and
   installs a `std::function` bind. Heap growth into the live config cache
   hard-resets (boot loop).

## Layout

```text
configs/
  1/   CA1 + flex_compensation_always_active
  2/   stock C1
```

Each fixture directory should contain:

- `config.txt` — SD user config overlay
- `README.md` — short description (also used as a machine-type hint)
- optional `flex_compensation.dat` — for on-device repro only

## Run

After a firmware build (needs `LPC1768/main.map`, ideally also `main.elf`):

```bash
./build/check-ahb-budget.py \
  --map LPC1768/main.map \
  --elf LPC1768/main.elf \
  --configs-dir tests/TEST_memory_budget/configs
```

Exit status `0` means every fixture fits both budgets with the default 512-byte
margin.

## Notes

- Flex buffer AHB cost is paid whenever rectangular-grid is enabled;
  `flex_compensation_always_active` adds the **main-heap autoload peak**
  (`FIL_t` ≈ 548B + `FILE` + handle + `std::function` / printf scratch).
- Production defers that load until after `config_cache_clear()`. The checker
  keeps a `DEFERRED_LOAD_CHECKS` list (currently just `flex_compensation`) and
  still requires the autoload peak to fit in the cache-live window — that is
  the boot-loop failure class (`STACK_SIZE=0` allows the heap to enter the
  cache region before clear detects it).
- Permanent modules are discovered from `new` / `new(AHB)` sites in
  `Kernel.cpp`, `main.cpp`, and `Config.cpp`. Types not present in the ELF
  (makefile-excluded) are skipped. Config-driven pools
  (`switch.*.enable`, `temperature_control.*.enable`, spindle type, cart grid)
  are counted from the merged config.
- `BOOT_HEAP_UNACCOUNTED` covers unmodeled boot heap (strings, vectors,
  transient SD opens, etc.). Refine with on-device `mem -v`.
