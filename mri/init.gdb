# ---------- General settings ---------------------------------------
set target-charset ASCII
set print pretty on
set pagination off
set remotelogfile mri.log
set mem inaccessible-by-default off
set logging file debug.log
set logging enabled on

# ---------- MemoryPool::alloc trace (non‑blocking) ----------

break memorypool_alloc_return_point
commands
    silent
    printf "[ALLOC] %6lu B  -> %p\n", $r1, $r0
    p *_ahb
    bt
    printf "\n\n"
    x/24xw 0x20080f74
    continue
end
disable $bpnum
set $bp_alloc = $bpnum

# ---------- MemoryPool::dealloc trace (disabled by default) --------
break memorypool_free_hook
commands
    silent
    printf "[FREE ] %6lu B  <- %p\n", $r1, $r0
    p *_ahb
    bt
    printf "\n\n"
    x/24xw 0x20080f74
    continue
end
disable $bpnum
set $bp_free = $bpnum

# ---------- MemoryPool debug values (disabled by default) --------
break memorypool_debug_hook
commands
    silent
    printf "[DEBUG] offset(p)=%lu, p->next=%lu, q_next->next=%lu, size=%lu\n", $r0, $r1, $r2, $r3
    p *_ahb
    bt
    printf "\n\n"
    x/24xw 0x20080f74
    continue
end
disable $bpnum
set $bp_debug = $bpnum

# ---------- Toggle trace helpers -----------------------------------
define enable-pool-trace
    enable $bp_alloc $bp_free $bp_debug
    echo MemoryPool trace ENABLED\n
end

define disable-pool-trace
    disable $bp_alloc $bp_free $bp_debug
    echo MemoryPool trace DISABLED\n
end

# ---------- Crash‑dump helpers -------------------------------------
define smoothie-full-dump
    echo \n===== FULL DUMP =====\n
    bt
    info registers
    list
    disassemble
    echo \n--- first 32kB of SRAM ---\n
    set $ptr = 0x10000000
    while $ptr < 0x10008000
        x/4wa $ptr
        set $ptr += 16
    end
    echo ===== END DUMP =====\n
    set pagination on
end

define smoothie-mini-dump
    echo \n===== MINI DUMP =====\n
    bt
    info registers
    list
    echo \n--- stack until AHB top ---\n
    set $ptr = $sp
    while $ptr < 0x10008000
        x/4wa $ptr
        set $ptr += 16
    end
    echo ===== END MINI DUMP =====\n
end

# ---------- Quick chip‑reset command -------------------------------
define reset
    # NVIC System Reset via SCB->AIRCR (0xE000ED0C)
    set {uint32_t}0xE000ED0C = 0x05FA0004
end

# ---------- Extra Cortex‑M helpers ---------------------------------
define fault-info
    printf "SCB->HFSR  = 0x%08x\n", *((unsigned int *)0xE000ED2C)
    printf "SCB->CFSR  = 0x%08x\n", *((unsigned int *)0xE000ED28)
    printf "SCB->BFAR  = 0x%08x\n", *((unsigned int *)0xE000ED38)
    printf "SCB->MMFAR = 0x%08x\n", *((unsigned int *)0xE000ED34)
end

define hardfault-break
    break HardFault_Handler
    echo Breakpoint set at HardFault_Handler.\n
end



