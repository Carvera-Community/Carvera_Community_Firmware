# ---------- General settings ---------------------------------------
set target-charset ASCII
set print pretty on
set pagination off
set remotelogfile mri.log
set mem inaccessible-by-default off
set logging file debug.log
set logging enabled on

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


