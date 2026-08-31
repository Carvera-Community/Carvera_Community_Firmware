if(NOT DEFINED INPUT OR NOT EXISTS "${INPUT}")
    message(FATAL_ERROR "Firmware binary not found: ${INPUT}")
endif()
if(NOT DEFINED MAX_SIZE OR NOT MAX_SIZE MATCHES "^[0-9]+$")
    message(FATAL_ERROR "MAX_SIZE must be a non-negative byte count")
endif()
if(NOT DEFINED ELF OR NOT EXISTS "${ELF}")
    message(FATAL_ERROR "Firmware ELF not found: ${ELF}")
endif()
if(NOT DEFINED NM_TOOL OR NOT EXISTS "${NM_TOOL}")
    message(FATAL_ERROR "ARM symbol table tool not found: ${NM_TOOL}")
endif()

file(SIZE "${INPUT}" _firmware_size)
math(EXPR _remaining "${MAX_SIZE} - ${_firmware_size}")
if(_remaining LESS 0)
    math(EXPR _overage "0 - ${_remaining}")
    message(FATAL_ERROR
        "firmware.bin is ${_firmware_size} bytes, exceeding the ${MAX_SIZE}-byte "
        "LPC1768 application flash limit by ${_overage} byte(s) (16 KiB bootloader reserved)."
    )
endif()

function(_format_usage used total output_variable)
    math(EXPR _tenths "(${used} * 1000 + ${total} / 2) / ${total}")
    math(EXPR _whole "${_tenths} / 10")
    math(EXPR _fraction "${_tenths} % 10")
    set(${output_variable} "${used}/${total} bytes (${_whole}.${_fraction}%)" PARENT_SCOPE)
endfunction()

execute_process(
    COMMAND "${NM_TOOL}" -P "${ELF}"
    RESULT_VARIABLE _nm_result
    OUTPUT_VARIABLE _nm_output
    ERROR_VARIABLE _nm_error
)
if(_nm_result)
    message(FATAL_ERROR "Unable to inspect firmware linker symbols: ${_nm_error}")
endif()

function(_read_symbol_address symbol output_variable)
    string(REGEX MATCH "(^|\n)${symbol} [^ ]+ ([0-9a-fA-F]+)" _match "${_nm_output}")
    if(NOT _match)
        message(FATAL_ERROR "Linker symbol ${symbol} is missing from the firmware")
    endif()
    math(EXPR _address "0x${CMAKE_MATCH_2}")
    set(${output_variable} "${_address}" PARENT_SCOPE)
endfunction()

_read_symbol_address("__MainHeapStart" _main_heap_start)
_read_symbol_address("__MainHeapEnd" _main_heap_end)
_read_symbol_address("__GeneralAHBStart" _ahb_heap_start)
_read_symbol_address("__GeneralAHBEnd" _ahb_heap_end)

math(EXPR _heap_capacity
    "${_main_heap_end} - ${_main_heap_start} + ${_ahb_heap_end} - ${_ahb_heap_start}")
set(_sram_capacity 65536)
math(EXPR _reserved_sram "${_sram_capacity} - ${_heap_capacity}")

_format_usage(${_firmware_size} ${MAX_SIZE} _flash_summary)
_format_usage(${_reserved_sram} ${_sram_capacity} _reserved_sram_summary)
_format_usage(${_heap_capacity} ${_sram_capacity} _heap_summary)

message(STATUS "Flash: ${_flash_summary}")
message(STATUS "SRAM reserved by firmware and stack: ${_reserved_sram_summary}")
message(STATUS "SRAM available to heap: ${_heap_summary}")
