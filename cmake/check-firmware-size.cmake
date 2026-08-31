if(NOT DEFINED INPUT OR NOT EXISTS "${INPUT}")
    message(FATAL_ERROR "Firmware binary not found: ${INPUT}")
endif()
if(NOT DEFINED MAX_SIZE OR NOT MAX_SIZE MATCHES "^[0-9]+$")
    message(FATAL_ERROR "MAX_SIZE must be a non-negative byte count")
endif()
if(NOT DEFINED ELF OR NOT EXISTS "${ELF}")
    message(FATAL_ERROR "Firmware ELF not found: ${ELF}")
endif()
if(NOT DEFINED SIZE_TOOL OR NOT EXISTS "${SIZE_TOOL}")
    message(FATAL_ERROR "ARM size tool not found: ${SIZE_TOOL}")
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

execute_process(
    COMMAND "${SIZE_TOOL}" -A -d "${ELF}"
    RESULT_VARIABLE _size_result
    OUTPUT_VARIABLE _size_output
    ERROR_VARIABLE _size_error
)
if(_size_result)
    message(FATAL_ERROR "Unable to inspect firmware memory usage: ${_size_error}")
endif()

function(_read_section_size section_regex section_name output_variable)
    string(REGEX MATCH "(^|\n)${section_regex}[ \t]+([0-9]+)" _match "${_size_output}")
    if(NOT _match)
        if(ARGC EQUAL 4)
            set(${output_variable} "${ARGV3}" PARENT_SCOPE)
            return()
        endif()
        message(FATAL_ERROR "Section ${section_name} is missing from the firmware size report")
    endif()
    set(${output_variable} "${CMAKE_MATCH_2}" PARENT_SCOPE)
endfunction()

function(_format_usage used total output_variable)
    math(EXPR _tenths "(${used} * 1000 + ${total} / 2) / ${total}")
    math(EXPR _whole "${_tenths} / 10")
    math(EXPR _fraction "${_tenths} % 10")
    set(${output_variable} "${used}/${total} bytes (${_whole}.${_fraction}%)" PARENT_SCOPE)
endfunction()

_read_section_size("\\.data" ".data" _data_size)
_read_section_size("\\.bss" ".bss" _bss_size)
_read_section_size("\\.stack_dummy" ".stack_dummy" _stack_size 0)
_read_section_size("\\.AHBSRAM" ".AHBSRAM" _ahb_ram_used)

math(EXPR _regular_ram_used "${_data_size} + ${_bss_size} + ${_stack_size}")
set(_ram_capacity 32768)
set(_ahb_ram_capacity 32768)

_format_usage(${_firmware_size} ${MAX_SIZE} _flash_summary)
_format_usage(${_regular_ram_used} ${_ram_capacity} _ram_summary)
_format_usage(${_ahb_ram_used} ${_ahb_ram_capacity} _ahb_ram_summary)

message(STATUS "Flash: ${_flash_summary}")
message(STATUS "Static RAM: ${_ram_summary}")
message(STATUS "Static AHB RAM: ${_ahb_ram_summary}")
