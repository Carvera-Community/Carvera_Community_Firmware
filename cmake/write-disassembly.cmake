foreach(_required OBJDUMP INPUT OUTPUT)
    if(NOT DEFINED ${_required} OR "${${_required}}" STREQUAL "")
        message(FATAL_ERROR "write-disassembly.cmake requires -D${_required}=<value>")
    endif()
endforeach()

execute_process(
    COMMAND "${OBJDUMP}" -d -f -M reg-names-std --demangle "${INPUT}"
    RESULT_VARIABLE _objdump_result
    OUTPUT_FILE "${OUTPUT}"
    ERROR_VARIABLE _objdump_error
)
if(_objdump_result)
    message(FATAL_ERROR "objdump failed (${_objdump_result}): ${_objdump_error}")
endif()
