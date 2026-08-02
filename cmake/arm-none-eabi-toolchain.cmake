set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-m3)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CARVERA_TOOLCHAIN_ROOT "" CACHE PATH "ARM GNU toolchain root; leave empty to search PATH")
set(_carvera_toolchain_root "${CARVERA_TOOLCHAIN_ROOT}")
if(_carvera_toolchain_root AND NOT EXISTS "${_carvera_toolchain_root}/bin")
    get_filename_component(_carvera_toolchain_dir_name "${_carvera_toolchain_root}" NAME)
    if(DEFINED ENV{TOOLCHAIN_DIR} AND
       EXISTS "$ENV{TOOLCHAIN_DIR}/${_carvera_toolchain_dir_name}/bin")
        set(_carvera_toolchain_root "$ENV{TOOLCHAIN_DIR}/${_carvera_toolchain_dir_name}")
    else()
        message(FATAL_ERROR
            "ARM GNU toolchain not found at '${CARVERA_TOOLCHAIN_ROOT}'. "
            "Run build/gcc.sh (or build/gcc.ps1) once, or set CARVERA_TOOLCHAIN_ROOT."
        )
    endif()
endif()

function(carvera_find_arm_tool output name)
    if(_carvera_toolchain_root)
        find_program(${output} "${name}" PATHS "${_carvera_toolchain_root}/bin" NO_DEFAULT_PATH REQUIRED)
    else()
        find_program(${output} "${name}" REQUIRED)
    endif()
    set(${output} "${${output}}" PARENT_SCOPE)
endfunction()

carvera_find_arm_tool(ARM_NONE_EABI_GCC arm-none-eabi-gcc)
carvera_find_arm_tool(ARM_NONE_EABI_GXX arm-none-eabi-g++)
carvera_find_arm_tool(ARM_NONE_EABI_AR arm-none-eabi-ar)
carvera_find_arm_tool(ARM_NONE_EABI_RANLIB arm-none-eabi-ranlib)
carvera_find_arm_tool(ARM_NONE_EABI_OBJCOPY arm-none-eabi-objcopy)
carvera_find_arm_tool(ARM_NONE_EABI_OBJDUMP arm-none-eabi-objdump)
carvera_find_arm_tool(ARM_NONE_EABI_SIZE arm-none-eabi-size)

set(CMAKE_C_COMPILER "${ARM_NONE_EABI_GCC}" CACHE FILEPATH "ARM C compiler" FORCE)
set(CMAKE_CXX_COMPILER "${ARM_NONE_EABI_GXX}" CACHE FILEPATH "ARM C++ compiler" FORCE)
set(CMAKE_ASM_COMPILER "${ARM_NONE_EABI_GCC}" CACHE FILEPATH "ARM assembler driver" FORCE)
set(CMAKE_AR "${ARM_NONE_EABI_AR}" CACHE FILEPATH "ARM archiver" FORCE)
set(CMAKE_RANLIB "${ARM_NONE_EABI_RANLIB}" CACHE FILEPATH "ARM archive indexer" FORCE)
set(CMAKE_OBJCOPY "${ARM_NONE_EABI_OBJCOPY}" CACHE FILEPATH "ARM objcopy" FORCE)
set(CMAKE_OBJDUMP "${ARM_NONE_EABI_OBJDUMP}" CACHE FILEPATH "ARM objdump" FORCE)
set(CMAKE_SIZE "${ARM_NONE_EABI_SIZE}" CACHE FILEPATH "ARM size tool" FORCE)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
