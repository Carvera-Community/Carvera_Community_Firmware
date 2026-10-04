cmake_minimum_required(VERSION 3.20)

project(CarveraCommunityFirmware LANGUAGES C CXX ASM)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "In-source builds are not supported. Configure with cmake -S . -B <build-directory>.")
endif()

if(CMAKE_CONFIGURATION_TYPES)
    message(FATAL_ERROR "A single-config generator is required. Use Ninja or Unix Makefiles and set CMAKE_BUILD_TYPE.")
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Firmware build profile" FORCE)
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Release Debug)
if(NOT CMAKE_BUILD_TYPE STREQUAL "Release" AND NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(FATAL_ERROR "Unsupported CMAKE_BUILD_TYPE '${CMAKE_BUILD_TYPE}'. Choose Release or Debug.")
endif()

set(VERSION "" CACHE STRING "Firmware version string; derived from Git when empty")
set(MACHINE "carvera" CACHE STRING "Target machine family")
set_property(CACHE MACHINE PROPERTY STRINGS carvera z1)
set(AXIS "5" CACHE STRING "Maximum number of robot actuators")
set(PAXIS "3" CACHE STRING "Number of primary axes")
option(DISABLEMSD "Disable USB mass storage support" OFF)
option(SERIAL_RX_DMA "Receive the primary UART through DMA" OFF)
option(REMOTE_SETTINGS_TRANSFER "Load machine settings through the Makera serial link" OFF)
option(STREAMED_JOB_PLAYBACK "Play jobs streamed through the Makera serial link" OFF)
option(CANOPEN_SUPPORT "Enable CAN and CANopen support" OFF)
option(NO_MODBUS_SPINDLE "Exclude the Modbus spindle drivers" ON)
option(NO_VESC_SPINDLE "Exclude the VESC USB spindle driver" OFF)
option(NO_USB_HOST "Exclude USB host support" OFF)
option(NO_SD_CARD "Exclude local SD card support" OFF)
option(NO_WIRELESS_PROBE "Exclude the wireless probe UART" OFF)
option(NO_WIFI_PROVIDER "Exclude the Wi-Fi module" OFF)
set(STACK_SIZE "4096" CACHE STRING "Reserved stack size in bytes; use 0 to disable the limit")
if(MACHINE STREQUAL "z1")
    set(REMOTE_SETTINGS_TRANSFER ON)
    set(STREAMED_JOB_PLAYBACK ON)
    set(CANOPEN_SUPPORT ON)
    set(SERIAL_RX_DMA ON)
    set(NO_USB_HOST ON)
    set(NO_VESC_SPINDLE ON)
    set(NO_SD_CARD ON)
    set(NO_WIRELESS_PROBE ON)
    set(NO_WIFI_PROVIDER ON)
elseif(NOT MACHINE STREQUAL "carvera")
    message(FATAL_ERROR "Unsupported MACHINE '${MACHINE}'; choose carvera or z1.")
endif()
if(NO_USB_HOST)
    set(NO_VESC_SPINDLE ON)
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(_default_optimization "g")
else()
    set(_default_optimization "s")
endif()
set(OPTIMIZATION "${_default_optimization}" CACHE STRING "GCC optimization level without the -O prefix")
set(DEFAULT_SERIAL_BAUD_RATE "115200" CACHE STRING "Default serial baud rate")

if(VERSION)
    set(CARVERA_FIRMWARE_VERSION "${VERSION}")
else()
    execute_process(
        COMMAND git symbolic-ref --short HEAD
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        RESULT_VARIABLE _branch_result
        OUTPUT_VARIABLE _git_branch
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if(_branch_result OR NOT _git_branch)
        set(_git_branch "2.0.0c")
    endif()
    execute_process(
        COMMAND git rev-parse --short=7 HEAD
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        RESULT_VARIABLE _commit_result
        OUTPUT_VARIABLE _git_commit
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if(_commit_result OR NOT _git_commit)
        message(FATAL_ERROR "Unable to derive the firmware version from Git. Configure with -DVERSION=<value>.")
    endif()
    set(CARVERA_FIRMWARE_VERSION "${_git_branch}-${_git_commit}")
endif()

message(STATUS "Firmware profile: ${CMAKE_BUILD_TYPE}")
message(STATUS "Firmware version: ${CARVERA_FIRMWARE_VERSION}")
message(STATUS "Machine configuration: MACHINE=${MACHINE} AXIS=${AXIS} PAXIS=${PAXIS}")

include(cmake/compiler-options.cmake)

add_subdirectory(mbed)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_subdirectory(mri)
endif()
add_subdirectory(src)

set(_generated_dir "${CMAKE_BINARY_DIR}/generated-config")
if(WIN32)
    set(_minify_script "${CMAKE_SOURCE_DIR}/build/minify-config.ps1")
    set(_minify_command powershell -ExecutionPolicy Bypass -File "${_minify_script}")
else()
    set(_minify_script "${CMAKE_SOURCE_DIR}/build/minify-config.sh")
    set(_minify_command "${_minify_script}")
endif()

function(carvera_embed_config input_name object_name section_name output_variable)
    set(_object "${_generated_dir}/${object_name}.o")
    add_custom_command(
        OUTPUT "${_object}"
        COMMAND "${CMAKE_COMMAND}" -E make_directory "${_generated_dir}"
        COMMAND ${_minify_command} "${CMAKE_SOURCE_DIR}/src/${input_name}" "${_generated_dir}/${input_name}"
        COMMAND "${CMAKE_COMMAND}" -E chdir "${_generated_dir}"
                "${CMAKE_OBJCOPY}" -I binary -O elf32-littlearm -B arm --readonly-text
                "--rename-section=.data=${section_name}" "${input_name}" "${object_name}.o"
        DEPENDS "${CMAKE_SOURCE_DIR}/src/${input_name}" "${_minify_script}"
        VERBATIM
    )
    set(${output_variable} "${_object}" PARENT_SCOPE)
endfunction()
carvera_embed_config(config.default configdefault .rodata.configdefault _configdefault_o)
carvera_embed_config(config2.default config2default .rodata.config2default _config2default_o)
set(_embedded_configs "${_configdefault_o}" "${_config2default_o}")
if(MACHINE STREQUAL "z1")
    carvera_embed_config(config_z1.default configz1default .rodata.configz1default _configz1default_o)
    carvera_embed_config(config_z1pro.default configz1prodefault .rodata.configz1prodefault _configz1prodefault_o)
    list(APPEND _embedded_configs "${_configz1default_o}" "${_configz1prodefault_o}")
endif()
set_source_files_properties(${_embedded_configs} PROPERTIES EXTERNAL_OBJECT TRUE GENERATED TRUE)

add_executable(firmware
    "${CMAKE_SOURCE_DIR}/build/mbed_custom.cpp"
    ${_embedded_configs}
)
if(MACHINE STREQUAL "z1")
    set(_firmware_output_dir "${CMAKE_BINARY_DIR}/LPC1768-z1")
else()
    set(_firmware_output_dir "${CMAKE_BINARY_DIR}/LPC1768")
endif()
set_target_properties(firmware PROPERTIES
    OUTPUT_NAME firmware
    SUFFIX .elf
    RUNTIME_OUTPUT_DIRECTORY "${_firmware_output_dir}"
)

set(_firmware_link_map "${CMAKE_BINARY_DIR}/CMakeFiles/firmware.link.map")
set(_firmware_linker_script
    "${CMAKE_SOURCE_DIR}/mbed/src/vendor/NXP/cmsis/LPC1768/GCC_ARM/LPC1768.ld"
)
set(_link_lto -flto=auto)
if(WIN32)
    set(_link_lto -flto)
endif()
set(_wraps
    --wrap=_isatty
    --wrap=malloc
    --wrap=calloc
    --wrap=realloc
    --wrap=free
    --wrap=_malloc_r
    --wrap=_calloc_r
    --wrap=_realloc_r
    --wrap=_free_r
)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    list(APPEND _wraps --wrap=_read --wrap=_write --wrap=semihost_connected)
endif()
set(_wrap_options)
foreach(_wrap IN LISTS _wraps)
    list(APPEND _wrap_options "-Wl,${_wrap}")
endforeach()
target_link_options(firmware PRIVATE
    -mcpu=cortex-m3
    -mthumb
    "-specs=${CMAKE_SOURCE_DIR}/build/startfile.spec"
    "${_link_lto}"
    -flto-partition=max
    "-Wl,-Map=${_firmware_link_map},--cref,--gc-sections"
    ${_wrap_options}
    "-T${_firmware_linker_script}"
    -specs=nano.specs
    "LINKER:--undefined=_scanf_float"
    "LINKER:--undefined=_printf_float"
)
add_custom_command(TARGET firmware POST_BUILD
    BYPRODUCTS "${_firmware_link_map}"
    COMMAND "${CMAKE_COMMAND}" -E true
)
target_link_libraries(firmware PRIVATE
    carvera_sources
    "$<$<CONFIG:Debug>:carvera_mri>"
    carvera_mbed
    stdc++
    supc++
    m
    gcc
    c
    nosys
)
set_property(TARGET firmware APPEND PROPERTY LINK_DEPENDS
    "${_firmware_linker_script}"
    "${CMAKE_SOURCE_DIR}/build/startfile.spec"
)

set(_firmware_bin "${_firmware_output_dir}/firmware.bin")
set(_firmware_hex "${_firmware_output_dir}/firmware.hex")
set(_firmware_map "${_firmware_output_dir}/firmware.map")
set(_firmware_disasm "${_firmware_output_dir}/firmware.disasm")
math(EXPR _firmware_max_size "512 * 1024 - 16 * 1024")
add_custom_command(
    OUTPUT "${_firmware_map}"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${_firmware_link_map}" "${_firmware_map}"
    DEPENDS firmware
    VERBATIM
)
add_custom_command(
    OUTPUT "${_firmware_bin}" "${_firmware_hex}" "${_firmware_disasm}"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${_firmware_output_dir}"
    COMMAND "${CMAKE_OBJCOPY}" -O binary "$<TARGET_FILE:firmware>" "${_firmware_bin}"
    COMMAND "${CMAKE_COMMAND}"
            "-DINPUT=${_firmware_bin}"
            "-DELF=$<TARGET_FILE:firmware>"
            "-DNM_TOOL=${CMAKE_NM}"
            "-DMAX_SIZE=${_firmware_max_size}"
            -P "${CMAKE_SOURCE_DIR}/cmake/check-firmware-size.cmake"
    COMMAND "${CMAKE_OBJCOPY}" -R .stack -O ihex "$<TARGET_FILE:firmware>" "${_firmware_hex}"
    COMMAND "${CMAKE_COMMAND}"
            "-DOBJDUMP=${CMAKE_OBJDUMP}"
            "-DINPUT=$<TARGET_FILE:firmware>"
            "-DOUTPUT=${_firmware_disasm}"
            -P "${CMAKE_SOURCE_DIR}/cmake/write-disassembly.cmake"
    DEPENDS firmware "${CMAKE_SOURCE_DIR}/cmake/check-firmware-size.cmake"
    VERBATIM
)
add_custom_target(firmware_artifacts ALL
    DEPENDS "${_firmware_bin}" "${_firmware_hex}" "${_firmware_map}" "${_firmware_disasm}"
)
