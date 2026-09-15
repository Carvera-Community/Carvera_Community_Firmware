add_library(carvera_sources OBJECT
    main.cpp
    version.cpp
)
target_include_directories(carvera_sources PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_SOURCE_DIR}/build"
    "${CMAKE_SOURCE_DIR}/mri/core"
)

set(_firmware_definitions
    __LPC17XX__
    CNC
    CARTESIAN_ONLY
    CARTGRID_ONLY
    NO_PID_AUTOTUNE
    NO_TOOLS_SCARACAL
    NO_TOOLS_ROTARYDELTACALIBRATION
    CHECKSUM_USE_CPP
    DEFAULT_SERIAL_BAUD_RATE=${DEFAULT_SERIAL_BAUD_RATE}
    WRITE_BUFFER_DISABLE=0
    STACK_SIZE=${STACK_SIZE}
    "__GITVERSIONSTRING__=\"${CARVERA_FIRMWARE_VERSION}\""
)
if(MACHINE STREQUAL "z1")
    list(APPEND _firmware_definitions MACHINE_FAMILY_Z1)
else()
    list(APPEND _firmware_definitions MACHINE_FAMILY_CARVERA)
endif()
foreach(_optional_definition
        SERIAL_RX_DMA
        REMOTE_SETTINGS_TRANSFER
        STREAMED_JOB_PLAYBACK
        CANOPEN_SUPPORT)
    if(${_optional_definition})
        list(APPEND _firmware_definitions ${_optional_definition})
    endif()
endforeach()
if(AXIS)
    list(APPEND _firmware_definitions MAX_ROBOT_ACTUATORS=${AXIS})
endif()
if(PAXIS)
    list(APPEND _firmware_definitions N_PRIMARY_AXIS=${PAXIS})
endif()
if(DISABLEMSD)
    list(APPEND _firmware_definitions DISABLEMSD)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(_mri_enable 1)
    set(_mri_semihost_stdio 1)
else()
    set(_mri_enable 0)
    set(_mri_semihost_stdio 0)
endif()
target_compile_definitions(carvera_sources PUBLIC
    ${_firmware_definitions}
    MRI_ENABLE=${_mri_enable}
    "MRI_INIT_PARAMETERS=\"MRI_UART_2 MRI_UART_BAUD=${DEFAULT_SERIAL_BAUD_RATE}\""
    MRI_BREAK_ON_INIT=0
    MRI_SEMIHOST_STDIO=${_mri_semihost_stdio}
)
target_link_libraries(carvera_sources PUBLIC
    carvera_project_options
    carvera_mbed_headers
)

set(_firmware_compile_options
    -O${OPTIMIZATION}
    -flto
    -flto-partition=max
    -fomit-frame-pointer
)
if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 10.3)
    list(APPEND _firmware_compile_options
        -fanalyzer
        -floop-unroll-and-jam
        -floop-interchange
        -mfix-cortex-m3-ldrd
    )
endif()
target_compile_options(carvera_sources PUBLIC
    "$<$<COMPILE_LANGUAGE:C,CXX>:${_firmware_compile_options}>"
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-rtti;-std=gnu++17>"
)

add_subdirectory(libs)
add_subdirectory(modules)

if(NO_MODBUS_SPINDLE)
    target_compile_definitions(carvera_sources PUBLIC NO_MODBUS_SPINDLE)
else()
    target_link_libraries(carvera_sources PRIVATE carvera_spindle_modbus_sources)
endif()

if(NO_VESC_SPINDLE)
    target_compile_definitions(carvera_sources PUBLIC NO_VESC_SPINDLE)
else()
    target_link_libraries(carvera_sources PRIVATE carvera_spindle_vesc_sources)
endif()

if(NO_USB_HOST)
    target_compile_definitions(carvera_sources PUBLIC NO_USB_HOST)
else()
    target_link_libraries(carvera_sources PRIVATE carvera_usb_host_sources)
endif()

if(NO_SD_CARD)
    target_compile_definitions(carvera_sources PUBLIC NO_SD_CARD)
endif()

if(NO_WIRELESS_PROBE)
    target_compile_definitions(carvera_sources PUBLIC NO_WIRELESS_PROBE)
else()
    target_link_libraries(carvera_sources PRIVATE carvera_wireless_probe_sources)
endif()

if(NO_WIFI_PROVIDER)
    target_compile_definitions(carvera_sources PUBLIC NO_WIFI_PROVIDER)
else()
    target_link_libraries(carvera_sources PRIVATE carvera_wifi_sources)
endif()
