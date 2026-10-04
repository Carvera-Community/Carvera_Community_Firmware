set(_mbed_root "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(_mbed_vendor "${_mbed_root}/vendor/NXP")
set(_mbed_device "LPC1768")

add_library(carvera_mbed STATIC
    src/capi/exit.c
    src/capi/mbed_interface.c
    src/capi/pinmap_common.c
    src/capi/rtc_time.c
    src/capi/semihost_api.c
    src/capi/wait_api.c
    src/cpp/FileBase.cpp
    src/cpp/FileLike.cpp
    src/cpp/FilePath.cpp
    src/cpp/FileSystemLike.cpp
    src/cpp/FunctionPointer.cpp
    src/cpp/I2C.cpp
    src/cpp/InterruptIn.cpp
    src/cpp/Serial.cpp
    src/cpp/SPI.cpp
    src/cpp/stdio.cpp
    src/cpp/Stream.cpp
    src/cpp/Ticker.cpp
    src/cpp/Timeout.cpp
    src/cpp/Timer.cpp
    src/cpp/TimerEvent.cpp
    src/cpp/CAN.cpp
    src/vendor/NXP/capi/board.c
    src/vendor/NXP/capi/can_api.c
    src/vendor/NXP/capi/gpio_api.c
    src/vendor/NXP/capi/gpio_irq_api.c
    src/vendor/NXP/capi/i2c_api.c
    src/vendor/NXP/capi/pinmap.c
    src/vendor/NXP/capi/port_api.c
    src/vendor/NXP/capi/pwmout_api.c
    src/vendor/NXP/capi/rtc_api.c
    src/vendor/NXP/capi/serial_api.c
    src/vendor/NXP/capi/spi_api.c
    src/vendor/NXP/capi/us_ticker_api.c
    src/vendor/NXP/cmsis/LPC1768/cmsis_nvic.c
    src/vendor/NXP/cmsis/LPC1768/GCC_ARM/startup_LPC17xx.s
    src/vendor/NXP/cmsis/LPC1768/system_LPC17xx.c
    "$<$<CONFIG:Debug>:${CMAKE_CURRENT_SOURCE_DIR}/src/cpp/LocalFileSystem.cpp>"
    "$<$<CONFIG:Release>:${CMAKE_CURRENT_SOURCE_DIR}/src/vendor/NXP/capi/semihost.c>"
)
set_target_properties(carvera_mbed PROPERTIES OUTPUT_NAME mbed)

add_library(carvera_mbed_headers INTERFACE)
target_include_directories(carvera_mbed_headers INTERFACE
    "${_mbed_root}/capi"
    "${_mbed_root}/cpp"
    "${_mbed_vendor}/capi"
    "${_mbed_vendor}/capi/${_mbed_device}"
    "${_mbed_vendor}/cmsis/${_mbed_device}"
)

target_link_libraries(carvera_mbed
    PUBLIC carvera_mbed_headers
    PRIVATE carvera_project_options
)
target_compile_options(carvera_mbed PRIVATE
    "$<$<COMPILE_LANGUAGE:C>:-std=c99>"
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>"
    "$<$<CONFIG:Release>:-O2>"
    "$<$<CONFIG:Debug>:-O0>"
)
target_compile_definitions(carvera_mbed PRIVATE
    TARGET_LPC1768
    TOOLCHAIN_GCC_ARM
    __STACK_SIZE=${STACK_SIZE}
)
