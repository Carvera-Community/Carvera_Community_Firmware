set(_mri_root "${CMAKE_CURRENT_SOURCE_DIR}")
add_library(carvera_mri STATIC
    architectures/armv7-m/armv7-m.c
    architectures/armv7-m/armv7-m_asm.S
    boards/mbed1768/mbed1768.c
    boards/mbed1768/mbed1768_asm.S
    core/buffer.c
    core/cmd_break_watch.c
    core/cmd_common.c
    core/cmd_continue.c
    core/cmd_file.c
    core/cmd_memory.c
    core/cmd_query.c
    core/cmd_registers.c
    core/cmd_step.c
    core/cmd_thread.c
    core/cmd_vcont.c
    core/context.c
    core/gdb_console.c
    core/libc.c
    core/memory.c
    core/mri.c
    core/packet.c
    core/token.c
    core/try_catch.c
    devices/lpc176x/lpc176x_asm.S
    devices/lpc176x/lpc176x_init.c
    devices/lpc176x/lpc176x_uart.c
    memory/native/native-mem.c
    rtos/rtos_weak.c
    semihost/arm/semihost_arm.c
    semihost/newlib/newlib_stubs.S
    semihost/newlib/semihost_newlib.c
    semihost/semihost.c
)
set_target_properties(carvera_mri PROPERTIES OUTPUT_NAME mri_mbed1768)
target_include_directories(carvera_mri PRIVATE
    "${_mri_root}"
    "${_mri_root}/cmsis"
    "${_mri_root}/cmsis/LPC17xx"
)
target_include_directories(carvera_mri INTERFACE "${_mri_root}/core")
target_link_libraries(carvera_mri PRIVATE carvera_embedded_options)
target_compile_options(carvera_mri PRIVATE
    "$<$<COMPILE_LANGUAGE:C,CXX>:-Os;-Werror;-fomit-frame-pointer;-std=gnu90>"
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>"
)
target_compile_definitions(carvera_mri PRIVATE
    MRI_THREAD_MRI=0
    MRI_ALWAYS_USE_HARDWARE_BREAKPOINT=0
    MRI_UINT_TYPE=uint32_t
    MRI_INT_TYPE=int32_t
    MRI_DEVICE_HAS_FPU=0
)
