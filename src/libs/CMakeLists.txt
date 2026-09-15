target_include_directories(carvera_sources PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_sources(carvera_sources PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/Adc.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/AppendFileStream.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Config.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/ConfigCache.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/ConfigSource.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/ConfigValue.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/FactorySettings.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/FirmwareFileSystem.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/gpio.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Hook.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Kernel.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/MakeraControl.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/MakeraFrame.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/md5.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Module.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/MRI_Hooks.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Pin.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/PublicData.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Pwm.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SDFAT.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SlowTicker.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SoftPWM.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/StepperMotor.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/StepTicker.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/StreamOutput.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Watchdog.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/utils.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/heap/heap_5.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/heap/heap_api.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/heap/heap_debug.cpp"
)

add_subdirectory(ADC)
add_subdirectory(ChaNFS)
add_subdirectory(ConfigSources)
add_subdirectory(LPC17xx)
add_subdirectory(USBDevice)
