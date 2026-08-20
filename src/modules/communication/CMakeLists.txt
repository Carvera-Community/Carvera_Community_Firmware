target_include_directories(carvera_sources PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_sources(carvera_sources PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/GcodeDispatch.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SerialConsole.cpp"
)
add_library(carvera_wireless_probe_sources INTERFACE)
target_sources(carvera_wireless_probe_sources INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/SerialConsole2.cpp"
)
if(REMOTE_SETTINGS_TRANSFER)
    target_sources(carvera_sources PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/RemoteTransfer.cpp")
endif()
if(SERIAL_RX_DMA)
    target_sources(carvera_sources PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/UartRxDma.cpp")
endif()

add_subdirectory(utils)
