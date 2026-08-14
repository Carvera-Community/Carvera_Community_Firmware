target_include_directories(carvera_sources PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_sources(carvera_sources PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/GcodeDispatch.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SerialConsole.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SerialConsole2.cpp"
)
if(SERIAL_RX_DMA)
    target_sources(carvera_sources PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/UartRxDma.cpp")
endif()

add_subdirectory(utils)
