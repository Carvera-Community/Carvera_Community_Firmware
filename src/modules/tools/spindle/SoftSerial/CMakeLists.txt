target_include_directories(carvera_sources PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_sources(carvera_spindle_modbus_sources INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/BufferedSoftSerial.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SoftSerial.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SoftSerial_rx.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SoftSerial_tx.cpp"
)
