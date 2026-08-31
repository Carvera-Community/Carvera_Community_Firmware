target_include_directories(carvera_sources PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_sources(carvera_spindle_modbus_sources INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/Modbus.cpp"
)
