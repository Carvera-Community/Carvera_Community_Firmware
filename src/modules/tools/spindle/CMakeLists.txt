target_include_directories(carvera_sources PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_sources(carvera_sources PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/AnalogSpindleControl.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/PIDPWMSpindleControl.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/PWMSpindleControl.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SpindleControl.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SpindleMaker.cpp"
)

add_library(carvera_spindle_modbus_sources INTERFACE)
target_sources(carvera_spindle_modbus_sources INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/HuanyangSpindleControl.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/ModbusSpindleControl.cpp"
)
add_subdirectory(Modbus)
add_subdirectory(SoftSerial)

add_library(carvera_spindle_vesc_sources INTERFACE)
target_sources(carvera_spindle_vesc_sources INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/VESCSpindleControl.cpp"
)
