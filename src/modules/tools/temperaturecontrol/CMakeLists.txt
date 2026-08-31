target_include_directories(carvera_sources PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_sources(carvera_sources PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/AD8495.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/max31855.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/PT100_E3D.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/TemperatureControl.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/TemperatureControlPool.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Thermistor.cpp"
)
