target_include_directories(carvera_sources PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_sources(carvera_sources PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/usbhost_lpc17xx.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/usbhost_ms.cpp"
)
