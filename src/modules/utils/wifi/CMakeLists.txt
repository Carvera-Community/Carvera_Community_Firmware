target_include_directories(carvera_sources PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_sources(carvera_sources PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/M8266HostIf_LPC17xx.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/WifiProvider.cpp"
)
