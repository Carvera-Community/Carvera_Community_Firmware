target_include_directories(carvera_sources PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

add_library(carvera_wifi_sources INTERFACE)
target_sources(carvera_wifi_sources INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/M8266HostIf_LPC17xx.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/WifiProvider.cpp"
)
target_link_libraries(carvera_wifi_sources INTERFACE
    "${CMAKE_SOURCE_DIR}/build/M8266WIFIDrv_LPC17xx.a"
)
