target_include_directories(carvera_sources PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

add_library(carvera_usb_host_sources INTERFACE)
target_sources(carvera_usb_host_sources INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/MSCFileSystem.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/USBHostCDC.cpp"
)

add_subdirectory(SDCard)
add_subdirectory(USBHostLite)
