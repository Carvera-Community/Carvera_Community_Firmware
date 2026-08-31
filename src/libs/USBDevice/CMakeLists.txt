target_include_directories(carvera_sources PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_sources(carvera_sources PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/MSCFileSystem.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/USBHostCDC.cpp"
)

add_subdirectory(SDCard)
add_subdirectory(USBHostLite)
