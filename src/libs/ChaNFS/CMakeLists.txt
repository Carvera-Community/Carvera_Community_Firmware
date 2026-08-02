target_include_directories(carvera_sources PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_sources(carvera_sources PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/FATDirHandle.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/FATFileHandle.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/FATFileSystem.cpp"
)

add_subdirectory(CHAN_FS)
