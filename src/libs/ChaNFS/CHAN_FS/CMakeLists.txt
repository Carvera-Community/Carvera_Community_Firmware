target_include_directories(carvera_sources PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_sources(carvera_sources PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/diskio.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/ff.cpp"
)

add_subdirectory(option)
