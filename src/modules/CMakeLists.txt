target_include_directories(carvera_sources PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

add_subdirectory(communication)
add_subdirectory(robot)
add_subdirectory(tools)
add_subdirectory(utils)
