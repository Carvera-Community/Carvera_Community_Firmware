target_include_directories(carvera_sources PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_sources(carvera_sources PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/Block.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/BlockQueue.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Conveyor.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Planner.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Robot.cpp"
)

add_subdirectory(arm_solutions)
