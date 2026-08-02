target_include_directories(carvera_sources PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_sources(carvera_sources PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/GcodeDispatch.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SerialConsole.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SerialConsole2.cpp"
)

add_subdirectory(utils)
