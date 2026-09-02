target_include_directories(carvera_sources PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_sources(carvera_sources PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/CANBus.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/CANopen.cpp"
)
