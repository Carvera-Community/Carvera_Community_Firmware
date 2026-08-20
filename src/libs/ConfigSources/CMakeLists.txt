target_include_directories(carvera_sources PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_sources(carvera_sources PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/FileConfigSource.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/FirmConfigSource.cpp"
)
if(REMOTE_SETTINGS_TRANSFER)
    target_sources(carvera_sources PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/RemoteConfigSource.cpp")
endif()
