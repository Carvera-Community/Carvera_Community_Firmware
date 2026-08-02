add_library(carvera_embedded_options INTERFACE)
target_compile_options(carvera_embedded_options INTERFACE
    "$<$<COMPILE_LANGUAGE:C,CXX>:-g3;-mcpu=cortex-m3;-mthumb;-mthumb-interwork;-ffunction-sections;-fdata-sections;-fno-exceptions;-fno-delete-null-pointer-checks;-Wall;-Wextra;-Wno-unused-parameter>"
    "$<$<COMPILE_LANGUAGE:ASM>:-g3;-mcpu=cortex-m3;-mthumb;-x;assembler-with-cpp>"
)

add_library(carvera_project_options INTERFACE)
target_link_libraries(carvera_project_options INTERFACE carvera_embedded_options)
target_compile_options(carvera_project_options INTERFACE
    "$<$<COMPILE_LANGUAGE:C,CXX>:-Wpointer-arith;-Wredundant-decls;-Wcast-qual;-Wcast-align>"
)
