target_include_directories(carvera_sources PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_sources(carvera_sources PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/lpc17xx_clkpwr.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/lpc17xx_gpio.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/lpc17xx_pinsel.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/lpc17xx_wdt.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/lpc17xx_gpdma.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/lpc17xx_uart.c"
)
