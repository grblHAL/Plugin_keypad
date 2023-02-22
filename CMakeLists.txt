add_library(keypad INTERFACE)

target_sources(keypad INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/keypad.c
 ${CMAKE_CURRENT_LIST_DIR}/macros.c
 ${CMAKE_CURRENT_LIST_DIR}/display/i2c_leds.c
 ${CMAKE_CURRENT_LIST_DIR}/display/i2c_interface.c
)

target_include_directories(keypad INTERFACE ${CMAKE_CURRENT_LIST_DIR})
