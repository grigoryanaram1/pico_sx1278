add_executable(unittest
               unittest.c)

pico_enable_stdio_uart(unittest 1)
pico_enable_stdio_usb(unittest 0)

pico_set_program_name(unittest "unittest")
pico_set_program_version(unittest "0.1")


target_include_directories(unittest
                           PRIVATE
                           "../lib/include/")

target_link_libraries(unittest
                      pico_stdlib
                      hardware_timer
                      hardware_spi)

target_link_libraries(unittest sx1278)

pico_add_extra_outputs(unittest)