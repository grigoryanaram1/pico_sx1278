add_executable(sx1278_receive
               sx1278_receive.c)

pico_enable_stdio_uart(sx1278_receive 1)
pico_enable_stdio_usb(sx1278_receive 0)

pico_set_program_name(sx1278_receive "sx1278_receive")
pico_set_program_version(sx1278_receive "0.1")


target_include_directories(sx1278_receive
                           PRIVATE
                           "../../lib/include/")

target_link_libraries(sx1278_receive
                      pico_stdlib
                      hardware_timer
                      hardware_spi)

target_link_libraries(sx1278_receive sx1278)

pico_add_extra_outputs(sx1278_receive)
