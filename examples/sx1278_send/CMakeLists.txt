add_executable(sx1278_send
               sx1278_send.c)

pico_enable_stdio_uart(sx1278_send 1)
pico_enable_stdio_usb(sx1278_send 0)

pico_set_program_name(sx1278_send "sx1278_send")
pico_set_program_version(sx1278_send "0.1")


target_include_directories(sx1278_send
                           PRIVATE
                           "../../lib/include/")

target_link_libraries(sx1278_send
                      pico_stdlib
                      hardware_timer
                      hardware_spi)

target_link_libraries(sx1278_send sx1278)

pico_add_extra_outputs(sx1278_send)
