add_library(sx1278 STATIC
            source/sx1278.c)

target_include_directories(sx1278 PRIVATE "include/")

target_link_libraries(sx1278
                      pico_stdlib
                      hardware_timer
                      hardware_spi)
