cmake_minimum_required(VERSION 3.12)
project(pico-i2s-pio)

if (NOT DEFINED I2S_MAX_FREQ_KHZ)
    set(I2S_MAX_FREQ_KHZ 384)
endif()

if (NOT DEFINED I2S_QUEUE_LEN)
    set(I2S_QUEUE_LEN 10)
endif()
 
add_library(pico-i2s-pio STATIC i2s.c)
pico_generate_pio_header(pico-i2s-pio ${CMAKE_CURRENT_LIST_DIR}/i2s.pio)
target_link_libraries(pico-i2s-pio
        pico_stdlib
        pico_multicore
        hardware_dma
        hardware_pio
        hardware_clocks
        hardware_sync
        )

target_compile_definitions(pico-i2s-pio PUBLIC
    I2S_MAX_FREQ_KHZ=${I2S_MAX_FREQ_KHZ}
    I2S_QUEUE_LEN=${I2S_QUEUE_LEN}
)

target_include_directories(pico-i2s-pio PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})