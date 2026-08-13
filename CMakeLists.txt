cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(nRF5340_Presence)

target_sources(app PRIVATE
    src/main.c
    src/radar.c
    src/sen5x.c
    src/tlv.c
    src/frame.c
    src/store.c
    src/ble.c
)
target_include_directories(app PRIVATE src)
