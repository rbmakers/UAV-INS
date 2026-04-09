cmake_minimum_required(VERSION 3.13)

# Pull in Raspberry Pi Pico SDK
# Set PICO_SDK_PATH in environment or pass -DPICO_SDK_PATH=...
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(nav_system C CXX ASM)
set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 17)

# Target board — RP2350 (use pico2 board def)
set(PICO_BOARD pico2)

pico_sdk_init()

# ─── Compiler warnings & optimisations ───────────────────
add_compile_options(
    -Wall -Wextra -Wno-unused-parameter
    -O2
    -ffunction-sections -fdata-sections
    -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard
)

# ─── Source files ─────────────────────────────────────────
set(NAV_SOURCES
    main.c
    comms/comms.h           # header-only HAL helpers
    sensors/sensor_hal.h
    sensors/sensor_hal.c
    sensors/imu_driver.c
    sensors/gnss_driver.c
    sensors/peripheral_drivers.c
    ekf/ekf.c
    fsm/nav_fsm.c
)

add_executable(nav_system ${NAV_SOURCES})

# ─── Link pico-sdk libraries ──────────────────────────────
target_link_libraries(nav_system
    pico_stdlib
    pico_multicore
    hardware_spi
    hardware_i2c
    hardware_uart
    hardware_gpio
    hardware_irq
    hardware_timer
    hardware_dma
    m                       # libm for sinf, cosf, sqrtf, powf
)

# ─── Include dirs ─────────────────────────────────────────
target_include_directories(nav_system PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/comms
    ${CMAKE_CURRENT_SOURCE_DIR}/sensors
    ${CMAKE_CURRENT_SOURCE_DIR}/ekf
    ${CMAKE_CURRENT_SOURCE_DIR}/fsm
)

# ─── Output formats ───────────────────────────────────────
pico_add_extra_outputs(nav_system)   # .uf2, .bin, .hex, .map

# Enable USB stdio (for printf telemetry during development)
pico_enable_stdio_usb (nav_system 1)
pico_enable_stdio_uart(nav_system 0)

# ─── Optional: linker GC + map file ──────────────────────
target_link_options(nav_system PRIVATE
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_BINARY_DIR}/nav_system.map
)

# ─── Print binary size after build ───────────────────────
add_custom_command(TARGET nav_system POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:nav_system>
    COMMENT "Binary size:"
)
