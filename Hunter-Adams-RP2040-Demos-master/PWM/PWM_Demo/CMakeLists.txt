# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

add_executable(PWM_Demo)

pico_generate_pio_header(PWM_Demo ${CMAKE_CURRENT_LIST_DIR}/hsync.pio)
pico_generate_pio_header(PWM_Demo ${CMAKE_CURRENT_LIST_DIR}/vsync.pio)
pico_generate_pio_header(PWM_Demo ${CMAKE_CURRENT_LIST_DIR}/rgb.pio)


# must match with executable name and source file names
target_sources(PWM_Demo PRIVATE pwm-demo.c vga16_graphics.c mpu6050.c)

# Add pico_multicore which is required for multicore functionality
target_link_libraries(PWM_Demo pico_stdlib pico_multicore pico_bootsel_via_double_reset hardware_pwm hardware_irq hardware_dma hardware_adc hardware_pio hardware_i2c)

# create map/bin/hex file etc.
pico_add_extra_outputs(PWM_Demo)