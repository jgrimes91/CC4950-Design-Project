cmake_minimum_required(VERSION 3.12)

# rest of your project
add_executable(assignment_i2c
    main.c
)

target_link_libraries(assignment_i2c pico_lorawan hardware_adc hardware_pwm hardware_i2c)

# enable usb output, disable uart output
pico_enable_stdio_usb(assignment_i2c 1)
pico_enable_stdio_uart(assignment_i2c 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(assignment_i2c)
