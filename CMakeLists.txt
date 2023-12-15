cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(my_project)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project

add_executable(app
    #picopwm.cpp
    si5351.cpp
    
    main.cpp
    #usdxpico.cpp
    #test.cpp
    #usdx.cpp
    #test-pll.cpp
    #qcx.cpp
    #qcx_si5351.cpp
    #mic.cpp
)

target_link_libraries(app pico_stdlib hardware_gpio hardware_pwm hardware_adc hardware_i2c)

pico_enable_stdio_usb(app 1)
pico_enable_stdio_uart(app 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(app)