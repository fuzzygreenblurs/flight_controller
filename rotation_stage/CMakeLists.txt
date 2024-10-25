cmake_minimum_required(VERSION 3.12)                # defines the minimum required version of CMAKE

if (NOT DEFINED PICO_SDK_PATH)
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
endif()

# if (NOT DEFINED PICO_SDK_PATH)
#     set(PICO_SDK_PATH "/Users/akhilsankar/workspace/embedded/Pico/pico-sdk")
# endif()

include(${PICO_SDK_PATH}/pico_sdk_init.cmake)
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)  # imports the Pico SDK: sets up paths/libraries specific to the Pico
pico_sdk_init()                                           # initializes the SDK. sets up libraries/headers for project use

################# IMU #######################
project(pico-multicore-imu)

add_executable(imu)
target_sources(imu PRIVATE imu.c)
target_link_libraries(imu
    pico_multicore
    pico_stdlib
    pico_multicore
    hardware_i2c
    hardware_irq
    hardware_gpio
    hardware_sync
    hardware_adc                            # remove this. quick test.
)

pico_enable_stdio_usb(imu 1)                # enabled USB standard IO (for things like printf) for the `temperature` target
pico_enable_stdio_uart(imu 0)               # disables (0) UART based standard IO for `temperature`

pico_add_extra_outputs(imu)                 # generate extra output formats (like .uf2, used for flashing on rp2040) of `multicore`. 


################# IMU #######################

################# TEMPERATURE #######################

# project(pico-multicore-temp)                        # names the project "pico-multicore-temp". this name manages build targets and outputs

# add_executable(temperature)                         # create an executable called `temperature`
# target_sources(temperature PRIVATE temperature.c)   # use the local file `temperature.c` as source code to generate the executable
# target_link_libraries(temperature                   # links libraries to the `temperature` target
#     pico_multicore                                  # this library provides temperature support for the RP2040
#     pico_stdlib                                     # std library that provides basic functions for IO and GPIO 
#     hardware_adc                                    # this library allows us to interface with the onboard ADC
# )

# pico_enable_stdio_usb(temperature 1)                # enabled USB standard IO (for things like printf) for the `temperature` target
# pico_enable_stdio_uart(temperature 0)               # disables (0) UART based standard IO for `temperature`

# pico_add_extra_outputs(temperature)                 # generate extra output formats (like .uf2) of `multicore`. 

################# TEMPERATURE #######################