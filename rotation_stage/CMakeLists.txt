cmake_minimum_required(VERSION 3.12)                # defines the minimum required version of CMAKE

if (NOT DEFINED PICO_SDK_PATH)
    set(PICO_SDK_PATH "/Users/akhilsankar/workspace/embedded/Pico/pico-sdk")
endif()

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake) # imports the Pico SDK: sets up paths/libraries specific to the Pico

################# IMU #######################
project(pico-multicore-imu)
pico_sdk_init()

add_executable(imu)
target_sources(imu PRIVATE imu.c)
target_link_libraries(imu
    pico_multicore
    pico_std_lib
    hardware_i2c
)


################# IMU #######################

################# TEMPERATURE #######################

project(pico-multicore)                             # names the project "pico-multicore". this name manages build targets and outputs

pico_sdk_init()                                     # initializes the SDK. sets up libraries/headers for project use

add_executable(temperature)                         # create an executable called `temperature`
target_sources(temperature PRIVATE temperature.c)   # use the local file `temperature.c` as source code to generate the executable
target_link_libraries(temperature                   # links libraries to the `temperature` target
    pico_multicore                                  # this library provides temperature support for the RP2040
    pico_stdlib                                     # std library that provides basic functions for IO and GPIO 
    hardware_adc                                    # this library allows us to interface with the onboard ADC
)

pico_enable_stdio_usb(temperature 1)                # enabled USB standard IO (for things like printf) for the `temperature` target
pico_enable_stdio_uart(temperature 0)               # disables (0) UART based standard IO for `temperature`

pico_add_extra_outputs(temperature)                 # generate extra output formats (like .uf2) of `multicore`. 
# we use the .uf2 file to flash firmware onto the rp2040

################# TEMPERATURE #######################