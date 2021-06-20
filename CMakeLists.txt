cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(pico-tof)

# Initialize the SDK
pico_sdk_init()

add_library(${PROJECT_NAME}
    include/pico_tof.h
    include/vl53lx_register_map.h
    src/vl53lx.c
)

target_compile_options(${PROJECT_NAME} PRIVATE -D${SENSOR})

target_include_directories(${PROJECT_NAME} PRIVATE include)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${PROJECT_NAME} PRIVATE pico_stdlib hardware_i2c)

if (${BUILD_EXAMPLES})
        add_subdirectory(examples)
endif()
