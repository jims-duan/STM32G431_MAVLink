cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME G431_MAVLink)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# 启用 printf 浮点数支持（修正：-u 和 _printf_float 之间需要空格）
if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID STREQUAL "ARMClang")
    add_link_options(-u _printf_float)
endif()

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    # APP 层
    ${CMAKE_CURRENT_SOURCE_DIR}/USER/APP/src/led_fsm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USER/APP/src/MavLinkAPP.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USER/APP/src/sbus_task.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USER/APP/src/uav_pid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USER/APP/src/odom_gps.c

    # BSP 层
    ${CMAKE_CURRENT_SOURCE_DIR}/USER/BSP/src/led.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USER/BSP/src/tim_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USER/BSP/src/usart_it.c
    
    # Middleware 层
    ${CMAKE_CURRENT_SOURCE_DIR}/USER/Middleware/src/pid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USER/Middleware/src/ring_buffer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USER/Middleware/src/sbus.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USER/Middleware/src/LowPassFilter.c

    # USMART调试层
    ${CMAKE_CURRENT_SOURCE_DIR}/USMART/src/usmart_config.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USMART/src/usmart_str.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USMART/src/usmart.c

    # 系统通用组件层
    ${CMAKE_CURRENT_SOURCE_DIR}/SYSTEM/src/system.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/USER/APP/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/USER/BSP/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/USER/Middleware/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/USER/ThirdParty/inc/mavlink
    ${CMAKE_CURRENT_SOURCE_DIR}/USER/ThirdParty/inc/mavlink/common
    ${CMAKE_CURRENT_SOURCE_DIR}/USER/ThirdParty/inc/mavlink/minimal
    ${CMAKE_CURRENT_SOURCE_DIR}/USER/ThirdParty/inc/mavlink/standard

    ${CMAKE_CURRENT_SOURCE_DIR}/USMART/inc

    ${CMAKE_CURRENT_SOURCE_DIR}/SYSTEM/inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    m
)
