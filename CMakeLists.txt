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
set(CMAKE_PROJECT_NAME g431_foc)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
include(cmake/development-profile.cmake)
set(FOC_PROFILE "${FOC_DEFAULT_PROFILE}" CACHE STRING "sensored or sensorless")
set_property(CACHE FOC_PROFILE PROPERTY STRINGS sensored sensorless)
if(NOT FOC_PROFILE MATCHES "^(sensored|sensorless)$")
    message(FATAL_ERROR "FOC_PROFILE must be sensored or sensorless")
endif()

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

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
    user/src/motor_angle_reference.c
    user/src/hardware_init.c
    user/src/motor_current_loop.c
    user/src/motor_cogging.c
    user/src/motor_calibration.c
    user/src/motor_feedforward.c
    user/src/motor_friction_storage.c
    user/src/motor_identify.c
    user/src/motor_music.c
    user/src/motor_parameters.c
    user/src/motor_position_loop.c
    user/src/motor_speed_loop.c
    user/src/motor_system.c
    user/src/motor_songs.c
    user/src/motor_trajectory.c
    user/src/mt6826s.c
    user/src/motor_encoder.c
    user/src/pid.c
    user/src/pc_protocol.c
    user/src/svpwm.c
    user/src/user_io.c
    user/src/vofa_usart.c
)

# Add include paths
if(FOC_PROFILE STREQUAL "sensorless")
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE user/src/as5600.c)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE FOC_PROFILE_SENSORLESS=1)
else()
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE FOC_PROFILE_SENSORLESS=0)
endif()
message(STATUS "FOC development profile: ${FOC_PROFILE}")
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    user/inc
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
)
