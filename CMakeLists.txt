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
    firmware/platform/hardware_init.c
    firmware/control/motor_current_loop.c
    firmware/compensation/motor_cogging.c
    firmware/calibration/motor_calibration.c
    firmware/compensation/motor_friction.c
    firmware/compensation/motor_friction_storage.c
    firmware/calibration/motor_identify.c
    firmware/music/motor_music.c
    firmware/calibration/motor_parameters.c
    firmware/control/motor_position_loop.c
    firmware/control/motor_speed_loop.c
    firmware/app/motor_system.c
    firmware/app/motor_calibration_session.c
    firmware/app/motor_control.c
    firmware/app/motor_host.c
    firmware/app/motor_haptic.c
    firmware/app/motor_modes.c
    firmware/app/motor_runtime.c
    firmware/app/private/motor_context.c
    firmware/music/motor_songs.c
    firmware/control/motor_trajectory.c
    firmware/drivers/mt6826s.c
    firmware/drivers/motor_encoder.c
    firmware/calibration/motor_encoder_calibration.c
    firmware/algorithm/pid.c
    firmware/communication/pc_protocol.c
    firmware/control/svpwm.c
    firmware/drivers/user_io.c
    firmware/communication/vofa_usart.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    firmware/app
    firmware/app/private
    firmware/platform
    firmware/control
    firmware/compensation
    firmware/calibration
    firmware/drivers
    firmware/communication
    firmware/music
    firmware/algorithm
    firmware/config
    firmware/diagnostics
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
