cmake_minimum_required(VERSION 3.22)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME water)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

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

# Add user sources from MDK-ARM project
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    MDK-ARM/system/system.c
    MDK-ARM/system/sys.c
    MDK-ARM/system/delay.c
    MDK-ARM/hardware/OLED.c
    MDK-ARM/hardware/OLED_Data.c
    MDK-ARM/hardware/APP_MAIN.c
    MDK-ARM/hardware/task_execute.c
    MDK-ARM/hardware/bsp_motor.c
    MDK-ARM/hardware/Emm_V5.c
    MDK-ARM/hardware/pressADC.c
    MDK-ARM/hardware/normal_motor.c
    MDK-ARM/hardware/layser.c
    MDK-ARM/NVIC/timer.c
    MDK-ARM/NVIC/UART_NVIC.c
    MDK-ARM/NVIC/uart_execute.c
    MDK-ARM/NVIC/TOF.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    MDK-ARM/hardware
    MDK-ARM/NVIC
    MDK-ARM/system
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# HAL/StdPeriph drivers sometimes intentionally keep unused parameters for API compatibility.
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wno-unused-parameter
)

# Keep OLED font tables unchanged while silencing style-only warnings.
set_source_files_properties(MDK-ARM/hardware/OLED_Data.c PROPERTIES
    COMPILE_OPTIONS "-Wno-missing-braces;-Wno-missing-field-initializers"
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
