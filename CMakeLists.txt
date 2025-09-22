cmake_minimum_required(VERSION 3.16.4)

# MUST specify toolchain file BEFORE project() call
if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi.cmake")
endif()

project(RA6M5_code VERSION 1.0.0 LANGUAGES C CXX ASM)

# Set build configurations
set(CMAKE_CONFIGURATION_TYPES Debug Release RelWithDebInfo MinSizeRel)

# Default build type
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build" FORCE)
endif()

# Include configuration and source files
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/GeneratedCfg.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/GeneratedSrc.cmake)

# Display build information
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Project name: ${RASC_PROJECT_NAME}")
message(STATUS "Target device: ${RASC_TARGET_DEVICE}")
message(STATUS "Target architecture: ${RASC_TARGET_ARCH}")

# Verify compiler is actually ARM GCC
execute_process(
    COMMAND ${CMAKE_C_COMPILER} --version
    OUTPUT_VARIABLE COMPILER_VERSION
    ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(COMPILER_VERSION MATCHES "arm-none-eabi")
    string(REGEX MATCH "([0-9]+\\.[0-9]+\\.[0-9]+)" GCC_VERSION ${COMPILER_VERSION})
    message(STATUS "ARM GCC Version: ${GCC_VERSION}")
else()
    message(FATAL_ERROR "Wrong compiler detected! Expected arm-none-eabi-gcc, got: ${CMAKE_C_COMPILER}")
endif()
