cmake_minimum_required(VERSION 3.15)

# MUST specify toolchain file BEFORE project() call
if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi-gcc.cmake")
endif()

project(RA6M5_code VERSION 1.0.0 LANGUAGES C CXX ASM)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include configuration files
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Source.cmake)

# Set target and linker script
set(TARGET ${RASC_PROJECT_NAME}.elf)
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/script/fsp.ld)

# Create executable
add_executable(${TARGET} ${SRC_FILES})

# Set target properties
target_include_directories(${TARGET} PRIVATE ${INC_FOLDERS})

# Compiler definitions
target_compile_definitions(${TARGET} PRIVATE ${RASC_CMAKE_DEFINITIONS})

# C Compiler flags
target_compile_options(${TARGET} PRIVATE
    $<$<COMPILE_LANGUAGE:C>:${RASC_CMAKE_C_FLAGS}>
    $<$<COMPILE_LANGUAGE:CXX>:${RASC_CMAKE_CXX_FLAGS}>
    $<$<COMPILE_LANGUAGE:ASM>:${RASC_CMAKE_ASM_FLAGS}>
)

# Build configuration specific flags
target_compile_options(${TARGET} PRIVATE
    $<$<CONFIG:Debug>:${RASC_DEBUG_FLAGS}>
    $<$<CONFIG:Release>:${RASC_RELEASE_FLAGS}>
    $<$<CONFIG:MinSizeRel>:${RASC_MIN_SIZE_RELEASE_FLAGS}>
    $<$<CONFIG:RelWithDebInfo>:${RASC_RELEASE_WITH_DEBUG_INFO}>
)

# Linker flags
target_link_options(${TARGET} PRIVATE ${RASC_CMAKE_EXE_LINKER_FLAGS})

# Link directories
target_link_directories(${TARGET} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/script
)

# Post-build commands to create hex and bin files
add_custom_command(TARGET ${TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${RASC_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${RASC_PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O srec $<TARGET_FILE:${TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${RASC_PROJECT_NAME}.srec
    COMMENT "Building ${RASC_PROJECT_NAME}.hex, .bin, and .srec"
)

# Print executable size
add_custom_command(TARGET ${TARGET} POST_BUILD

    COMMAND ${CMAKE_COMMAND} -E echo ""
    COMMAND ${CMAKE_COMMAND} -E echo "=========================================="
    COMMAND ${CMAKE_COMMAND} -E echo "         BUILD SUCCESSFUL!"
    COMMAND ${CMAKE_COMMAND} -E echo "=========================================="
    COMMAND ${CMAKE_COMMAND} -E echo ""
    
    COMMAND ${CMAKE_COMMAND} -E echo "Memory Usage:"
    COMMAND ${CMAKE_SIZE_UTIL} $<TARGET_FILE:${TARGET}>
    COMMAND ${CMAKE_COMMAND} -E echo ""
    COMMAND ${CMAKE_COMMAND} -E echo "Ready to flash: ${RASC_PROJECT_NAME}.hex"
    COMMAND ${CMAKE_COMMAND} -E echo "=========================================="
    
    COMMENT "Creating firmware files and showing build summary"
    VERBATIM
)
# Custom targets for flashing and erasing
add_custom_target(flash
    DEPENDS ${TARGET}
    COMMAND rfp-cli -device RA6M5 -interface USB -auto -erase -program ${CMAKE_CURRENT_BINARY_DIR}/${RASC_PROJECT_NAME}.mot -verify
    COMMENT "Flashing ${RASC_PROJECT_NAME}.mot to RA6M5"
)

add_custom_target(erase
    COMMAND rfp-cli -device RA6M5 -interface USB -auto -erase
    COMMENT "Erasing RA6M5 flash"
)

# Set output directory
set_target_properties(${TARGET} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    OUTPUT_NAME ${RASC_PROJECT_NAME}
)

# Ensure linker script exists
if(NOT EXISTS ${LINKER_SCRIPT})
    message(FATAL_ERROR "Linker script not found: ${LINKER_SCRIPT}")
endif()

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
