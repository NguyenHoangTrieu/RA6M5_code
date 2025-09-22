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
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/GeneratedCfg.cmake)

# Set target and linker script
set(TARGET ${RASC_PROJECT_NAME}.elf)
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/script/fsp.ld)

# Essential sources - Renesas RA6M5 files
set(SRC_FILES
    # Startup and system files (RA6M5)
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/src/bsp/cmsis/Device/RENESAS/Source/startup.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/src/bsp/cmsis/Device/RENESAS/Source/system.c
    
    # Board support
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/board/ra6m5_ck/board_init.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/board/ra6m5_ck/board_leds.c
    
    # BSP (Board Support Package) files
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/src/bsp/mcu/all/bsp_clocks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/src/bsp/mcu/all/bsp_common.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/src/bsp/mcu/all/bsp_delay.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/src/bsp/mcu/all/bsp_group_irq.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/src/bsp/mcu/all/bsp_guard.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/src/bsp/mcu/all/bsp_io.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/src/bsp/mcu/all/bsp_ipc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/src/bsp/mcu/all/bsp_irq.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/src/bsp/mcu/all/bsp_macl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/src/bsp/mcu/all/bsp_register_protection.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/src/bsp/mcu/all/bsp_sbrk.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/src/bsp/mcu/all/bsp_sdram.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/src/bsp/mcu/all/bsp_security.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/src/bsp/mcu/ra6m5/bsp_linker.c
    
    # FSP driver files
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/src/r_ioport/r_ioport.c
    
    # FreeRTOS source files
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/aws/FreeRTOS/FreeRTOS/Source/event_groups.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/aws/FreeRTOS/FreeRTOS/Source/list.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/aws/FreeRTOS/FreeRTOS/Source/queue.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/aws/FreeRTOS/FreeRTOS/Source/stream_buffer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/aws/FreeRTOS/FreeRTOS/Source/tasks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/aws/FreeRTOS/FreeRTOS/Source/timers.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/src/rm_freertos_port/port.c
    
    # Generated files
    ${CMAKE_CURRENT_SOURCE_DIR}/ra_gen/blinky_thread.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra_gen/common_data.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra_gen/hal_data.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra_gen/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra_gen/pin_data.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra_gen/vector_data.c
    
    # Application files
    ${CMAKE_CURRENT_SOURCE_DIR}/src/blinky_thread_entry.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hal_warmstart.c
)

# Include directories
set(INC_FOLDERS
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/arm/CMSIS_6/CMSIS/Core/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/aws/FreeRTOS/FreeRTOS/Source/include
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/inc/api
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/inc/instances
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/src/rm_freertos_port
    ${CMAKE_CURRENT_SOURCE_DIR}/ra_cfg/aws
    ${CMAKE_CURRENT_SOURCE_DIR}/ra_cfg/fsp_cfg
    ${CMAKE_CURRENT_SOURCE_DIR}/ra_cfg/fsp_cfg/bsp
    ${CMAKE_CURRENT_SOURCE_DIR}/ra_gen
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_BINARY_DIR}
)

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
