# This file contains source files and target configuration for RA6M5

include_guard()

# Collect all source files
file(GLOB_RECURSE RASC_SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ra_gen/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra_gen/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

# Collect assembly files if they exist
file(GLOB RASC_ASM_SOURCE_FILES ${RASC_ASM_FILES})

# Create executable target
add_executable(${RASC_PROJECT_NAME}.elf
    ${RASC_SOURCE_FILES}
    ${RASC_ASM_SOURCE_FILES}
)

# Apply compiler flags per language
target_compile_options(${RASC_PROJECT_NAME}.elf PRIVATE
    $<$<COMPILE_LANGUAGE:C>:${RASC_CMAKE_C_FLAGS}>
    $<$<COMPILE_LANGUAGE:CXX>:${RASC_CMAKE_CXX_FLAGS}>
    $<$<COMPILE_LANGUAGE:ASM>:${RASC_CMAKE_ASM_FLAGS}>
)

# Apply build configuration specific flags
target_compile_options(${RASC_PROJECT_NAME}.elf PRIVATE
    $<$<CONFIG:Debug>:${RASC_DEBUG_FLAGS}>
    $<$<CONFIG:Release>:${RASC_RELEASE_FLAGS}>
    $<$<CONFIG:MinSizeRel>:${RASC_MIN_SIZE_RELEASE_FLAGS}>
    $<$<CONFIG:RelWithDebInfo>:${RASC_RELEASE_WITH_DEBUG_INFO}>
)

# Apply linker flags
target_link_options(${RASC_PROJECT_NAME}.elf PRIVATE ${RASC_CMAKE_EXE_LINKER_FLAGS})

# Apply preprocessor definitions
target_compile_definitions(${RASC_PROJECT_NAME}.elf PRIVATE ${RASC_CMAKE_DEFINITIONS})

# Include directories
target_include_directories(${RASC_PROJECT_NAME}.elf PRIVATE
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
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Link directories
target_link_directories(${RASC_PROJECT_NAME}.elf PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/script
)

# Post-build step: Generate S-record file
if(CMAKE_OBJCOPY)
    add_custom_command(TARGET ${RASC_PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O srec $<TARGET_FILE:${RASC_PROJECT_NAME}.elf> ${CMAKE_CURRENT_BINARY_DIR}/${RASC_PROJECT_NAME}.srec
        COMMENT "Creating S-record file: ${RASC_PROJECT_NAME}.srec"
        VERBATIM
    )
    
    add_custom_command(TARGET ${RASC_PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${RASC_PROJECT_NAME}.elf> ${CMAKE_CURRENT_BINARY_DIR}/${RASC_PROJECT_NAME}.hex
        COMMENT "Creating Intel HEX file: ${RASC_PROJECT_NAME}.hex"
        VERBATIM
    )
    
    add_custom_command(TARGET ${RASC_PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${RASC_PROJECT_NAME}.elf> ${CMAKE_CURRENT_BINARY_DIR}/${RASC_PROJECT_NAME}.bin
        COMMENT "Creating binary file: ${RASC_PROJECT_NAME}.bin"
        VERBATIM
    )
endif()

# Print size information
if(CMAKE_SIZE)
    add_custom_command(TARGET ${RASC_PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${RASC_PROJECT_NAME}.elf>
        COMMENT "Executable size information"
        VERBATIM
    )
endif()

# Display build information
message(STATUS "Source files found: ${RASC_SOURCE_FILES}")
if(RASC_ASM_SOURCE_FILES)
    message(STATUS "Assembly files found: ${RASC_ASM_SOURCE_FILES}")
endif()
