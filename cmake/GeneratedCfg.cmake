# This file contains build configuration and compiler flags for RA6M5

include_guard()

# Target device configuration
set(RASC_TARGET_DEVICE R7FA6M5BH)
set(RASC_TARGET_ARCH cortex-m33)
set(RASC_PROJECT_NAME RA6M5_code)
set(RASC_TOOLCHAIN_NAME GCC)

# Base compiler flags for Cortex-M33 with hard FPU
set(RASC_BASE_FLAGS
    "-mfloat-abi=hard"
    "-mcpu=cortex-m33"
    "-mfpu=fpv5-sp-d16"
    "-mthumb"
    "-ffunction-sections"
    "-fdata-sections"
    "-fsigned-char"
    "-fmessage-length=0"
)

# Warning flags
set(RASC_WARNING_FLAGS
    "-Wall"
    "-Wextra"
    "-Wunused"
    "-Wuninitialized"
    "-Wmissing-declarations"
    "-Wconversion"
    "-Wpointer-arith"
    "-Wshadow"
    "-Wlogical-op"
    "-Waggregate-return"
    "-Wfloat-equal"
)

# Dependency generation flags
set(RASC_DEP_FLAGS "-MMD" "-MP")

# C compiler flags
set(RASC_CMAKE_C_FLAGS ${RASC_BASE_FLAGS} ${RASC_WARNING_FLAGS} ${RASC_DEP_FLAGS} "-std=c99")

# C++ compiler flags
set(RASC_CMAKE_CXX_FLAGS ${RASC_BASE_FLAGS} ${RASC_WARNING_FLAGS} ${RASC_DEP_FLAGS} "-std=c++11")

# Assembly flags
set(RASC_CMAKE_ASM_FLAGS ${RASC_BASE_FLAGS} ${RASC_WARNING_FLAGS} ${RASC_DEP_FLAGS} "-x" "assembler-with-cpp")

# Linker flags
set(RASC_CMAKE_EXE_LINKER_FLAGS
    ${RASC_BASE_FLAGS}
    ${RASC_WARNING_FLAGS}
    "-T${CMAKE_CURRENT_SOURCE_DIR}/script/fsp.ld"
    "-Wl,--gc-sections"
    "-Wl,-Map,${CMAKE_CURRENT_BINARY_DIR}/${RASC_PROJECT_NAME}.map"
    "--specs=nano.specs"
)

# Preprocessor definitions
set(RASC_CMAKE_DEFINITIONS
    "_RA_CORE=CM33"
    "_RA_ORDINAL=1"
    "_RENESAS_RA_"
)

# GCC version specific flags for GCC >= 12.2
if(CMAKE_C_COMPILER_ID STREQUAL "GNU" AND CMAKE_C_COMPILER_VERSION VERSION_GREATER_EQUAL 12.2)
    list(PREPEND RASC_CMAKE_C_FLAGS
        "--param=min-pagesize=0"
        "-Wno-format-truncation"
        "-Wno-stringop-overflow"
    )
    list(PREPEND RASC_CMAKE_CXX_FLAGS
        "--param=min-pagesize=0"
        "-Wno-format-truncation"
        "-Wno-stringop-overflow"
    )
endif()

# Build configuration specific optimization flags
set(RASC_DEBUG_FLAGS "-g" "-O0")
set(RASC_RELEASE_FLAGS "-O2" "-DNDEBUG")
set(RASC_MIN_SIZE_RELEASE_FLAGS "-Os" "-DNDEBUG")
set(RASC_RELEASE_WITH_DEBUG_INFO "-g" "-O2" "-DNDEBUG")

# Assembly source files pattern
set(RASC_ASM_FILES "${CMAKE_CURRENT_SOURCE_DIR}/ra_gen/*.asm")
