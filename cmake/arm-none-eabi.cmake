# Toolchain file for ARM Cortex-M cross compilation
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Function to find ARM toolchain automatically
function(find_arm_toolchain_path)
    set(ARM_TOOLCHAIN_FOUND FALSE PARENT_SCOPE)
    set(ARM_TOOLCHAIN_BIN_PATH "" PARENT_SCOPE)
    
    # Search paths for ARM toolchain
    set(SEARCH_PATHS
        # User-defined paths (highest priority)
        "${ARM_TOOLCHAIN_PATH}"
        "$ENV{ARM_TOOLCHAIN_PATH}"
        "$ENV{ARM_GCC_TOOLCHAIN_PATH}"
        
        # Windows ARM toolchain locations
        "C:/Program Files (x86)/GNU Arm Embedded Toolchain/*/bin"
        "C:/Program Files/GNU Arm Embedded Toolchain/*/bin"
        "C:/Program Files (x86)/GNU Tools ARM Embedded/*/bin"  
        "C:/Program Files/GNU Tools ARM Embedded/*/bin"
        "C:/Program Files (x86)/Arm GNU Toolchain*/bin"
        "C:/Program Files/Arm GNU Toolchain*/bin"
        
        # Linux paths
        "/usr/bin"
        "/usr/local/bin"
        "/opt/gcc-arm-none-eabi*/bin"
        "/opt/arm-none-eabi*/bin"
        
        # macOS paths
        "/usr/local/bin"
        "/opt/homebrew/bin"
        "/Applications/ARM/bin"
    )
    
    message(STATUS "Searching for ARM GCC toolchain...")
    
    find_program(ARM_GCC_FOUND
        NAMES arm-none-eabi-gcc
        PATHS ${SEARCH_PATHS}
        DOC "ARM GCC Compiler"
        NO_DEFAULT_PATH
    )
    
    if(ARM_GCC_FOUND)
        get_filename_component(FOUND_BIN_PATH ${ARM_GCC_FOUND} DIRECTORY)
        message(STATUS "Found ARM GCC at: ${ARM_GCC_FOUND}")
        message(STATUS "ARM toolchain bin path: ${FOUND_BIN_PATH}")
        
        # Verify other tools exist
        find_program(ARM_GXX_FOUND arm-none-eabi-g++ PATHS ${FOUND_BIN_PATH} NO_DEFAULT_PATH REQUIRED)
        find_program(ARM_OBJCOPY_FOUND arm-none-eabi-objcopy PATHS ${FOUND_BIN_PATH} NO_DEFAULT_PATH REQUIRED) 
        find_program(ARM_SIZE_FOUND arm-none-eabi-size PATHS ${FOUND_BIN_PATH} NO_DEFAULT_PATH REQUIRED)
        
        set(ARM_TOOLCHAIN_FOUND TRUE PARENT_SCOPE)
        set(ARM_TOOLCHAIN_BIN_PATH ${FOUND_BIN_PATH} PARENT_SCOPE)
        
        message(STATUS "ARM toolchain verification successful")
    else()
        message(FATAL_ERROR 
            "ARM toolchain not found! Please install ARM GCC toolchain or set ARM_TOOLCHAIN_PATH.\n"
            "Download from: https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain"
        )
    endif()
endfunction()

# Find toolchain
find_arm_toolchain_path()

# Set binary extension
if(CMAKE_HOST_WIN32)
    set(TOOL_EXECUTABLE_SUFFIX ".exe")
else()
    set(TOOL_EXECUTABLE_SUFFIX "")
endif()

# Set compilers - MUST be absolute paths for cross-compilation
set(CMAKE_C_COMPILER "${ARM_TOOLCHAIN_BIN_PATH}/arm-none-eabi-gcc${TOOL_EXECUTABLE_SUFFIX}")
set(CMAKE_CXX_COMPILER "${ARM_TOOLCHAIN_BIN_PATH}/arm-none-eabi-g++${TOOL_EXECUTABLE_SUFFIX}")
set(CMAKE_ASM_COMPILER "${ARM_TOOLCHAIN_BIN_PATH}/arm-none-eabi-gcc${TOOL_EXECUTABLE_SUFFIX}")

# Set other tools
set(CMAKE_AR "${ARM_TOOLCHAIN_BIN_PATH}/arm-none-eabi-ar${TOOL_EXECUTABLE_SUFFIX}")
set(CMAKE_OBJCOPY "${ARM_TOOLCHAIN_BIN_PATH}/arm-none-eabi-objcopy${TOOL_EXECUTABLE_SUFFIX}")
set(CMAKE_OBJDUMP "${ARM_TOOLCHAIN_BIN_PATH}/arm-none-eabi-objdump${TOOL_EXECUTABLE_SUFFIX}")
set(CMAKE_SIZE "${ARM_TOOLCHAIN_BIN_PATH}/arm-none-eabi-size${TOOL_EXECUTABLE_SUFFIX}")
set(CMAKE_DEBUGGER "${ARM_TOOLCHAIN_BIN_PATH}/arm-none-eabi-gdb${TOOL_EXECUTABLE_SUFFIX}")

# Configure for cross-compilation
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Search path configuration
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

message(STATUS "Using ARM C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "Using ARM C++ Compiler: ${CMAKE_CXX_COMPILER}")
