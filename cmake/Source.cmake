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
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/src/r_sci_uart/r_sci_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/src/r_ether/r_ether.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/src/r_ether_phy/r_ether_phy.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/src/r_iic_master/r_iic_master.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/src/rm_comms_i2c/rm_comms_i2c.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/src/rm_comms_i2c/rm_comms_i2c_driver_ra.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/fsp/src/r_rtc/r_rtc.c
    
    
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
    ${CMAKE_CURRENT_SOURCE_DIR}/ra_gen/ether_thread.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra_gen/sensor_thread.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra_gen/common_data.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra_gen/hal_data.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra_gen/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra_gen/pin_data.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ra_gen/vector_data.c
    
    # Application files
    ${CMAKE_CURRENT_SOURCE_DIR}/src/blinky_thread_entry.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ether_thread_entry.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor_thread_entry.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hal_warmstart.c
)

# Include directories
set(INC_FOLDERS
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/ra/board/ra6m5_ck
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