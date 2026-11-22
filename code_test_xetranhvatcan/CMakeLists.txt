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
set(CMAKE_PROJECT_NAME code_test_xetranhvatcan)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})
option(TEACH_DRIVE "Build the Bluetooth teach/replay firmware" OFF)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})
if(TEACH_DRIVE)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE TEACH_DRIVE=1)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE Src/teach_drive.c Src/teach_storage.c)
    # Define the reservation before the linker evaluates the MEMORY block.
    set(CMAKE_EXE_LINKER_FLAGS "-Wl,--defsym=teach_reserved=2048 ${CMAKE_EXE_LINKER_FLAGS}")
endif()

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)
# Share code across the firmware modules to fit the STM32F103's 64 KB flash.
set_property(TARGET ${CMAKE_PROJECT_NAME} STM32_Drivers PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
set_source_files_properties(
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
    Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
    TARGET_DIRECTORY STM32_Drivers PROPERTIES COMPILE_OPTIONS "-Os")

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Src/software_uart.c
    Src/vl53l0x.c
    Src/motor.c
    Src/board_drive.c
    Src/motor_bench.c
    Src/mpu6050.c
    Src/motion.c
    Src/front_clearance.c
    Src/grid_map.c
    Src/navigation.c
)

# Keep the map and navigation within the 64 KB flash budget in Debug.
set_source_files_properties(Src/grid_map.c PROPERTIES COMPILE_OPTIONS "-Os")
set_source_files_properties(Src/navigation.c PROPERTIES COMPILE_OPTIONS "-Oz")
set_source_files_properties(Src/main.c PROPERTIES COMPILE_OPTIONS "-Os")
set_source_files_properties(Src/motion.c PROPERTIES COMPILE_OPTIONS "-Os")
set_source_files_properties(Src/vl53l0x.c PROPERTIES COMPILE_OPTIONS "-Os")
set_source_files_properties(Src/software_uart.c Src/board_drive.c
    Src/stm32f1xx_hal_msp.c Src/stm32f1xx_it.c
    PROPERTIES COMPILE_OPTIONS "-Os")
set_source_files_properties(Src/mpu6050.c Src/motor.c Src/front_clearance.c Src/motor_bench.c
    PROPERTIES COMPILE_OPTIONS "-Os")

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    m
)

# Post-build: Create .hex and .bin files for flashing via ST-Link
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "Building .hex and .bin for ST-Link flashing"
)
