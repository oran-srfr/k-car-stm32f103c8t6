cmake_minimum_required(VERSION 3.22)
project(k_car LANGUAGES C ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(HAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver")
add_executable(k_car
    Core/Src/main.c
    Core/Src/app.c
    Core/Src/app_modules.c
    Core/Src/ultrasonic.c
    Core/Src/ultrasonic_stm32.c
    Core/Src/newlib_io.c
    Core/Src/stm32f1xx_it.c
    Core/Src/system_stm32f1xx.c
    Core/Startup/startup_stm32f103xb.s
    ${HAL_DIR}/Src/stm32f1xx_hal.c
    ${HAL_DIR}/Src/stm32f1xx_hal_cortex.c
    ${HAL_DIR}/Src/stm32f1xx_hal_dma.c
    ${HAL_DIR}/Src/stm32f1xx_hal_exti.c
    ${HAL_DIR}/Src/stm32f1xx_hal_flash.c
    ${HAL_DIR}/Src/stm32f1xx_hal_flash_ex.c
    ${HAL_DIR}/Src/stm32f1xx_hal_gpio.c
    ${HAL_DIR}/Src/stm32f1xx_hal_gpio_ex.c
    ${HAL_DIR}/Src/stm32f1xx_hal_pwr.c
    ${HAL_DIR}/Src/stm32f1xx_hal_rcc.c
    ${HAL_DIR}/Src/stm32f1xx_hal_rcc_ex.c
)
set_target_properties(k_car PROPERTIES SUFFIX ".elf")
target_compile_definitions(k_car PRIVATE STM32F103xB USE_HAL_DRIVER)
target_include_directories(k_car PRIVATE
    Core/Inc
    Drivers/CMSIS/Include
    Drivers/CMSIS/Device/ST/STM32F1xx/Include
    ${HAL_DIR}/Inc
    ${HAL_DIR}/Inc/Legacy
)
target_compile_options(k_car PRIVATE
    -Wall -Wextra -ffunction-sections -fdata-sections
    $<$<CONFIG:Debug>:-Og>
    $<$<CONFIG:Debug>:-g3>
)
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32F103C8Tx_FLASH.ld")
set_property(TARGET k_car APPEND PROPERTY LINK_DEPENDS "${LINKER_SCRIPT}")
target_link_options(k_car PRIVATE
    "-T${LINKER_SCRIPT}"
    --specs=nano.specs --specs=nosys.specs
    -Wl,--gc-sections
    "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/k_car.map"
    -Wl,--print-memory-usage
)
add_custom_command(TARGET k_car POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:k_car> ${CMAKE_CURRENT_BINARY_DIR}/k_car.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:k_car> ${CMAKE_CURRENT_BINARY_DIR}/k_car.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:k_car>
    VERBATIM
)
