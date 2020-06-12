cmake_minimum_required(VERSION 3.10)

project(std32f1-printf)

enable_language(C ASM)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(STM32CUBE_DIR "/home/deokhwan/Workspace/STM32/f1/STM32CubeF1/")

set(STM32CUBE_GENERATED_FILES
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_adc.c
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_crc.c
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_dac.c
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_dma.c
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_exti.c
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_fsmc.c
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_gpio.c
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_i2c.c
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_pwr.c
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_rcc.c
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_rtc.c
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_sdmmc.c
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_spi.c
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_tim.c
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_usart.c
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_usb.c
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_utils.c

        #${STM32CUBE_DIR}/BSP/Src/main.c
        #${STM32CUBE_DIR}/BSP/Src/stm32f1xx_hal_msp.c
        #${STM32CUBE_DIR}/BSP/Src/stm32f1xx_it.c
        #${STM32CUBE_DIR}/BSP/Src/system_stm32f1xx.c

        #${STM32CUBE_DIR}/BSP/startup_stm32f103xb.s
        )

set(SOURCES
        Src/main.c
        Src/uart.c
        Src/led.c
        Src/motor.c
        Src/battery.c
        #Src/stm32f1xx_hal_msp.c
        Src/stm32f1xx_it.c
        Src/system_stm32f1xx.c
        SW4STM32/startup_stm32f103xb.s
        #SW4STM32/syscalls.c
        )


set(EXECUTABLE ${PROJECT_NAME}.exe)

set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/SW4STM32/STM32F103RB-Nucleo/STM32F103RBTx_FLASH.ld")
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -lc -lm -lnosys -specs=nosys.specs -specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T ${LINKER_SCRIPT}")

add_executable(${EXECUTABLE} ${SOURCES} ${STM32CUBE_GENERATED_FILES})

set_target_properties(${TARGET_NAME} PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT})

target_compile_definitions(${EXECUTABLE} PRIVATE
        #-DUSE_HAL_DRIVER
        -DUSE_FULL_LL_DRIVER
        -DUSE_FULL_ASSERT
        -DSTM32F103xB
        #-D__GNUC__
        )

target_include_directories(${EXECUTABLE} PRIVATE
        ${STM32CUBE_DIR}/Drivers/STM32F1xx_HAL_Driver/Inc
        ${STM32CUBE_DIR}/Drivers/CMSIS/Device/ST/STM32F1xx/Include
        ${STM32CUBE_DIR}/Drivers/CMSIS/Include
        ${CMAKE_SOURCE_DIR}/Inc
        )

target_compile_options(${EXECUTABLE} PUBLIC
        -mcpu=cortex-m3
        -mthumb
        #-mfpu=fpv4-sp-d16  # it is only valid for F4
        -mfloat-abi=soft
        -mlittle-endian

        -fdata-sections
        -ffunction-sections

        -Wall
        -Werror
        #-Wstrict-prototypes
        -Warray-bounds
        -fno-strict-aliasing
        -Wno-unused-const-variable
        -specs=nano.specs
        -specs=nosys.specs

        -O0
        -g

        #$<$<CONFIG:Debug>:-Og>
        )

    #target_link_options(${EXECUTABLE} PRIVATE
    #        -T${CMAKE_SOURCE_DIR}/SW4STM32/STM32F103RB_Nucleo/STM32F103VBIx_FLASH.ld
    #        -mcpu=cortex-m3
    #        -mthumb
    #        #-mfpu=fpv4-sp-d16
    #        -mfloat-abi=soft
    #        -specs=nano.specs
    #        -lc
    #        -lm
    #        -lnosys
    #        -Wl,-Map=${PROJECT_NAME}.map,--cref
    #        -Wl,--gc-sections
    #        )

# Print executable size
add_custom_command(TARGET ${EXECUTABLE}
        POST_BUILD
        COMMAND arm-none-eabi-size ${EXECUTABLE})

# Create hex file
add_custom_command(TARGET ${EXECUTABLE}
        POST_BUILD
        COMMAND arm-none-eabi-objcopy -O ihex ${EXECUTABLE} ${PROJECT_NAME}.hex
        COMMAND arm-none-eabi-objcopy -O binary ${EXECUTABLE} ${PROJECT_NAME}.bin)
