cmake_minimum_required(VERSION 3.15)

project(nucleo32g0 LANGUAGES C CXX ASM)

find_bsp(
	ID com.sysprogs.arm.stm32
	VERSION 2024.07
	MCU STM32G031K8
	HWREGISTER_LIST_FILE STM32G0xxxx/DeviceDefinitions/stm32g031xx.xml
	DISABLE_GNU_EXTENSIONS)

register_imported_project(NAME nucleo32g0
	PATH ../.cproject
	LEVEL 0)
add_bsp_based_executable(
	NAME nucleo32g0
	LINKER_SCRIPT ${nucleo32g0_LOCATION}/STM32G031K8TX_FLASH.ld
	SOURCES
		${nucleo32g0_LOCATION}/Core/Inc/main.h
		${nucleo32g0_LOCATION}/Core/Inc/stm32g0xx_hal_conf.h
		${nucleo32g0_LOCATION}/Core/Inc/stm32g0xx_it.h
		${nucleo32g0_LOCATION}/Core/Src/main.c
		${nucleo32g0_LOCATION}/Core/Src/stm32g0xx_hal_msp.c
		${nucleo32g0_LOCATION}/Core/Src/stm32g0xx_it.c
		${nucleo32g0_LOCATION}/Core/Src/syscalls.c
		${nucleo32g0_LOCATION}/Core/Src/sysmem.c
		${nucleo32g0_LOCATION}/Core/Src/system_stm32g0xx.c
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Device/ST/STM32G0xx/Include/stm32g031xx.h
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Device/ST/STM32G0xx/Include/stm32g0xx.h
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Device/ST/STM32G0xx/Include/system_stm32g0xx.h
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Include/cmsis_armcc.h
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Include/cmsis_armclang.h
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Include/cmsis_armclang_ltm.h
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Include/cmsis_compiler.h
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Include/cmsis_gcc.h
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Include/cmsis_iccarm.h
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Include/cmsis_version.h
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Include/core_armv81mml.h
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Include/core_armv8mbl.h
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Include/core_armv8mml.h
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Include/core_cm0.h
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Include/core_cm0plus.h
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Include/core_cm1.h
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Include/core_cm23.h
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Include/core_cm3.h
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Include/core_cm33.h
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Include/core_cm35p.h
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Include/core_cm4.h
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Include/core_cm7.h
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Include/core_sc000.h
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Include/core_sc300.h
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Include/mpu_armv7.h
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Include/mpu_armv8.h
		${nucleo32g0_LOCATION}/Drivers/CMSIS/Include/tz_context.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_hal.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_hal_cortex.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_hal_def.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_hal_dma.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_hal_dma_ex.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_hal_exti.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_hal_flash.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_hal_flash_ex.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_hal_gpio.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_hal_gpio_ex.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_hal_pwr.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_hal_pwr_ex.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_hal_rcc.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_hal_rcc_ex.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_hal_tim.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_hal_tim_ex.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_hal_uart.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_hal_uart_ex.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_ll_bus.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_ll_cortex.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_ll_dma.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_ll_dmamux.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_ll_exti.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_ll_gpio.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_ll_lpuart.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_ll_pwr.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_ll_rcc.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_ll_system.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_ll_usart.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/stm32g0xx_ll_utils.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal.c
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_cortex.c
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_dma.c
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_dma_ex.c
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_exti.c
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_flash.c
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_flash_ex.c
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_gpio.c
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_pwr.c
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_pwr_ex.c
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_rcc.c
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_rcc_ex.c
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_tim.c
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_tim_ex.c
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_uart.c
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_hal_uart_ex.c
		${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_ll_rcc.c
	GENERATE_BIN
	GENERATE_MAP)
target_include_directories(nucleo32g0
	PRIVATE
	${nucleo32g0_LOCATION}/Core/Inc
	${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc
	${nucleo32g0_LOCATION}/Drivers/STM32G0xx_HAL_Driver/Inc/Legacy
	${nucleo32g0_LOCATION}/Drivers/CMSIS/Device/ST/STM32G0xx/Include
	${nucleo32g0_LOCATION}/Drivers/CMSIS/Include)

target_compile_definitions(nucleo32g0
	PRIVATE
	USE_HAL_DRIVER
	STM32G031xx)

set(ignoreMe "${LOCALAPPDATA}")
