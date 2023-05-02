
if(NOT (CMAKE_SYSTEM_PROCESSOR STREQUAL "arm"))
    message(FATAL_ERROR "${PROJECT_NAME} can only compile with a suitable ARM cross compiler; no target build.")
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CMakePrintHelpers)

add_library(${PROJECT_NAME} STATIC)
add_library(${PROJECT_NAME}::framework ALIAS ${PROJECT_NAME})

include(cmake/lib_cmsis.cmake)
set(cmsis_DEVICE_INCLUDE_DIR "${cmsis_SOURCE_DIR}/Device/ARM/ARMCM4/Include" CACHE STRING "Path to ARM CM4 from standard CMSIS")
set(cmsis_CORE_INCLUDE_DIR "${cmsis_SOURCE_DIR}/CMSIS/Core/Include" CACHE STRING "Path to CMSIS core includes")

set(hal_core_INCLUDES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/NUCLEO-STM32G431RB-MC/Inc")
set(hal_core_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/NUCLEO-STM32G431RB-MC/Src")
set(hal_core_SOURCES

    ${hal_core_SOURCE_DIR}/aspep.c
    ${hal_core_SOURCE_DIR}/main.c
    ${hal_core_SOURCE_DIR}/mc_api.c
    ${hal_core_SOURCE_DIR}/mc_app_hooks.c
    ${hal_core_SOURCE_DIR}/mc_config.c
    ${hal_core_SOURCE_DIR}/mc_configuration_registers.c
    ${hal_core_SOURCE_DIR}/mc_interface.c
    ${hal_core_SOURCE_DIR}/mc_math.c
    ${hal_core_SOURCE_DIR}/mc_parameters.c
    ${hal_core_SOURCE_DIR}/mc_perf.c
    ${hal_core_SOURCE_DIR}/mc_tasks.c
    ${hal_core_SOURCE_DIR}/mcp_config.c
    ${hal_core_SOURCE_DIR}/motorcontrol.c
    ${hal_core_SOURCE_DIR}/pwm_curr_fdbk.c
    ${hal_core_SOURCE_DIR}/register_interface.c
    ${hal_core_SOURCE_DIR}/regular_conversion_manager.c
    ${hal_core_SOURCE_DIR}/stm32g4xx_hal_msp.c
    ${hal_core_SOURCE_DIR}/system_stm32g4xx.c
    ${hal_core_SOURCE_DIR}/usart_aspep_driver.c
)

set(hal_drivers_CMSIS_device_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/NUCLEO-STM32G431RB-MC/Drivers/CMSIS/Device/ST/STM32G4xx/Include")
set(hal_drivers_CMSIS_INCLUDE_DIR "${cmake_SOURCE_DIR}/CMSIS/Core/Include")
set(hal_drivers_legacy_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/NUCLEO-STM32G431RB-MC/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy")
set(hal_drivers_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/NUCLEO-STM32G431RB-MC/Drivers/STM32G4xx_HAL_Driver/Inc")
set(hal_drivers_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/NUCLEO-STM32G431RB-MC/Drivers/STM32G4xx_HAL_Driver/Src")
set(hal_drivers_SOURCES
    ${hal_drivers_SOURCE_DIR}/stm32g4xx_hal.c
    ${hal_drivers_SOURCE_DIR}/stm32g4xx_hal_adc.c
    ${hal_drivers_SOURCE_DIR}/stm32g4xx_hal_adc_ex.c
    ${hal_drivers_SOURCE_DIR}/stm32g4xx_hal_cordic.c
    ${hal_drivers_SOURCE_DIR}/stm32g4xx_hal_cortex.c
    ${hal_drivers_SOURCE_DIR}/stm32g4xx_hal_dma.c
    ${hal_drivers_SOURCE_DIR}/stm32g4xx_hal_dma_ex.c
    ${hal_drivers_SOURCE_DIR}/stm32g4xx_hal_exti.c
    ${hal_drivers_SOURCE_DIR}/stm32g4xx_hal_fdcan.c
    ${hal_drivers_SOURCE_DIR}/stm32g4xx_hal_flash.c
    ${hal_drivers_SOURCE_DIR}/stm32g4xx_hal_flash_ex.c
    ${hal_drivers_SOURCE_DIR}/stm32g4xx_hal_flash_ramfunc.c
    ${hal_drivers_SOURCE_DIR}/stm32g4xx_hal_gpio.c
    ${hal_drivers_SOURCE_DIR}/stm32g4xx_hal_pwr.c
    ${hal_drivers_SOURCE_DIR}/stm32g4xx_hal_pwr_ex.c
    ${hal_drivers_SOURCE_DIR}/stm32g4xx_hal_rcc.c
    ${hal_drivers_SOURCE_DIR}/stm32g4xx_hal_rcc_ex.c
    ${hal_drivers_SOURCE_DIR}/stm32g4xx_hal_tim.c
    ${hal_drivers_SOURCE_DIR}/stm32g4xx_hal_tim_ex.c
    ${hal_drivers_SOURCE_DIR}/stm32g4xx_hal_uart.c
    ${hal_drivers_SOURCE_DIR}/stm32g4xx_hal_uart_ex.c
    ${hal_drivers_SOURCE_DIR}/stm32g4xx_ll_adc.c
)

set(mcsdk_Any_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/NUCLEO-STM32G431RB-MC/MCSDK_v6.1.1-Full/MotorControl/MCSDK/MCLib/Any/Inc")
set(mcsdk_Any_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/NUCLEO-STM32G431RB-MC/MCSDK_v6.1.1-Full/MotorControl/MCSDK/MCLib/Any/Src")
set(mcsdk_Any_SOURCES
    ${mcsdk_Any_SOURCE_DIR}/bus_voltage_sensor.c
    ${mcsdk_Any_SOURCE_DIR}/circle_limitation.c
    ${mcsdk_Any_SOURCE_DIR}/digital_output.c
    ${mcsdk_Any_SOURCE_DIR}/mcp.c
    ${mcsdk_Any_SOURCE_DIR}/mcpa.c
    ${mcsdk_Any_SOURCE_DIR}/ntc_temperature_sensor.c
    ${mcsdk_Any_SOURCE_DIR}/open_loop.c
    ${mcsdk_Any_SOURCE_DIR}/pid_regulator.c
    ${mcsdk_Any_SOURCE_DIR}/pqd_motor_power_measurement.c
    ${mcsdk_Any_SOURCE_DIR}/potentiometer.c
    ${mcsdk_Any_SOURCE_DIR}/pwm_common.c
    ${mcsdk_Any_SOURCE_DIR}/r_divider_bus_voltage_sensor.c
    ${mcsdk_Any_SOURCE_DIR}/ramp_ext_mngr.c
    ${mcsdk_Any_SOURCE_DIR}/revup_ctrl.c
    ${mcsdk_Any_SOURCE_DIR}/speed_pos_fdbk.c
    ${mcsdk_Any_SOURCE_DIR}/speed_potentiometer.c
    ${mcsdk_Any_SOURCE_DIR}/speed_torq_ctrl.c
    ${mcsdk_Any_SOURCE_DIR}/sto_pll_speed_pos_fdbk.c
    ${mcsdk_Any_SOURCE_DIR}/virtual_speed_sensor.c
)

set(mcsdk_G4xx_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/NUCLEO-STM32G431RB-MC/MCSDK_v6.1.1-Full/MotorControl/MCSDK/MCLib/G4xx/Inc")
set(mcsdk_G4xx_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/NUCLEO-STM32G431RB-MC/MCSDK_v6.1.1-Full/MotorControl/MCSDK/MCLib/G4xx/Src")
set(mcsdk_G4xx_SOURCES
    ${mcsdk_G4xx_SOURCE_DIR}/r3_2_g4xx_pwm_curr_fdbk.c
)


target_sources(${PROJECT_NAME}
    PRIVATE
    ${hal_core_SOURCES}
    ${hal_drivers_SOURCES}
    # MCSDK Sources
    ${mcsdk_Any_SOURCES}
    ${mcsdk_G4xx_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/NUCLEO-STM32G431RB-MC/STM32CubeIDE/Application/User/syscalls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/NUCLEO-STM32G431RB-MC/STM32CubeIDE/Application/User/sysmem.c
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${cmsis_CORE_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${cmsis_DEVICE_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${hal_drivers_CMSIS_device_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${hal_drivers_legacy_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${hal_drivers_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${hal_core_INCLUDES_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
        $<BUILD_INTERFACE:${mcsdk_Any_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${mcsdk_G4xx_INCLUDE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>
)

target_compile_definitions(${PROJECT_NAME}
    PUBLIC
        USE_HAL_DRIVER
        STM32G431xx
        __UVISION_VERSION="537"
        _RTE_
        ARM_MATH_CM4
)

set(${PROJECT_NAME}_PUBLIC_HEADERS
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_hal.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_hal_cortex.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_hal_def.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_hal_dma.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_hal_dma_ex.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_hal_exti.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_hal_fdcan.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_hal_flash.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_hal_flash_ex.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_hal_flash_ramfunc.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_hal_gpio.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_hal_gpio_ex.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_hal_pcd_ex.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_hal_pwr.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_hal_pwr_ex.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_hal_rcc.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_hal_rcc_ex.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_hal_tim.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_hal_tim_ex.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_ll_bus.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_ll_cortex.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_ll_crs.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_ll_dma.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_ll_dmamux.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_ll_exti.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_ll_gpio.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_ll_lpuart.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_ll_pwr.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_ll_rcc.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_ll_system.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_ll_tim.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_ll_usart.h
    ${hal_drivers_CMSIS_device_INCLUDE_DIR}/stm32g4xx_ll_utils.h

    ${hal_core_INCLUDES_DIR}/fdcan.h
    ${hal_core_INCLUDES_DIR}/gpio.h
    ${hal_core_INCLUDES_DIR}/main.h
    ${hal_core_INCLUDES_DIR}/stm32g4xx_hal_conf.h
    ${hal_core_INCLUDES_DIR}/stm32g4xx_it.h
    ${hal_core_INCLUDES_DIR}/tim.h
    ${hal_core_INCLUDES_DIR}/usart.h
)

set_target_properties(${PROJECT_NAME}
    PROPERTIES
        C_STANDARD          11
        C_STANDARD_REQUIRED ON
        C_EXTENSIONS        OFF
        PUBLIC_HEADER       "${${PROJECT_NAME}_PUBLIC_HEADERS}"
        EXPORT_NAME         framework
)

write_basic_package_version_file(${PROJECT_NAME}ConfigVersion.cmake
    VERSION       ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

setTargetCompileOptions(PROJECT_NAME)

# CPACK begins here
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
set(CPACK_BINARY_7Z ON)
set(CPACK_BINARY_NSIS OFF)
include(CPack)
