cmake_minimum_required(VERSION 3.25)

    project(demo
        VERSION     0.0.1
        LANGUAGES   C ASM CXX
        DESCRIPTION "Hardware Abstraction Layer for STM32CubeMx based project"
    )

    if(CMAKE_C_COMPILER_ID STREQUAL "ARMClang")
        set(${PROJECT_NAME}_STARTUP_FILE ${CMAKE_CURRENT_SOURCE_DIR}/__startup/__vectors.c CACHE STRING "ARMClang based startup script for ${PROJECT_NAME}")
    else()
        #set(${PROJECT_NAME}_STARTUP_FILE ${CMAKE_CURRENT_SOURCE_DIR}/__startup/__vectors.c CACHE STRING "ARMClang based startup script for ${PROJECT_NAME}")
        set(${PROJECT_NAME}_STARTUP_FILE ${CMAKE_SOURCE_DIR}/NUCLEO-STM32G431RB-MC/STM32CubeIDE/Application/Startup/startup_stm32g431rbtx.s )#${CMAKE_CURRENT_SOURCE_DIR}/NUCLEO-STM32G431RB-MC/STM32CubeIDE/Application/Startup/startup_stm32g431rbtx.s CACHE STRING "ARMClang based startup script for ${PROJECT_NAME}")
    endif()

    add_executable(${PROJECT_NAME})

    target_sources(${PROJECT_NAME}
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/demo.c
            ${hal_core_SOURCE_DIR}/stm32g4xx_it.c
            ${hal_core_SOURCE_DIR}/stm32g4xx_mc_it.c
        # Expose the startup file
        PUBLIC
            ${${PROJECT_NAME}_STARTUP_FILE}
    )

    target_include_directories(${PROJECT_NAME}
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}
        # Expose the linker files
        PUBLIC
            ${cmsis_SOURCE_DIR}/CMSIS/Core/Include
    )

    if(NOT CMAKE_EXECUTABLE_SUFFIX)
        set(CMAKE_EXECUTABLE_SUFFIX ".elf")
    endif()

    # Always set the linker script path and file separately
    set(${PROJECT_NAME}_LINKER_PATH "/Users/sohal/GitHub/NUCLEO-STM32G431RB-MC//NUCLEO-STM32G431RB-MC/STM32CubeIDE" CACHE STRING "Path to armgcc linker script")
    set(${PROJECT_NAME}_LINKER_SCRIPT "STM32G431RBTX_FLASH.ld" CACHE STRING "armgcc based linker script for ${PROJECT_NAME}")
    set(${PROJECT_NAME}_SCATTER_PATH "${CMAKE_CURRENT_SOURCE_DIR}/__linker" CACHE STRING "Path to ARMClang linker script")
    set(${PROJECT_NAME}_SCATTER_FILE "ac6_arm.sct" CACHE STRING "ARMClang based linker script for ${PROJECT_NAME}")
    set(${PROJECT_NAME}_LLVM_LINKER_PATH "${CMAKE_CURRENT_SOURCE_DIR}/__linker" CACHE STRING "Path to llvm linker script")
    set(${PROJECT_NAME}_LLVM_LINKER_SCRIPT "llvm_arm.ld" CACHE STRING "llvm based linker script for ${PROJECT_NAME}")

    setTargetCompileOptions(PROJECT_NAME)
    setTargetLinkOptions(PROJECT_NAME)
    target_link_libraries(${PROJECT_NAME}
        hal
        cmsis
    )

convertELF_BIN_HEX(${PROJECT_NAME})
