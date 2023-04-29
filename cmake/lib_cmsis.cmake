include(CMakePrintHelpers)
include(FetchContent)

set(GITHUB_BRANCH_CMSIS "5.9.0")
set(GITHUB_BRANCH_CMSIS_DSP "v1.10.1")
cmake_print_variables(GITHUB_BRANCH_CMSIS)

FetchContent_Declare(
    cmsis                             # Recommendation: Stick close to the original name.
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    URL https://github.com/ARM-software/CMSIS_5/archive/refs/tags/${GITHUB_BRANCH_CMSIS}.tar.gz
    URL_HASH MD5=6b67968b5a3540156a4bd772d899339e
)

FetchContent_GetProperties(cmsis)

if(NOT cmsis_POPULATED)
    FetchContent_Populate(cmsis)
    add_library(cmsis INTERFACE)
    target_link_libraries(cmsis
        INTERFACE
            CMSISDSP
    )
    # This is just a header based interface used in various upstream libraries including HAL
    add_library(cmsis-core INTERFACE)
    target_include_directories(cmsis-core
        INTERFACE
        ${cmsis_SOURCE_DIR}/CMSIS/Core/Include
    )
endif()


if(USE_CMSIS_DSP)
    cmake_print_variables(GITHUB_BRANCH_CMSIS_DSP)

    FetchContent_Declare(
        cmsis_dsp                          # Recommendation: Stick close to the original name.
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        URL https://github.com/ARM-software/CMSIS-DSP/archive/refs/tags/${GITHUB_BRANCH_CMSIS_DSP}.tar.gz
        URL_HASH MD5=f3387e91b178806347c614dc26813472
    )

    FetchContent_GetProperties(cmsis_dsp)

    if(NOT cmsis_dsp_POPULATED)
        FetchContent_Populate(cmsis_dsp)
        set(cmsisDSPProject "CMSISDSP")
        set(CMSISCORE "${cmsis_SOURCE_DIR}/CMSIS/Core" CACHE STRING "CMSIS Core Includes" FORCE)
        cmake_print_variables(CMSISCORE)
        # Library cmsis.a is in the /build/_deps/cmsis-build directory
        if(CMAKE_SYSTEM_PROCESSOR STREQUAL  "ARM")
            list(APPEND CMAKE_MODULE_PATH ${cmsis_dsp_SOURCE_DIR})

            # BANSHI specific CMSIS customization, helps to limit the tables to be imported to the flash memory
            set(BASICMATH "ON"        CACHE BOOL "CMSIS-DSP basic math functions")
            set(COMPLEXMATH "OFF"     CACHE BOOL "CMSIS-DSP complex math functions")
            set(CONTROLLER "OFF"      CACHE BOOL "CMSIS-DSP controller functions")
            set(FASTMATH "ON"         CACHE BOOL "CMSIS-DSP fast math functions")
            set(FILTERING "OFF"       CACHE BOOL "CMSIS-DSP filtering functions")
            set(MATRIX "ON"           CACHE BOOL "CMSIS-DSP matrices functions")
            set(STATISTICS "OFF"      CACHE BOOL "CMSIS-DSP statistics functions")
            set(SUPPORT "ON"          CACHE BOOL "CMSIS-DSP support functions")
            set(TRANSFORM "OFF"       CACHE BOOL "CMSIS-DSP transformation functions")
            set(SVM "OFF"             CACHE BOOL "CMSIS-DSP SVM functions")
            set(BAYES "OFF"           CACHE BOOL "CMSIS-DSP Bayes functions")
            set(DISTANCE "OFF"        CACHE BOOL "CMSIS-DSP distance functions")
            set(INTERPOLATION "OFF"   CACHE BOOL "CMSIS-DSP interpolation functions")
            set(QUATERNIONMATH "OFF"  CACHE BOOL "CMSIS-DSP qaterionmath functions")
            set(CONFIGTABLE "ON"      CACHE BOOL "CMSIS-DSP configtable functions")
            set(ARM_SIN_F32 "ON"      CACHE BOOL "CMSIS-DSP arm-sin floating 32 bit tables and functions")
            set(ARM_COS_F32 "ON"      CACHE BOOL "CMSIS-DSP arm-cos floating 32 bit tables and functions")

            # This is where the CMSIS DSP library is first generated. It's exclusive property of cmsis as  part of interface
            add_subdirectory(${cmsis_dsp_SOURCE_DIR}/Source bin_dsp)
            setTargetCompileOptions(cmsisDSPProject)
            target_link_libraries(CMSISDSP INTERFACE cmsis-core)
        endif()
    endif()
else()
    message(STATUS "CMSIS DSP is not used in the project")
endif(USE_CMSIS_DSP)
