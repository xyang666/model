cmake_minimum_required(VERSION 3.14)
project(ESM_Core CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# MSVC: treat source files as UTF-8 and use UTF-8 for execution character set
if(MSVC)
    add_compile_options(/utf-8)
endif()

# --------------------------------------------------------------------------
# Eigen (header-only, adjust path if needed)
# --------------------------------------------------------------------------
set(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/eigen-3.4.0"
    CACHE PATH "Path to Eigen3 root directory")

# --------------------------------------------------------------------------
# Library: esm_core
# --------------------------------------------------------------------------
add_library(esm_core
    src/AntennaPattern.cpp
    src/Antenna.cpp
    src/ESM_Interaction.cpp
    src/ESM_Sensor.cpp
    src/ESM_Reporting.cpp
    src/ESM_ErrorModel.cpp
    src/ESM_TrackManager.cpp
)

target_include_directories(esm_core
    PUBLIC
        include
        ${EIGEN3_INCLUDE_DIR}
)

# --------------------------------------------------------------------------
# DLL: esm_core_dll（电子侦察模型封装）
# --------------------------------------------------------------------------
add_library(esm_core_dll SHARED
    src/ESM_Plugin.cpp
    src/ESM_Framework.cpp
)

target_link_libraries(esm_core_dll PRIVATE esm_core)

target_include_directories(esm_core_dll
    PRIVATE
        include
        ${EIGEN3_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_definitions(esm_core_dll PRIVATE MODELTEMPLATE_EXPORTS)

# --------------------------------------------------------------------------
# Executable: esm_demo
# --------------------------------------------------------------------------
add_executable(esm_demo src/main.cpp)

target_link_libraries(esm_demo PRIVATE esm_core)
