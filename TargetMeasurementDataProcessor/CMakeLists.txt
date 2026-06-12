cmake_minimum_required(VERSION 3.14)
project(TargetMeasurementDataProcessingModel CXX)

# TargetMeasurementDataProcessingModel
# -----------------------------------
# 目标测量数据处理模型的构建脚本。
# 生成静态核心库、DLL 插件库和演示可执行文件。

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    add_compile_options(/utf-8)
endif()

add_library(tmdp_core STATIC
    src/tracker/Tracker.cpp
    src/tracker/TrackerUtils.cpp
    src/ProcessingPipeline.cpp
    src/tracker/RadarTracker.cpp
    src/tracker/EOTracker.cpp
    src/tracker/IRTracker.cpp
    src/CoordinateTransform.cpp
)

target_include_directories(tmdp_core
    PUBLIC
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/eigen-3.4.0
)

add_library(tmdp_dll SHARED
    src/Plugin.cpp
    src/Framework.cpp
)

target_link_libraries(tmdp_dll
    PRIVATE tmdp_core
)

target_include_directories(tmdp_dll
    PRIVATE
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/eigen-3.4.0
)

target_compile_definitions(tmdp_dll
    PRIVATE
    TM_EXPORTS
)

add_executable(tmdp_demo
    src/main.cpp
)

target_include_directories(tmdp_demo
    PRIVATE
    include
)

target_link_libraries(tmdp_demo
    PRIVATE tmdp_dll
)
