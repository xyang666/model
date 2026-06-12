cmake_minimum_required(VERSION 3.14)
project(ImageProcessor CXX)

# ImageProcessor
# ---------------
# 图像目标探测、跟踪与识别处理模型。
# 对应 AFSIM WsfImageProcessor 的功能范畴：
#   - 图像/视频目标检测
#   - 目标航迹管理（创建/更新/回收）
#   - 目标识别状态机（Johnson 准则）
#   - 可配置滤波（复用 TargetMeasurementDataProcessor 的滤波器库）
# 生成静态核心库、DLL 插件库和演示可执行文件。

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    add_compile_options(/utf-8)
endif()

# ── 第三方依赖路径 ──────────────────────────────────────────────
set(TM_DIR      ${CMAKE_CURRENT_SOURCE_DIR}/../TargetMeasurementDataProcessor)
set(EIGEN_DIR   ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/eigen-3.4.0)

# ── 静态核心库 ───────────────────────────────────────────────────
add_library(ip_core STATIC
    src/ImageProcessor.cpp
)

target_include_directories(ip_core
    PUBLIC
    include
    ${TM_DIR}/include
    ${EIGEN_DIR}
)

# ── 动态链接库 ───────────────────────────────────────────────────
add_library(ip_dll SHARED
    src/Plugin.cpp
    src/Framework.cpp
)

target_link_libraries(ip_dll
    PRIVATE ip_core
)

target_include_directories(ip_dll
    PRIVATE
    include
    ${TM_DIR}/include
    ${EIGEN_DIR}
)

target_compile_definitions(ip_dll
    PRIVATE
    IP_EXPORTS
)

# ── 演示程序 ─────────────────────────────────────────────────────
add_executable(ip_demo
    src/main.cpp
    src/Plugin.cpp
    src/Framework.cpp
)

target_include_directories(ip_demo
    PRIVATE
    include
    ${TM_DIR}/include
    ${EIGEN_DIR}
)

target_compile_definitions(ip_demo
    PRIVATE
    IP_EXPORTS
)

target_link_libraries(ip_demo
    PRIVATE ip_core
)
