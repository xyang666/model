cmake_minimum_required(VERSION 3.10)
project(Astronomy)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(include)

# Source files
set(SOURCES
    src/main.cpp
    src/coord_transform.cpp
    src/erfa.c
)

# Executable
add_executable(exe ${SOURCES})

# 链接数学库
target_link_libraries(exe m)