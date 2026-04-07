cmake_minimum_required(VERSION 3.10)
project(XXXModel)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(include)
include_directories("../3rdparty/eigen-3.4.0")

# Source files
set(SOURCES
    src/main.cpp
    src/SGP4.cpp
    src/erfa.c
)

# Executable
add_executable(SGP4 ${SOURCES})