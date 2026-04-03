cmake_minimum_required(VERSION 3.10)
project(XXXModel)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(include)

# Source files
set(SOURCES
    src/main.cpp
)

# Executable
add_executable(exe ${SOURCES})