cmake_minimum_required(VERSION 3.10)
project(HPOPModel)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(include)
include_directories("../3rdparty/src")

# Source files
set(SOURCES
    src/main.cpp
)

# Executable
add_executable(hpop ${SOURCES})