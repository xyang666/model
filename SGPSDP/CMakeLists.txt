cmake_minimum_required(VERSION 3.10)
project(SGPSDP)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(${CMAKE_SOURCE_DIR}/include)

set(SOURCES
    src/main.cpp
    src/SGP_SDP.cpp
)

add_executable(sgpsdp ${SOURCES})
