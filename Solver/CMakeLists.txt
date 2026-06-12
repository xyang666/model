cmake_minimum_required(VERSION 3.10)
project(Solver VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(include)

# Library sources
set(SOLVER_SOURCES
    src/ODEIntegrator.cpp
    src/ODEFixedStepIntegrator.cpp
    src/ODEVarStepIntegrator.cpp
    src/Impl/RK4.cpp
    src/Impl/RK8.cpp
    src/Impl/RKV8.cpp
    src/Impl/ABM.cpp
    src/Impl/RKCK.cpp
    src/Impl/RKF45.cpp
    src/Impl/RKF56.cpp
    src/Impl/RKF78.cpp
    src/StateObserver/ODEStateVectorCollector.cpp
    src/StateObserver/ODEInnerStateObserver.cpp
    src/StateObserver/ODEStateObserverList.cpp
    src/StateObserver/ODEEventDetectorList.cpp
    src/StateObserver/ODEEventObserver.cpp
)

# Static library
add_library(Solver STATIC ${SOLVER_SOURCES})

# Examples
add_executable(01-harmonic-oscillator examples/01-harmonic-oscillator.cpp)
target_link_libraries(01-harmonic-oscillator Solver)

add_executable(02-adaptive-comparison examples/02-adaptive-comparison.cpp)
target_link_libraries(02-adaptive-comparison Solver)

add_executable(03-event-detection examples/03-event-detection.cpp)
target_link_libraries(03-event-detection Solver)

