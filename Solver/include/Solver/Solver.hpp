///
/// @file      Solver.hpp
/// @brief     Solver library — umbrella header.
/// @details   Include this single header to access all Solver functionality.

#pragma once

#include "Solver/Config.hpp"
#include "Solver/OrdinaryDifferentialEquation.hpp"
#include "Solver/ODEIntegrator.hpp"
#include "Solver/ODEFixedStepIntegrator.hpp"
#include "Solver/ODEVarStepIntegrator.hpp"
#include "Solver/ODEWorkspace.hpp"
#include "Solver/ODEEventDetector.hpp"
#include "Solver/ODEStateObserver.hpp"

// Fixed-step integrators
#include "Solver/Impl/RK4.hpp"
#include "Solver/Impl/RK8.hpp"
#include "Solver/Impl/RKV8.hpp"
#include "Solver/Impl/ABM.hpp"

// Adaptive integrators
#include "Solver/Impl/RKCK.hpp"
#include "Solver/Impl/RKF45.hpp"
#include "Solver/Impl/RKF56.hpp"
#include "Solver/Impl/RKF78.hpp"

// Observers
#include "Solver/StateObserver/ODEStateVectorCollector.hpp"
#include "Solver/StateObserver/ODEEventDetectorList.hpp"
#include "Solver/StateObserver/ODEEventObserver.hpp"
#include "Solver/StateObserver/ODEInnerStateObserver.hpp"
#include "Solver/StateObserver/ODEStateObserverList.hpp"
