///
/// @file      Config.hpp
/// @brief     Solver library configuration and common types.

#pragma once

#include <cstddef>

/// @brief Always-inline hint
#ifdef _MSC_VER
#  define SOLVER_INLINE __forceinline
#else
#  define SOLVER_INLINE __attribute__((always_inline)) inline
#endif

/// @brief Disable copy constructor and assignment
#define SOLVER_NO_COPY(ClassName)         \
    ClassName(const ClassName&) = delete; \
    ClassName& operator=(const ClassName&) = delete;

/// @brief Namespace macros
#define SOLVER_NS_BEGIN namespace Solver {
#define SOLVER_NS_END   }
#define SOLVER_NS       Solver

/// @brief Error code type
using errc_t = int;
constexpr errc_t eNoError              = 0;  ///< Success
constexpr errc_t eErrorMaxIter         = 1;  ///< Max iteration / step-attempt limit reached
constexpr errc_t eErrorDimension       = 2;  ///< Dimension mismatch (e.g., state vector vs ODE)
constexpr errc_t eErrorInvalidArgument = 3;  ///< Invalid argument (e.g., root not bracketed)
