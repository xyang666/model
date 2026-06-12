///
/// @file      Logger.hpp
/// @brief     Minimal logging utility.

#pragma once

#include <iostream>
#include <sstream>

template<typename... Args>
inline void aWarning(Args&&... args)
{
    std::ostringstream oss;
    int unused[] = {0, (oss << std::forward<Args>(args), 0)...};
    (void)unused;
    std::cerr << "[WARNING] " << oss.str() << std::endl;
}

template<typename... Args>
inline void aError(Args&&... args)
{
    std::ostringstream oss;
    int unused[] = {0, (oss << std::forward<Args>(args), 0)...};
    (void)unused;
    std::cerr << "[ERROR] " << oss.str() << std::endl;
}
