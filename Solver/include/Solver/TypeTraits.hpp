///
/// @file      TypeTraits.hpp
/// @brief     Type traits utilities (is_callable).

#pragma once

#include <type_traits>

/// @brief Check if F is callable with given argument types.
template <typename F, typename... Args>
struct is_callable
{
private:
    template <typename U>
    static auto test(int) -> decltype(std::declval<U>()(std::declval<Args>()...), std::true_type());

    template <typename>
    static std::false_type test(...);

public:
    static constexpr bool value = decltype(test<F>(0))::value;
};
