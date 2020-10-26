#pragma once

#include <type_traits>

namespace pcp {
namespace traits {

template <class Normal, class = void>
struct is_normal : std::false_type
{
};

template <class Normal>
struct is_normal<
    Normal,
    std::void_t<
        typename Normal::coordinate_type,
        decltype(Normal{
            typename Normal::coordinate_type{},
            typename Normal::coordinate_type{},
            typename Normal::coordinate_type{}}),
        decltype(std::declval<Normal&>().x()),
        decltype(std::declval<Normal&>().y()),
        decltype(std::declval<Normal&>().z()),
        decltype(std::declval<typename Normal::coordinate_type>() * std::declval<Normal&>()),
        decltype(std::declval<Normal&>() / std::declval<typename Normal::coordinate_type>()),
        decltype(std::declval<Normal&>() + std::declval<Normal&>()),
        decltype(std::declval<Normal&>() - std::declval<Normal&>()),
        decltype(std::declval<Normal&>() == std::declval<Normal&>()),
        decltype(std::declval<Normal&>() != std::declval<Normal&>())>> : std::true_type
{
};

template <class Normal>
static constexpr bool is_normal_v = is_normal<Normal>::value;

} // namespace traits
} // namespace pcp