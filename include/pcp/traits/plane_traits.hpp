#pragma once

#include <type_traits>

namespace pcp {
namespace traits {

template <class Plane, class = void>
struct is_plane : std::false_type
{
};

/**
 * @brief 
 * Plane concept
 * Requirements:
 * - Type members point_type and normal_type
 * - member functions normal_type normal() and point_type point()
 * - member functions void normal(normal_type) and void point(point_type)
 * - Constructor taking (point_type, normal_type) parameters
 * @tparam Plane 
*/
template <class Plane>
struct is_plane<
    Plane,
    std::void_t<
        typename Plane::point_type,
        typename Plane::normal_type,
        decltype(std::declval<Plane&>().normal()),
        decltype(std::declval<Plane&>().point()),
        decltype(std::declval<Plane&>().point(std::declval<typename Plane::point_type>())),
        decltype(std::declval<Plane&>().normal(std::declval<typename Plane::normal_type>()))>>
    : std::true_type
{
    static_assert(
        std::is_constructible_v<Plane, typename Plane::point_type, typename Plane::normal_type>,
        "Plane must be constructible from its point and normal");
};

template <class Plane>
inline bool constexpr is_plane_v = is_plane<Plane>::value;

} // namespace traits
} // namespace pcp