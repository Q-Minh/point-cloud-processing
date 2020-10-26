#pragma once

#include <type_traits>

namespace pcp {
namespace traits {

template <class Point, class = void>
struct is_point : std::false_type
{
};

template <class Point>
struct is_point<
    Point,
    std::void_t<
        typename Point::coordinate_type,
        decltype(Point{
            typename Point::coordinate_type{},
            typename Point::coordinate_type{},
            typename Point::coordinate_type{}}),
        decltype(std::declval<Point&>().x()),
        decltype(std::declval<Point&>().y()),
        decltype(std::declval<Point&>().z()),
        decltype(std::declval<Point::coordinate_type>() * std::declval<Point&>()),
        decltype(std::declval<Point&>() / std::declval<Point::coordinate_type>()),
        decltype(std::declval<Point&>() + std::declval<Point&>()),
        decltype(std::declval<Point&>() - std::declval<Point&>()),
        decltype(std::declval<Point&>() == std::declval<Point&>()),
        decltype(std::declval<Point&>() != std::declval<Point&>())>> : std::true_type
{
};

template <class Point>
static constexpr bool is_point_v = is_point<Point>::value;

} // namespace traits
} // namespace pcp