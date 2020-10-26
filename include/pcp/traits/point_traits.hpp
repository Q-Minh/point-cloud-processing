#pragma once

#include <type_traits>

namespace pcp {
namespace traits {

template <class PointView, class = void>
struct is_point_view : std::false_type
{
};

template <class PointView>
struct is_point_view<
    PointView,
    std::void_t<
        typename PointView::coordinate_type,
        decltype(std::declval<PointView&>().x()),
        decltype(std::declval<PointView&>().y()),
        decltype(std::declval<PointView&>().z()),
        decltype(std::declval<PointView&>() == std::declval<PointView&>()),
        decltype(std::declval<PointView&>() != std::declval<PointView&>())>> : std::true_type
{
};

template <class PointView>
static constexpr bool is_point_view_v = is_point_view<PointView>::value;

template <class Point, class = void>
struct is_point : std::false_type
{
};

template <class Point>
struct is_point<
    Point,
    std::void_t<
        decltype(std::declval<typename Point::coordinate_type>() * std::declval<Point&>()),
        decltype(std::declval<Point&>() / std::declval<typename Point::coordinate_type>()),
        decltype(std::declval<Point&>() + std::declval<Point&>()),
        decltype(std::declval<Point&>() - std::declval<Point&>())>> : is_point_view<Point>
{
};

template <class Point>
static constexpr bool is_point_v = is_point<Point>::value;

} // namespace traits
} // namespace pcp
