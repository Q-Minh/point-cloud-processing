#pragma once

#include <pcp/common/intersections.hpp>
#include <type_traits>

namespace pcp {
namespace traits {

/**
 * Traits type for checking Range concept.
 * A Range is a type that can be queried for containment
 * of points and intersection with another Range.
 * A Range r1 should satisfy these operations on a Point p and another Range r2:
 * - r1.contains(p) // convertible to bool
 * - intersections::intersects(r1, r2) // convertible to bool
 */
template <class Range, class Point, class = void>
struct is_range : std::false_type
{
};

template <class Range, class Point>
struct is_range<
    Range,
    Point,
    std::void_t<
        decltype(std::declval<Range&>().contains(std::declval<Point>())),
        decltype(intersections::intersects(std::declval<Range>(), std::declval<Range>()))>>
    : std::true_type
{
};

template <class Range, class Point>
static constexpr bool is_range_v = is_range<Range, Point>::value;

} // namespace traits
} // namespace pcp
