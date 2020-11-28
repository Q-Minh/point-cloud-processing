#pragma once

#include <algorithm>
#include <array>
#include <limits>
#include <numeric>
#include <pcp/traits/point_traits.hpp>

namespace pcp {

/**
 * @brief
 * https://en.wikipedia.org/wiki/Bounding_volume
 * @tparam Point Type satisfying Point concept
 */
template <class Point>
struct axis_aligned_bounding_box_t
{
    using point_type = Point;
    static_assert(traits::is_point_v<Point>, "Point must satisfy Point concept");

    Point min{0., 0., 0.}, max{0., 0., 0.};

    /**
     * Containment predicate.
     * @return <code>true</code> if point <code>p</code> is contained in this AABB
     */
    template <class TPoint>
    bool contains(TPoint const& p) const
    {
        static_assert(traits::is_point_view_v<TPoint>, "TPoint must satisfy PointView concept");

        auto const greater_than_or_equal = [](TPoint const& p1, Point const& p2) -> bool {
            return p1.x() >= p2.x() && p1.y() >= p2.y() && p1.z() >= p2.z();
        };

        auto const less_than_or_equal = [](TPoint const& p1, Point const& p2) -> bool {
            return p1.x() <= p2.x() && p1.y() <= p2.y() && p1.z() <= p2.z();
        };

        return greater_than_or_equal(p, min) && less_than_or_equal(p, max);
    }

    /**
     * @return the center point of this AABB
     */
    Point center() const { return (min + max) / 2.f; }

    /**
     * Predicate for closest point from this AABB to the point <code>p</code>
     * in Euclidean space.
     * @param p The point closest to the returned point
     * @return The closest point from this AABB to the point <code>p</code>
     */
    template <class TPoint>
    Point nearest_point_from(TPoint const& p) const
    {
        static_assert(traits::is_point_view_v<TPoint>, "TPoint must satisfy PointView concept");
        Point nearest_point{p.x(), p.y(), p.z()};
        nearest_point.x(std::clamp(nearest_point.x(), min.x(), max.x()));
        nearest_point.y(std::clamp(nearest_point.y(), min.y(), max.y()));
        nearest_point.z(std::clamp(nearest_point.z(), min.z(), max.z()));

        return nearest_point;
    }
};

template <class ForwardIter, class Point>
inline axis_aligned_bounding_box_t<Point> bounding_box(ForwardIter begin, ForwardIter end)
{
    using aabb_type       = axis_aligned_bounding_box_t<Point>;
    using point_view_type = typename std::iterator_traits<ForwardIter>::value_type;

    static_assert(traits::is_point_v<Point>, "Point must satisfy Point concept");
    static_assert(
        traits::is_point_v<point_view_type>,
        "std::iterator_traits<ForwardIter>::value_type must satisfy PointView concept");

    aabb_type aabb;
    aabb.min.x(std::numeric_limits<typename Point::coordinate_type>::max());
    aabb.min.y(std::numeric_limits<typename Point::coordinate_type>::max());
    aabb.min.z(std::numeric_limits<typename Point::coordinate_type>::max());
    aabb.max.x(std::numeric_limits<typename Point::coordinate_type>::min());
    aabb.max.y(std::numeric_limits<typename Point::coordinate_type>::min());
    aabb.max.z(std::numeric_limits<typename Point::coordinate_type>::min());

    aabb = std::accumulate(begin, end, aabb, [](aabb_type& bbox, point_view_type const& p) {
        if (p.x() < bbox.min.x())
            bbox.min.x(p.x());
        if (p.y() < bbox.min.y())
            bbox.min.y(p.y());
        if (p.z() < bbox.min.z())
            bbox.min.z(p.z());
        if (p.x() > bbox.max.x())
            bbox.max.x(p.x());
        if (p.y() > bbox.max.y())
            bbox.max.y(p.y());
        if (p.z() > bbox.max.z())
            bbox.max.z(p.z());

        return bbox;
    });

    return aabb;
}

} // namespace pcp
