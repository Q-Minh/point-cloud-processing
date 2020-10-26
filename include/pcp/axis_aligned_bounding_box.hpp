#pragma once

#include "traits/point_traits.hpp"

#include <algorithm>
#include <array>

namespace pcp {

template <class Point>
struct axis_aligned_bounding_box_t
{
    static_assert(traits::is_point_v<Point>, "Point must satisfy Point concept");

    Point min{}, max{};

    /**
     * Containment predicate.
     * @return <code>true</code> if point <code>p</code> is contained in this AABB
     */
    bool contains(Point const& p) const
    {
        auto const greater_than_or_equal = [](Point const& p1, Point const& p2) -> bool {
            return p1.x() >= p2.x() && p1.y() >= p2.y() && p1.z() >= p2.z();
        };

        auto const less_than_or_equal = [](Point const& p1, Point const& p2) -> bool {
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
    Point nearest_point_from(Point const& p) const
    {
        Point nearest_point = p;
        nearest_point.x()   = std::clamp(nearest_point.x(), min.x(), max.x());
        nearest_point.y()   = std::clamp(nearest_point.y(), min.y(), max.y());
        nearest_point.z()   = std::clamp(nearest_point.z(), min.z(), max.z());

        return nearest_point;
    }
};

} // namespace pcp
