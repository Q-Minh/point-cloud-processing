#pragma once

#include "axis_aligned_bounding_box.hpp"
#include "pcp/common/norm.hpp"
#include "sphere.hpp"

namespace pcp {
namespace intersections {

template <class Point>
inline bool intersects(
    axis_aligned_bounding_box_t<Point> const& b1,
    axis_aligned_bounding_box_t<Point> const& b2)
{
    return (b1.max.x() >= b2.min.x() && b1.max.y() >= b2.min.y() && b1.max.z() >= b2.min.z()) &&
           (b1.min.x() <= b2.max.x() && b1.min.y() <= b2.max.y() && b1.min.z() <= b2.max.z());
}

template <class Point>
inline bool intersects(sphere_t<Point> const& s1, sphere_t<Point> const& s2)
{
    auto const c1 = s1.center();
    auto const c2 = s2.center();

    auto const maximum_distance_between_spheres = (s1.radius + s2.radius);
    auto const maximum_squared_distance_between_spheres =
        maximum_distance_between_spheres * maximum_distance_between_spheres;

    return common::squared_distance(c1, c2) <= maximum_squared_distance_between_spheres;
}

template <class Point>
inline bool intersects(axis_aligned_bounding_box_t<Point> const& b, sphere_t<Point> const& s)
{
    Point const center = s.center();

    bool const is_center_in_box_x = center.x() >= b.min.x() && center.x() <= b.max.x();
    bool const is_center_in_box_y = center.y() >= b.min.y() && center.y() <= b.max.y();
    bool const is_center_in_box_z = center.z() >= b.min.z() && center.z() <= b.max.z();
    bool const is_center_in_box   = is_center_in_box_x && is_center_in_box_y && is_center_in_box_z;

    if (is_center_in_box)
        return true;

    Point const nearest_point_on_box_from_sphere = b.nearest_point_from(center);
    return common::squared_distance(nearest_point_on_box_from_sphere, center) <= s.radius;
}

template <class Point>
inline bool intersects(sphere_t<Point> const& s, axis_aligned_bounding_box_t<Point> const& b)
{
    return intersects(b, s);
}

} // namespace intersections
} // namespace pcp