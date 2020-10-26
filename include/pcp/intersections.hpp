#pragma once

#include "axis_aligned_bounding_box.hpp"
#include "sphere.hpp"

namespace pcp {
namespace intersections {

template <class Point>
static inline typename Point::coordinate_type squared_distance(Point const& p1, Point const& p2)
{
    static_assert(traits::is_point_v<Point>, "Point must satisfy Point concept");

    auto const dx = p2.x() - p1.x();
    auto const dy = p2.y() - p1.y();
    auto const dz = p2.z() - p1.z();

    return dx * dx + dy * dy + dz * dz;
}

template <class Point>
inline bool intersects(axis_aligned_bounding_box_t<Point> const& b1, axis_aligned_bounding_box_t<Point> const& b2)
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

    return squared_distance(c1, c2) <= maximum_squared_distance_between_spheres;
}

template <class Point>
inline bool intersects(axis_aligned_bounding_box_t<Point> const& b, sphere_t<Point> const& s)
{
    point_t const center = s.center();

    bool const is_center_in_box_x = center.x() >= b.min.x() && center.x() <= b.max.x();
    bool const is_center_in_box_y = center.y() >= b.min.y() && center.y() <= b.max.y();
    bool const is_center_in_box_z = center.z() >= b.min.z() && center.z() <= b.max.z();
    bool const is_center_in_box   = is_center_in_box_x && is_center_in_box_y && is_center_in_box_z;

    if (is_center_in_box)
        return true;

    point_t const nearest_point_on_box_from_sphere = b.nearest_point_from(center);
    return squared_distance(nearest_point_on_box_from_sphere, center) <= s.radius;
}

template <class Point>
inline bool intersects(sphere_t<Point> const& s, axis_aligned_bounding_box_t<Point> const& b)
{
    return intersects(b, s);
}

} // namespace intersections
} // namespace pcp