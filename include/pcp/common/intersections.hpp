#ifndef PCP_COMMON_INTERSECTIONS_HPP
#define PCP_COMMON_INTERSECTIONS_HPP

/**
 * @file
 * @ingroup common
 */

#include "axis_aligned_bounding_box.hpp"
#include "norm.hpp"
#include "sphere.hpp"

namespace pcp {
namespace intersections {

/**
 * @ingroup intersection-tests
 * @brief
 * Intersections test between AABBs
 * @tparam Point
 * @param b1
 * @param b2
 * @return
 */
template <class Point>
inline bool intersects(
    axis_aligned_bounding_box_t<Point> const& b1,
    axis_aligned_bounding_box_t<Point> const& b2)
{
    return (b1.max.x() >= b2.min.x() && b1.max.y() >= b2.min.y() && b1.max.z() >= b2.min.z()) &&
           (b1.min.x() <= b2.max.x() && b1.min.y() <= b2.max.y() && b1.min.z() <= b2.max.z());
}

/**
 * @ingroup intersection-tests
 * @brief
 * Intersections test between AABBs
 * @tparam CoordinateType
 * @param b1
 * @param b2
 * @return
 */
template <class CoordinateType, std::size_t K>
inline bool intersects(
    kd_axis_aligned_bounding_box_t<CoordinateType, K> const& b1,
    kd_axis_aligned_bounding_box_t<CoordinateType, K> const& b2)
{
    for (size_t i = 0; i < K; ++i)
    {
        if (!(b1.max[i] >= b2.min[i] && b1.min[i] <= b2.max[i]))
            return false;
    }
    return true;
}

/**
 * @ingroup intersection-tests
 * @brief
 * Intersection test between spheres
 * @tparam Point
 * @param s1
 * @param s2
 * @return
 */
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

/**
 * @ingroup intersection-tests
 * @brief
 * Intersection test between AABB and sphere
 * @tparam Point
 * @param b
 * @param s
 * @return
 */
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

/**
 * @ingroup intersection-tests
 * @brief
 * Intersection test between kd-AABB and sphere
 * @tparam CoordinateType
 * @param b
 * @param s
 * @return
 */
template <class CoordinateType>
inline bool intersects(
    kd_axis_aligned_bounding_box_t<CoordinateType, 3> const& b,
    sphere_a<CoordinateType> const& s)
{
    auto const center = s.center();

    bool const is_center_in_box_x = center[0] >= b.min[0] && center[0] <= b.max[0];
    bool const is_center_in_box_y = center[1] >= b.min[1] && center[1] <= b.max[1];
    bool const is_center_in_box_z = center[2] >= b.min[2] && center[2] <= b.max[2];
    bool const is_center_in_box   = is_center_in_box_x && is_center_in_box_y && is_center_in_box_z;

    if (is_center_in_box)
        return true;

    auto const nearest_point_on_box_from_sphere = b.nearest_point_from(center);
    return common::squared_distance(nearest_point_on_box_from_sphere, center) <= s.radius;
}

/**
 * @ingroup intersection-tests
 * @brief
 * Intersection test between kd-AABB and sphere
 * @tparam CoordinateType
 * @param s
 * @param b
 * @return
 */
template <class CoordinateType>
inline bool intersects(
    sphere_a<CoordinateType> const& s,
    kd_axis_aligned_bounding_box_t<CoordinateType, 3> const& b)
{
    return intersects(b, s);
}

/**
 * @ingroup intersection-tests
 * @brief
 * Intersection test between sphere and AABB
 * @tparam Point
 * @param s
 * @param b
 * @return
 */
template <class Point>
inline bool intersects(sphere_t<Point> const& s, axis_aligned_bounding_box_t<Point> const& b)
{
    return intersects(b, s);
}

} // namespace intersections
} // namespace pcp

#endif // PCP_COMMON_INTERSECTIONS_HPP