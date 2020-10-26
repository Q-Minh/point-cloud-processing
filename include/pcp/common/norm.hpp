#pragma once

#include "pcp/traits/point_traits.hpp"
#include "pcp/traits/vector3d_traits.hpp"

#include <cmath>

namespace pcp {
namespace common {

template <class Vector3d>
static inline typename Vector3d::component_type
inner_product(Vector3d const& v1, Vector3d const& v2)
{
    static_assert(traits::is_vector3d_v<Vector3d>, "Vector3d must satisfy Vector3d concept");

    auto const xx = v2.x() * v1.x();
    auto const yy = v2.y() * v1.y();
    auto const zz = v2.z() * v1.z();

    return xx + yy + zz;
}

struct l2
{
};

template <class Vector3d, class Norm = l2>
typename Vector3d::component_type norm(Vector3d const& v1, Vector3d const& v2, Norm const&);

template <class Vector3d, class Norm>
static inline typename Vector3d::component_type norm(Vector3d const& v, l2 const&)
{
    static_assert(traits::is_vector3d_v<Vector3d>, "Vector3d must satisfy Vector3d concept");

    auto const xx = v.x() * v.x();
    auto const yy = v.y() * v.y();
    auto const zz = v.z() * v.z();

    return std::sqrt(xx + yy + zz);
}

/**
 * The l2-norm is defined as sqrt(x*x + y*y + z*z), but we don't always need
 * the sqrt computation for distance comparisons:
 *
 * Having points p1 and p2, we have that:
 * sqrt((p1.x-p.x)^2 + (p1.y-p.y)^2 + (p1.z-p.z)^2) < sqrt((p2.x-p.x)^2 + (p2.y-p.y)^2 +
 * (p2.z-p.z)^2) is equivalent to (p1.x-p.x)^2 + (p1.y-p.y)^2 + (p1.z-p.z)^2 < (p2.x-p.x)^2
 * + (p2.y-p.y)^2 + (p2.z-p.z)^2
 *
 * since we are squaring both sides of the equation.
 */
template <class Point>
static inline typename Point::coordinate_type squared_distance(Point const& p1, Point const& p2)
{
    static_assert(traits::is_point_v<Point>, "Point must satisfy Point concept");
    auto const& diff = p2 - p1;
    return inner_product(diff, diff);
}

} // namespace common
} // namespace pcp