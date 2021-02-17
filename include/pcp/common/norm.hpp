#ifndef PCP_COMMON_NORM_HPP
#define PCP_COMMON_NORM_HPP

/**
 * @file
 * @ingroup common
 */

#include "pcp/traits/point_traits.hpp"
#include "pcp/traits/vector3d_traits.hpp"

#include <cmath>

namespace pcp {
namespace common {

/**
 * @ingroup common-vector3
 * @brief
 * Standard inner product for 3d vectors satisfying Vector3D concept
 * @tparam Vector3d1
 * @tparam Vector3d2
 * @param v1
 * @param v2
 * @return
 */
template <class Vector3d1, class Vector3d2>
inline typename Vector3d1::component_type inner_product(Vector3d1 const& v1, Vector3d2 const& v2)
{
    static_assert(
        traits::is_vector3d_v<Vector3d1> && traits::is_vector3d_v<Vector3d2>,
        "Vector3d1 and Vector3d2 must satisfy Vector3d concept");

    auto const xx = v2.x() * v1.x();
    auto const yy = v2.y() * v1.y();
    auto const zz = v2.z() * v1.z();

    return xx + yy + zz;
}

struct l2
{
};

/**
 * @ingroup common-vector3
 * @brief
 * Standard norm computation for Vector3D types
 * @tparam Vector3d Type satisfying Vector3d concept
 * @tparam Norm Type of norm to compute
 * @param v The vector from which we want the norm
 * @return
 */
template <class Vector3d, class Norm = l2>
typename Vector3d::component_type norm(Vector3d const& v, Norm const& = Norm{})
{
    static_assert(traits::is_vector3d_v<Vector3d>, "Vector3d must satisfy Vector3d concept");

    if constexpr (std::is_same_v<Norm, l2>)
    {
        auto const xx = v.x() * v.x();
        auto const yy = v.y() * v.y();
        auto const zz = v.z() * v.z();

        return std::sqrt(xx + yy + zz);
    }
    else
    {
        // if the norm is unknown, default to l2
        auto const xx = v.x() * v.x();
        auto const yy = v.y() * v.y();
        auto const zz = v.z() * v.z();

        return std::sqrt(xx + yy + zz);
    }
}

/**
 * @ingroup common-vector3
 * @brief
 * The l2-norm is defined as sqrt(x*x + y*y + z*z), but we don't always need
 * the sqrt computation for distance comparisons:
 *
 * Having points p1 and p2, we have that:
 * sqrt((p1.x-p.x)^2 + (p1.y-p.y)^2 + (p1.z-p.z)^2) < sqrt((p2.x-p.x)^2 + (p2.y-p.y)^2 +
 * (p2.z-p.z)^2) is equivalent to (p1.x-p.x)^2 + (p1.y-p.y)^2 + (p1.z-p.z)^2 < (p2.x-p.x)^2
 * + (p2.y-p.y)^2 + (p2.z-p.z)^2
 *
 * since we are squaring both sides of the equation.
 * @tparam Point1
 * @tparam Point2
 * @param p1
 * @param p2
 * @return
 */
template <class Point1, class Point2>
inline typename Point1::coordinate_type squared_distance(Point1 const& p1, Point2 const& p2)
{
    static_assert(
        traits::is_point_view_v<Point1> && traits::is_point_view_v<Point2>,
        "Point1 and Point2 must satisfy PointView concept");
    auto const dx = p2.x() - p1.x();
    auto const dy = p2.y() - p1.y();
    auto const dz = p2.z() - p1.z();
    return dx * dx + dy * dy + dz * dz;
}

/**
 * @ingroup common-vector3
 * @brief
 * The l2-norm is defined as sqrt(x*x + y*y + z*z), but we don't always need
 * the sqrt computation for distance comparisons:
 *
 * Having points p1 and p2, we have that:
 * sqrt((p1.x-p.x)^2 + (p1.y-p.y)^2 + (p1.z-p.z)^2) < sqrt((p2.x-p.x)^2 + (p2.y-p.y)^2 +
 * (p2.z-p.z)^2) is equivalent to (p1.x-p.x)^2 + (p1.y-p.y)^2 + (p1.z-p.z)^2 < (p2.x-p.x)^2
 * + (p2.y-p.y)^2 + (p2.z-p.z)^2
 *
 * since we are squaring both sides of the equation.
 * @tparam Type
 * @param p1
 * @param p2
 * @return
 */
template <class Type>
inline typename Type squared_distance(std::array<Type, 3> const& p1, std::array<Type, 3> const& p2)
{
    auto const dx = p2[0] - p1[0];
    auto const dy = p2[1] - p1[1];
    auto const dz = p2[2] - p1[2];
    return dx * dx + dy * dy + dz * dz;
}

} // namespace common
} // namespace pcp

#endif // PCP_COMMON_NORM_HPP