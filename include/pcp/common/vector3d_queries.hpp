#ifndef PCP_COMMON_VECTOR3D_QUERIES_HPP
#define PCP_COMMON_VECTOR3D_QUERIES_HPP

/**
 * @file
 * @ingroup common
 */

#include "pcp/traits/point_map.hpp"
#include "pcp/traits/vector3d_traits.hpp"

#include <cmath>
#include <iterator>
#include <numeric>
#include <type_traits>

namespace pcp {
namespace common {

/**
 * @ingroup common
 * @brief
 * Equality test for floating point types (float, double)
 * @tparam T Type of the floating point numbers
 * @param v1 left operand
 * @param v2 right operand
 * @param eps Error tolerance
 * @return True if |v1 - v2| < eps
 */
template <class T>
bool floating_point_equals(T v1, T v2, T eps = static_cast<T>(1e-5))
{
    T const d = std::abs(v1 - v2);
    return d < eps;
}

/**
 * @ingroup common-vector3
 * @brief Equality test of 2 points by coordinates within given precision.
 * @tparam Vector3d1 Type satisfying PointView concept
 * @tparam Vector3d2 Type satisfying PointView concept
 * @param v1 Vector 1
 * @param v2 Vector3 2
 * @param eps Precision (margin of error)
 * @return
 */
template <class Vector3d1, class Vector3d2>
bool are_vectors_equal(
    Vector3d1 const& v1,
    Vector3d2 const& v2,
    typename Vector3d1::component_type eps = static_cast<typename Vector3d1::component_type>(1e-5))
{
    // TODO:
    // These requirements are too restrictive. We only want to compare x,y,z components.
    // Maybe move this function to a point_queries.hpp file and have type safety check
    // for only points or point_views.
    /*static_assert(traits::is_vector3d_v<Vector3d1>, "Vector3d1 must satisfy Vector3d concept");
    static_assert(traits::is_vector3d_v<Vector3d2>, "Vector3d2 must satisfy Vector3d concept");*/

    bool const equals = floating_point_equals(v1.x(), v2.x(), eps) &&
                        floating_point_equals(v1.y(), v2.y(), eps) &&
                        floating_point_equals(v1.z(), v2.z(), eps);
    return equals;
}

/**
 * @ingroup common
 * @brief Returns the center of geometry of a range of 3d vectors
 * @tparam ForwardIter Type of iterator to Vector3d
 * @tparam PointMap Type satisfying PointMap concept
 * @tparam Vector3d Type of the 3d vector to return as the center of geometry
 * @param begin Iterator to start of the sequence of elements
 * @param end Iterator to one past the end of the sequence of elements
 * @param point_map The point map property map
 * @return The center of geometry of the range as a Vector3d
 */
template <class ForwardIter, class PointMap>
std::invoke_result_t<PointMap, typename std::iterator_traits<ForwardIter>::value_type>
center_of_geometry(ForwardIter begin, ForwardIter end, PointMap const& point_map)
{
    using key_type = decltype(*begin);
    static_assert(
        traits::is_point_map_v<PointMap, key_type>,
        "point_map must satisfy PointMap concept");

    using point_type =
        std::invoke_result_t<PointMap, typename std::iterator_traits<ForwardIter>::value_type>;
    using component_type = typename point_type::coordinate_type;

    auto const reduce_op = [&](point_type current, key_type const& value) {
        auto point = point_map(value);
        return current + point_type{point.x(), point.y(), point.z()};
    };

    auto const n   = std::distance(begin, end);
    auto const sum = std::accumulate(begin, end, point_type{}, reduce_op);
    auto const np  = static_cast<component_type>(n);
    return sum / np;
}

} // namespace common
} // namespace pcp

#endif // PCP_COMMON_VECTOR3D_QUERIES_HPP