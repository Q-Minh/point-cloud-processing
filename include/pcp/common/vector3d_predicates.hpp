#pragma once

#include <cmath>

namespace pcp {
namespace common {

template <class T>
bool floating_point_equals(T v1, T v2, T eps = static_cast<T>(1e-5))
{
    T const d = std::abs(v1 - v2);
    return d < eps;
}

/**
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
    using component_type = typename Vector3d1::component_type;
    bool const equals    = floating_point_equals(v1.x(), v2.x(), eps) &&
                        floating_point_equals(v1.y(), v2.y(), eps) &&
                        floating_point_equals(v1.z(), v2.z(), eps);
    return equals;
}

} // namespace common
} // namespace pcp
