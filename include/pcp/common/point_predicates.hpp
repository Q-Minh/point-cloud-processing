#pragma once

#include <cmath>

namespace pcp {

/**
 * @brief Equality test of 2 points by coordinates within given precision.
 * @tparam PointView1 Type satisfying PointView concept
 * @tparam PointView2 Type satisfying PointView concept
 * @param p1 Point 1
 * @param p2 Point 2
 * @param eps Precision (margin of error)
 * @return
 */
template <class PointView1, class PointView2>
bool are_points_equal(
    PointView1 const& p1,
    PointView2 const& p2,
    typename PointView1::component_type eps =
        static_cast<typename PointView1::component_type>(1e-5))
{
    using component_type    = typename PointView1::component_type;
    component_type const dx = std::abs(p1.x() - p2.x());
    component_type const dy = std::abs(p1.y() - p2.y());
    component_type const dz = std::abs(p1.z() - p2.z());
    bool const equals       = (dx < eps) && (dy < eps) && (dz < eps);
    return equals;
}

} // namespace pcp
