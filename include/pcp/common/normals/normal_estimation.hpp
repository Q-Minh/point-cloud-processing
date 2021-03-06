#ifndef PCP_COMMON_NORMALS_NORMAL_ESTIMATION_HPP
#define PCP_COMMON_NORMALS_NORMAL_ESTIMATION_HPP

/**
 * @file
 * @ingroup common
 */

#include "pcp/common/normals/normal.hpp"
#include "pcp/traits/point_map.hpp"
#include "pcp/traits/point_traits.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <cstdint>
#include <iterator>

namespace pcp {

/**
 * @ingroup common
 * @brief
 * Estimates the normal from a group of points using PCA.
 * @tparam ForwardIter Type of iterator to the points
 * @tparam PointViewMap Type satisfying PointViewMap concept
 * @tparam Normal Type of the normal to return
 * @param it Begin iterator to the points
 * @param end End iterator to the points
 * @param point_map The point view map property map
 * @return
 */
template <class ForwardIter, class PointViewMap, class Normal = pcp::normal_t>
Normal estimate_normal(ForwardIter it, ForwardIter end, PointViewMap const& point_map)
{
    using normal_type = Normal;

    static_assert(
        traits::is_point_view_map_v<PointViewMap, decltype(*it)>,
        "Type of point_map must satisfy PointViewMap concept");

    auto const n = std::distance(it, end);
    Eigen::Matrix3Xf V;
    V.resize(3, n);
    for (auto i = 0; i < n; ++i, ++it)
    {
        auto p              = point_map(*it);
        V.block(0, i, 3, 1) = Eigen::Vector3f(p.x(), p.y(), p.z());
    }

    Eigen::Vector3f const Mu      = V.rowwise().mean();
    Eigen::Matrix3Xf const Vprime = V.colwise() - Mu;
    Eigen::Matrix3f const Cov     = Vprime * Vprime.transpose();
    Eigen::SelfAdjointEigenSolver<decltype(Cov)> A(Cov);
    auto const l  = A.eigenvalues();
    auto const& X = A.eigenvectors();

    normal_type normal;
    // instead of sorting, just use 3 if statements
    // First eigenvalue is smallest
    if (l(0) <= l(1) && l(0) <= l(2))
    {
        normal = {X(0, 0), X(1, 0), X(2, 0)};
    }
    // Second eigenvalue is smallest
    if (l(1) <= l(0) && l(1) <= l(2))
    {
        normal = {X(0, 1), X(1, 1), X(2, 1)};
    }
    // Third eigenvalue is smallest
    if (l(2) <= l(0) && l(2) <= l(1))
    {
        normal = {X(0, 2), X(1, 2), X(2, 2)};
    }

    // normal is already normalized, since Eigen returns
    // normalized eigenvectors
    return normal;
}

} // namespace pcp

#endif // PCP_COMMON_NORMALS_NORMAL_ESTIMATION_HPP
