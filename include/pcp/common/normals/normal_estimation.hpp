#ifndef PCP_COMMON_NORMALS_NORMAL_ESTIMATION_HPP
#define PCP_COMMON_NORMALS_NORMAL_ESTIMATION_HPP

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <cstdint>
#include <iterator>
#include <pcp/common/normals/normal.hpp>
#include <pcp/traits/point_traits.hpp>

namespace pcp {

template <class ForwardIter, class Normal = pcp::normal_t>
Normal estimate_normal(ForwardIter it, ForwardIter end)
{
    using point_view_type = typename std::iterator_traits<ForwardIter>::value_type;
    using normal_type     = Normal;

    static_assert(
        traits::is_point_view_v<point_view_type>,
        "Type of dereferenced ForwardIter must satisfy PointView concept");

    auto const n = std::distance(it, end);
    Eigen::Matrix3Xf V;
    V.resize(3, n);
    for (auto i = 0; i < n; ++i, ++it)
    {
        V.block(0, i, 3, 1) = Eigen::Vector3f(it->x(), it->y(), it->z());
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