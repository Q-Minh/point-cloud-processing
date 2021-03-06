#ifndef PCP_ALGORITHM_COVARIANCE_HPP
#define PCP_ALGORITHM_COVARIANCE_HPP

/**
 * @file
 * @ingroup algorithm
 */

#include "pcp/common/vector3d_queries.hpp"
#include "pcp/traits/point_map.hpp"

#include <Eigen/Core>

namespace pcp {
namespace algorithm {

template <class ForwardIter, class PointMap>
Eigen::Matrix3d covariance(ForwardIter begin, ForwardIter end, PointMap const& point_map)
{
    using element_type = typename std::iterator_traits<ForwardIter>::value_type;

    static_assert(
        traits::is_point_map_v<PointMap, element_type>,
        "point_map must satisfy PointMap concept");

    using point_type  = std::invoke_result_t<PointMap, element_type>;
    using scalar_type = typename point_type::coordinate_type;

    /**
     * element 0 : x*x
     * element 1 : y*y
     * element 2 : z*z
     * element 3 : x*y
     * element 4 : x*z
     * element 5 : y*z
     */
    std::array<scalar_type, 6u> cov{0.};

    auto const mu = common::center_of_geometry(begin, end, point_map);
    for (auto it = begin; it != end; ++it)
    {
        auto const& p = point_map(*it);
        auto const pp = p - mu;

        auto const xx = pp.x() * pp.x();
        auto const yy = pp.y() * pp.y();
        auto const zz = pp.z() * pp.z();
        auto const xy = pp.x() * pp.y();
        auto const xz = pp.x() * pp.z();
        auto const yz = pp.y() * pp.z();

        cov[0] += xx;
        cov[1] += yy;
        cov[2] += zz;
        cov[3] += xy;
        cov[4] += xz;
        cov[5] += yz;
    }

    Eigen::Matrix3d Cov;
    Cov(0, 0) = cov[0];
    Cov(1, 1) = cov[1];
    Cov(2, 2) = cov[2];

    Cov(0, 1) = cov[3];
    Cov(0, 2) = cov[4];
    Cov(1, 2) = cov[5];
    Cov(1, 0) = Cov(0, 1);
    Cov(2, 0) = Cov(0, 2);
    Cov(2, 1) = Cov(1, 2);

    return Cov;
}

} // namespace algorithm
} // namespace pcp

#endif // PCP_ALGORITHM_COVARIANCE_HPP