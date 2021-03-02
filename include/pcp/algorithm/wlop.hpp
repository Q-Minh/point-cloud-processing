#ifndef PCP_ALGORITHM_WLOP_HPP
#define PCP_ALGORITHM_WLOP_HPP

/**
 * @file
 * @ingroup algorithm
 */

#include "pcp/common/norm.hpp"
#include "pcp/common/points/point.hpp"
#include "pcp/common/sphere.hpp"
#include "pcp/common/vector3d.hpp"
#include "pcp/kdtree/linked_kdtree.hpp"
#include "pcp/traits/point_map.hpp"

#include <algorithm>
#include <cstddef>
#include <vector>

namespace pcp {
namespace algorithm {
namespace wlop {
namespace detail {

template <class PKdTree, class PCoordinateMap, class Theta, class ScalarType>
ScalarType compute_vj(
    std::size_t const j,
    ScalarType const h,
    PKdTree const& kdtree,
    PCoordinateMap const& p_cmap,
    Theta const& theta)
{
    using scalar_type = ScalarType;

    auto const& c1 = p_cmap(j);
    pcp::basic_point_t<scalar_type> pj{c1[0], c1[1], c1[2]};

    pcp::sphere_a<scalar_type> radial_support_region;
    radial_support_region.radius   = h;
    radial_support_region.position = c1;

    auto const neighbors = kdtree.range_search(radial_support_region);

    scalar_type vj{1.0};
    auto begin = neighbors.begin();
    auto end   = neighbors.end();
    for (auto it = begin; it != end; ++it)
    {
        auto const& c2 = p_cmap(*it);
        pcp::basic_point_t<scalar_type> pjp{c2[0], c2[1], c2[2]};

        auto const r = common::norm(pj - pjp);
        vj += theta(r);
    }

    return vj;
}

template <class QKdTree, class QCoordinateMap, class Theta, class ScalarType>
ScalarType compute_wi(
    std::size_t const i,
    ScalarType const h,
    QKdTree const& kdtree,
    QCoordinateMap const& q_cmap,
    Theta const& theta)
{
    using scalar_type = ScalarType;

    auto const& c1 = q_cmap(i);
    pcp::basic_point_t<scalar_type> qi{c1[0], c1[1], c1[2]};

    pcp::sphere_a<scalar_type> radial_support_region;
    radial_support_region.radius   = h;
    radial_support_region.position = c1;

    auto const neighbors = kdtree.range_search(radial_support_region);

    auto begin = neighbors.begin();
    auto end   = neighbors.end();
    scalar_type wi{1.0};
    for (auto it = begin; it != end; ++it)
    {
        auto const& c2 = q_cmap(*it);
        pcp::basic_point_t<scalar_type> qip{c2[0], c2[1], c2[2]};

        auto const r = common::norm(qi - qip);
        wi += theta(r);
    }

    return wi;
}

template <
    class PKdTree,
    class PCoordinateMap,
    class QCoordinateMap,
    class VjMap,
    class Theta,
    class ScalarType>
basic_point_t<ScalarType> solve_first_energy_median(
    std::size_t const i,
    ScalarType const h,
    PKdTree const& kdtree,
    PCoordinateMap const& p_cmap,
    QCoordinateMap const& q_cmap,
    VjMap const& vj_map,
    Theta const& theta)
{
    using scalar_type = ScalarType;

    pcp::sphere_a<scalar_type> radial_support_region;
    radial_support_region.radius   = h;
    radial_support_region.position = q_cmap(i);

    auto const neighbors = kdtree.range_search(radial_support_region);

    // loop over j in support region
    scalar_type sum{0.0};
    basic_point_t<scalar_type> median;
    auto begin = neighbors.begin();
    auto end   = neighbors.end();
    for (auto it = begin; it != end; ++it)
    {
        auto const& c1 = q_cmap(i);
        auto const& c2 = p_cmap(*it);

        basic_point_t<scalar_type> const q{c1[0], c1[1], c1[2]};
        basic_point_t<scalar_type> const p{c2[0], c2[1], c2[2]};

        scalar_type const r = common::norm(q - p);
        auto const alpha_ij = theta(r) / r;

        scalar_type const vj    = vj_map(*it);
        scalar_type const coeff = alpha_ij / vj;
        common::basic_vector3d_t<scalar_type> t{p.x(), p.y(), p.z()};
        t      = coeff * t;
        median = median + t;
        sum += coeff;
    }
    median = median / sum;

    return median;
}

template <class QKdTree, class QCoordinateMap, class WiMap, class Theta, class ScalarType>
common::basic_vector3d_t<ScalarType> solve_second_energy_repulsion_force(
    std::size_t const ip,
    ScalarType const h,
    ScalarType const mu,
    QKdTree const& kdtree,
    QCoordinateMap const& q_cmap,
    WiMap const& wi_map,
    Theta const& theta)
{
    using scalar_type = ScalarType;

    auto const& c1 = q_cmap(ip);
    basic_point_t<scalar_type> const qip{c1[0], c1[1], c1[2]};

    pcp::sphere_a<scalar_type> radial_support_region;
    radial_support_region.radius   = h;
    radial_support_region.position = c1;

    common::basic_vector3d_t<scalar_type> repulsion{0., 0., 0.};
    scalar_type sum{0.};
    auto const neighbors = kdtree.range_search(radial_support_region);
    auto begin           = neighbors.begin();
    auto end             = neighbors.end();
    for (auto it = begin; it != end; ++it)
    {
        auto const& c2 = q_cmap(*it);
        basic_point_t<scalar_type> const qi{c2[0], c2[1], c2[2]};

        common::basic_vector3d_t<scalar_type> const d = qip - qi;
        scalar_type const r                           = common::norm(d);
        scalar_type const wi                          = wi_map(*it);

        scalar_type const beta_ii = theta(r) / r; // * | d (eta(r)) / dr | = | -1 | = 1
        scalar_type const coeff   = wi * beta_ii;
        repulsion = repulsion + coeff * d;
        sum += coeff;
    }
    repulsion = (mu / sum) * repulsion;

    return repulsion;
}

} // namespace detail

/**
 * @brief
 * The WLOP algorithm's parameters
 * @ingroup point-cloud-simplification
 */
struct params_t
{
    std::size_t I = 0u;   ///< Size of resampled point cloud
    double mu     = 0.45; ///< Repulsion coefficient
    double h      = 0.;   ///< Radial functions' support
    std::size_t k = 10u;  ///< Number of solver iterations
    bool uniform  = true; ///< Use local densities to handle non-uniform point clouds. If uniform is
                          ///< false, LOP is performed instead.
};

/**
 * @ingroup point-cloud-simplification
 * @brief Simplifies an input point cloud using the WLOP operator.
 * The output point cloud is a simplified (smaller) point cloud with
 * uniform distribution.
 *
 * This implementation must maintain two copies of vectors of output points.
 *
 * Additionally, it uses two vectors of std::size_t to hold indices of the input point cloud
 * and the output point cloud.
 * A kd-tree over the output point cloud's points is created at
 * every iteration of the solver's loops.
 *
 * Two vectors of float|double are also maintained
 * for the non-uniform density weights if params.uniform == true.
 *
 * At each iteration, every new point of the output point cloud is computed
 * in parallel.
 *
 * @tparam OutputIter Iterator type dereferenceable to a type satisfying Point concept
 * @tparam PointMap Type satisfying PointMap concept
 * @tparam RandomAccessIter Iterator type satisfying Random Access requirements
 * @param begin Start iterator of input point cloud
 * @param end End iterator of input point cloud
 * @param out_begin Start iterator of output point cloud
 * @param point_map The point map property map
 * @param params The WLOP algorithm's parameters
 */
template <class RandomAccessIter, class OutputIter, class PointMap>
void wlop(
    RandomAccessIter begin,
    RandomAccessIter end,
    OutputIter out_begin,
    PointMap point_map,
    params_t const& params)
{
    using input_element_type = typename std::iterator_traits<RandomAccessIter>::value_type;
    using input_point_type   = std::invoke_result_t<PointMap, input_element_type>;
    using scalar_type        = typename input_point_type::coordinate_type;
    using output_point_type  = input_point_type;

    static_assert(
        traits::is_point_map_v<PointMap, input_element_type>,
        "point_map must satisfy PointMap concept");

    std::size_t const J  = static_cast<std::size_t>(std::distance(begin, end));
    std::size_t const I  = params.I;
    scalar_type const mu = static_cast<scalar_type>(params.mu);
    scalar_type const h  = static_cast<scalar_type>(params.h);
    std::size_t const K  = params.k;
    bool const uniform   = params.uniform;

    assert(I > 0u && J >= I);
    assert(mu >= 0. && mu <= .5);
    assert(K > 0u);

    std::vector<scalar_type> alpha(I);
    std::vector<scalar_type> beta(I);
    std::vector<scalar_type> vj(J, scalar_type{1.0});
    std::vector<scalar_type> wi(I, scalar_type{1.0});
    std::vector<output_point_type> x(I);
    std::vector<output_point_type> xp(I);

    std::vector<std::size_t> is(x.size());
    std::iota(is.begin(), is.end(), 0u);

    std::vector<std::size_t> js(J);
    std::iota(js.begin(), js.end(), 0u);

    scalar_type constexpr four2{4.0 * 4.0};
    scalar_type const h2               = h * h;
    scalar_type const h_over_4_squared = h2 / four2;

    auto const theta = [=](scalar_type const r) {
        return std::exp(-r / h_over_4_squared);
    };

    auto const p_coordinate_map = [&](std::size_t const j) {
        auto const it  = begin + j;
        auto const& pj = point_map(*it);
        return std::array<scalar_type, 3u>{pj.x(), pj.y(), pj.z()};
    };
    auto const q_coordinate_map = [&](std::size_t const i) {
        auto const& qi = x[i];
        return std::array<scalar_type, 3u>{qi.x(), qi.y(), qi.z()};
    };
    auto const vj_map = [&](std::size_t const j) {
        return vj[j];
    };
    auto const wi_map = [&](std::size_t const i) {
        return wi[i];
    };

    std::transform(begin + (J - I), end, x.begin(), [&](input_element_type const& e) {
        auto const& p = point_map(e);
        return output_point_type{p.x(), p.y(), p.z()};
    });

    kdtree::construction_params_t kdtree_params;
    kdtree_params.compute_max_depth     = true;
    kdtree_params.construction          = kdtree::construction_t::nth_element;
    kdtree_params.max_elements_per_leaf = 64u;

    basic_linked_kdtree_t<std::size_t, 3u, decltype(p_coordinate_map)> p_kdtree{
        js.begin(),
        js.end(),
        p_coordinate_map,
        kdtree_params};

    std::transform(js.begin(), js.end(), vj.begin(), [&](std::size_t const j) {
        return detail::compute_vj(j, h, p_kdtree, p_coordinate_map, theta);
    });

    for (std::size_t k = 0u; k < K; ++k)
    {
        basic_linked_kdtree_t<std::size_t, 3u, decltype(q_coordinate_map)> q_kdtree{
            is.begin(),
            is.end(),
            q_coordinate_map,
            kdtree_params};

        std::transform(is.begin(), is.end(), wi.begin(), [&](std::size_t const i) {
            return detail::compute_wi(i, h, q_kdtree, q_coordinate_map, theta);
        });

        std::transform(is.begin(), is.end(), xp.begin(), [&](std::size_t const ip) {
            basic_point_t<scalar_type> const median = detail::solve_first_energy_median(
                ip,
                h,
                p_kdtree,
                p_coordinate_map,
                q_coordinate_map,
                vj_map,
                theta);

            common::basic_vector3d_t<scalar_type> const repulsion =
                detail::solve_second_energy_repulsion_force(
                    ip,
                    h,
                    mu,
                    q_kdtree,
                    q_coordinate_map,
                    wi_map,
                    theta);

            return output_point_type{
                median.x() + repulsion.x(),
                median.y() + repulsion.y(),
                median.z() + repulsion.z()};
        });

        // swap buffers
        std::copy(xp.begin(), xp.end(), x.begin());
    }

    std::copy(xp.begin(), xp.end(), out_begin);
}

} // namespace wlop
} // namespace algorithm
} // namespace pcp

#endif // PCP_ALGORITHM_WLOP_HPP