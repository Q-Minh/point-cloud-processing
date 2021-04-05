#ifndef PCP_ALGORITHM_RESAMPLE_AWAY_FROM_EDGES_HPP
#define PCP_ALGORITHM_RESAMPLE_AWAY_FROM_EDGES_HPP

#include "pcp/common/norm.hpp"
#include "pcp/kdtree/linked_kdtree.hpp"
#include "pcp/traits/normal_map.hpp"
#include "pcp/traits/output_iterator_traits.hpp"
#include "pcp/traits/point_map.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <execution>
#include <numeric>
#include <random>
#include <vector>

namespace pcp {
namespace algorithm {
namespace ear {

struct normal_smoothing_params_t
{
    double sigmap;            ///< Radial function support radius
    double sigman = 0.261799; ///< Angle parameter scaling the similarity of neighboring normals,
                              ///< default = 15 degrees
    std::size_t K = 1u;       ///< Number of iterations
};

struct wlop_params_t
{
    std::size_t I = 0u;   ///< Size of resampled point cloud
    double mu     = 0.45; ///< Repulsion coefficient
    double h      = 0.;   ///< Radial functions' support
    std::size_t K = 1u;   ///< Number of solver iterations
    bool uniform  = true; ///< Use local densities to handle non-uniform point clouds. If uniform is
                          ///< false, LOP is performed instead.
};

template <class RandomAccessIter, class OutputIter, class PointMap, class NormalMap>
OutputIter smooth_normals(
    RandomAccessIter begin,
    RandomAccessIter end,
    OutputIter out_begin,
    PointMap const& point_map,
    NormalMap const& normal_map,
    normal_smoothing_params_t const& params)
{
    using input_element_type = typename std::iterator_traits<RandomAccessIter>::value_type;
    using input_point_type   = std::invoke_result_t<PointMap, input_element_type>;
    using input_normal_type  = std::invoke_result_t<NormalMap, input_element_type>;
    using scalar_type        = typename input_point_type::coordinate_type;
    using output_normal_type = typename xstd::output_iterator_traits<OutputIter>::value_type;

    static_assert(
        traits::is_point_map_v<PointMap, input_element_type>,
        "point_map must satisfy PointMap concept");

    static_assert(
        traits::is_normal_map_v<NormalMap, input_element_type>,
        "normal_map must satisfy NormalMap concept");

    std::size_t const N      = static_cast<std::size_t>(std::distance(begin, end));
    std::size_t const K      = params.K;
    scalar_type const sigmap = static_cast<scalar_type>(params.sigmap);
    scalar_type const sigman = static_cast<scalar_type>(params.sigman);

    std::vector<std::size_t> indices(N);
    std::iota(indices.begin(), indices.end(), 0u);

    std::vector<input_normal_type> prev_normals(N);
    std::transform(begin, end, prev_normals.begin(), [&](input_element_type const& e) {
        return normal_map(e);
    });
    std::vector<input_normal_type> next_normals(N);

    auto const internal_point_map = [&](std::size_t const i) {
        auto const it = std::next(begin, i);
        return point_map(*it);
    };
    auto const internal_normal_map = [&](std::size_t const i) {
        return prev_normals[i];
    };

    auto const coordinate_map = [&](std::size_t const i) {
        auto const it = std::next(begin, i);
        auto const& p = point_map(*it);
        return std::array<scalar_type, 3u>{p.x(), p.y(), p.z()};
    };

    auto const theta = [sigma = sigmap](scalar_type const r) {
        scalar_type const r2    = r * r;
        scalar_type const s2    = sigma * sigma;
        scalar_type const power = -(r2 / s2);
        return std::exp(power);
    };

    using vector_3d_type = Eigen::Matrix<scalar_type, 3, 1>;

    scalar_type constexpr zero{0.};
    scalar_type constexpr eps{1e-6};

    auto const psi = [sigma = sigman](input_normal_type const& ni, input_normal_type const& nip) {
        scalar_type constexpr one{1.};
        scalar_type const num =
            one - common::inner_product(
                      common::basic_vector3d_t<scalar_type>{ni.x(), ni.y(), ni.z()},
                      common::basic_vector3d_t<scalar_type>{nip.x(), nip.y(), nip.z()});
        scalar_type const den    = one - std::cos(sigma);
        scalar_type const power  = num / den;
        scalar_type const power2 = power * power;
        scalar_type const wpsi   = std::exp(-power2);
        return wpsi;
    };

    kdtree::construction_params_t kdtree_params{};
    kdtree_params.min_element_count_for_parallel_exec = N; // Don't parallelize ctor
    kdtree_params.compute_max_depth                   = true;
    basic_linked_kdtree_t<std::size_t, 3u, decltype(coordinate_map)> kdtree{
        indices.begin(),
        indices.end(),
        coordinate_map,
        kdtree_params};

    for (std::size_t k = 0u; k < K; ++k)
    {
        auto const get_updated_normal = [&](std::size_t const i) {
            auto const& ci              = coordinate_map(i);
            input_point_type const& pi  = internal_point_map(i);
            input_normal_type const& ni = internal_normal_map(i);

            sphere_a<scalar_type> radial_support_region{};
            radial_support_region.radius   = sigmap;
            radial_support_region.position = ci;

            auto const neighbours = kdtree.range_search(radial_support_region);

            scalar_type normalizing_coefficient{0.};
            vector_3d_type ni_new{0., 0., 0.};
            for (auto const& neighbour : neighbours)
            {
                // if (neighbour == i)
                //    continue;

                input_point_type const& pip  = internal_point_map(neighbour);
                input_normal_type const& nip = internal_normal_map(neighbour);

                // if (common::are_vectors_equal(pi, pip, eps))
                //    continue;

                scalar_type const r      = common::norm(pi - pip);
                scalar_type const wtheta = theta(r);
                scalar_type const wpsi   = psi(ni, nip);
                scalar_type coeff        = wtheta * wpsi;

                ni_new += coeff * vector_3d_type{nip.nx(), nip.ny(), nip.nz()};
                normalizing_coefficient += coeff;
            }

            if (common::floating_point_equals(normalizing_coefficient, zero, eps))
                return ni;

            ni_new /= normalizing_coefficient;
            return input_normal_type{ni_new.x(), ni_new.y(), ni_new.z()};
        };

        std::transform(
            std::execution::par,
            indices.begin(),
            indices.end(),
            next_normals.begin(),
            get_updated_normal);

        // if (k < K - 1u)
        //{
        std::copy(next_normals.begin(), next_normals.end(), prev_normals.begin());
        //}
    }

    return std::copy(next_normals.begin(), next_normals.end(), out_begin);
}

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
    scalar_type constexpr eps = static_cast<scalar_type>(1e-9);

    auto begin = neighbors.begin();
    auto end   = neighbors.end();
    for (auto it = begin; it != end; ++it)
    {
        auto const& c2 = p_cmap(*it);
        pcp::basic_point_t<scalar_type> pjp{c2[0], c2[1], c2[2]};

        if (common::are_vectors_equal(pj, pjp, eps))
            continue;

        auto const r = common::norm(pj, pjp);
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

    scalar_type wi{1.0};
    scalar_type constexpr eps = static_cast<scalar_type>(1e-9);

    auto begin = neighbors.begin();
    auto end   = neighbors.end();
    for (auto it = begin; it != end; ++it)
    {
        auto const& c2 = q_cmap(*it);
        pcp::basic_point_t<scalar_type> qip{c2[0], c2[1], c2[2]};

        if (common::are_vectors_equal(qi, qip, eps))
            continue;

        auto const r = common::norm(qi, qip);
        wi += theta(r);
    }

    return wi;
}

} // namespace detail

template <class RandomAccessIter, class OutputIter, class PointMap, class NormalMap>
OutputIter wlop(
    RandomAccessIter begin,
    RandomAccessIter end,
    OutputIter out_begin,
    PointMap const& point_map,
    NormalMap const& normal_map,
    wlop_params_t const& params)
{
    using input_element_type = typename std::iterator_traits<RandomAccessIter>::value_type;
    using input_point_type   = std::invoke_result_t<PointMap, input_element_type>;
    using input_normal_type  = std::invoke_result_t<NormalMap, input_element_type>;
    using scalar_type        = typename input_point_type::coordinate_type;
    using output_point_type  = typename xstd::output_iterator_traits<OutputIter>::value_type;
    using difference_type    = typename std::iterator_traits<RandomAccessIter>::difference_type;

    static_assert(
        traits::is_point_map_v<PointMap, input_element_type>,
        "point_map must satisfy PointMap concept");

    static_assert(
        traits::is_normal_map_v<NormalMap, input_element_type>,
        "normal_map must satisfy NormalMap concept");

    static_assert(
        traits::is_point_v<output_point_type>,
        "out_begin must be iterator to a type satisfying the Point concept");

    std::size_t const J  = static_cast<std::size_t>(std::distance(begin, end));
    std::size_t const I  = params.I;
    std::size_t const K  = params.K;
    scalar_type const mu = static_cast<scalar_type>(params.mu);
    scalar_type const h  = static_cast<scalar_type>(params.h);
    bool const uniform   = params.uniform;

    scalar_type constexpr zero{0.};
    scalar_type constexpr one{1.};
    scalar_type constexpr half{.5};

    assert(I > 0u && J >= I);
    assert(mu >= zero && mu <= half);
    assert(K >= 0u);

    std::vector<std::size_t> is(I);
    std::iota(is.begin(), is.end(), 0u);

    std::vector<std::size_t> js(J);
    std::iota(js.begin(), js.end(), 0u);

    std::vector<input_point_type> prev_points(I);
    std::vector<input_point_type> next_points(I);
    std::vector<input_normal_type> normals(I);

    // shuffle initial point set and take a subset of it as our set I
    {
        std::random_device rd{};
        std::mt19937 generator{rd()};
        std::shuffle(js.begin(), js.end(), generator);

        auto const offset = static_cast<difference_type>(J) - static_cast<difference_type>(I);
        std::transform(
            js.begin() + offset,
            js.end(),
            prev_points.begin(),
            [&](std::size_t const j) {
                auto const it = std::next(begin, j);
                return point_map(*it);
            });
        std::transform(js.begin() + offset, js.end(), normals.begin(), [&](std::size_t const j) {
            auto const it = std::next(begin, j);
            return normal_map(*it);
        });
        std::copy(prev_points.begin(), prev_points.end(), next_points.begin());
    }

    std::vector<scalar_type> vj(J, one);
    std::vector<scalar_type> wi(I, one);

    using coordinates_type      = std::array<scalar_type, 3u>;
    auto const q_coordinate_map = [&](std::size_t const i) {
        auto const& qi = prev_points[i];
        return coordinates_type{qi.x(), qi.y(), qi.z()};
    };
    auto const p_coordinate_map = [&](std::size_t const j) {
        auto const it  = std::next(begin, j);
        auto const& pj = point_map(*it);
        return coordinates_type{pj.x(), pj.y(), pj.z()};
    };
    auto const q_point_map = [&](std::size_t const i) {
        return prev_points[i];
    };
    auto const p_point_map = [&](std::size_t const j) {
        auto const it = std::next(begin, j);
        return point_map(*it);
    };
    auto const q_normal_map = [&](std::size_t const i) {
        return normals[i];
    };
    auto const wi_map = [&](std::size_t const i) {
        return wi[i];
    };
    auto const vj_map = [&](std::size_t const j) {
        return vj[j];
    };

    scalar_type constexpr eps{1e-6};
    scalar_type constexpr four2{4.0 * 4.0};

    scalar_type const h2               = h * h;
    scalar_type const h_over_4_squared = h2 / four2;
    auto const theta                   = [sigma2 = h_over_4_squared](scalar_type const r) {
        scalar_type const r2    = r * r;
        scalar_type const power = -(r2 / sigma2);
        scalar_type wtheta      = std::exp(power);
        return wtheta;
    };

    using vector_3d_type = Eigen::Matrix<scalar_type, 3, 1>;
    auto const phi       = [sigma = h_over_4_squared](
                         input_normal_type const& ni,
                         input_point_type const& qi,
                         input_point_type const& pj) {
        vector_3d_type const d{qi.x() - pj.x(), qi.y() - pj.y(), qi.z() - pj.z()};
        vector_3d_type const n{ni.nx(), ni.ny(), ni.nz()};
        scalar_type const num   = n.dot(d);
        scalar_type const num2  = num * num;
        scalar_type const s2    = sigma * sigma;
        scalar_type const power = -num2 / s2;
        scalar_type wphi        = std::exp(power);
        return wphi;
    };

    kdtree::construction_params_t kdtree_params{};
    kdtree_params.min_element_count_for_parallel_exec = J; // Don't parallelize ctor
    kdtree_params.compute_max_depth                   = true;

    basic_linked_kdtree_t<std::size_t, 3u, decltype(p_coordinate_map)> p_kdtree{
        js.begin(),
        js.end(),
        p_coordinate_map,
        kdtree_params};

    if (uniform)
    {
        std::transform(
            std::execution::par,
            js.begin(),
            js.end(),
            vj.begin(),
            [&](std::size_t const j) {
                return detail::compute_vj(j, h, p_kdtree, p_coordinate_map, theta);
            });
    }

    for (std::size_t k = 0u; k < K; ++k)
    {
        basic_linked_kdtree_t<std::size_t, 3u, decltype(q_coordinate_map)> q_kdtree{
            is.begin(),
            is.end(),
            q_coordinate_map,
            kdtree_params};

        if (uniform)
        {
            std::transform(
                std::execution::par,
                is.begin(),
                is.end(),
                wi.begin(),
                [&](std::size_t const i) {
                    return detail::compute_wi(i, h, q_kdtree, q_coordinate_map, theta);
                });
        }

        auto const get_updated_point = [&](std::size_t const i) {
            input_point_type const& qi  = q_point_map(i);
            input_normal_type const& ni = q_normal_map(i);
            coordinates_type const& ci  = q_coordinate_map(i);

            sphere_a<scalar_type> radial_support_region{};
            radial_support_region.position = ci;
            radial_support_region.radius   = h;

            auto const p_neighbours = p_kdtree.range_search(radial_support_region);
            auto const q_neighbours = q_kdtree.range_search(radial_support_region);

            scalar_type constexpr eps{1e-6};

            scalar_type first_energy_normalization_coefficient{zero};
            vector_3d_type first_energy_sum{zero, zero, zero};
            for (std::size_t const& neighbour : p_neighbours)
            {
                if (neighbour == i)
                    continue;

                input_point_type const& pj = p_point_map(neighbour);

                if (common::are_vectors_equal(qi, pj, eps))
                    continue;

                scalar_type const r = common::norm(qi - pj);

                scalar_type const wphi     = phi(ni, qi, pj);
                scalar_type const alpha_ij = wphi / r;
                scalar_type const coeff    = alpha_ij / vj[neighbour];

                first_energy_sum += coeff * vector_3d_type{pj.x(), pj.y(), pj.z()};
                first_energy_normalization_coefficient += coeff;
            }
            first_energy_sum /= first_energy_normalization_coefficient;

            if (common::floating_point_equals(first_energy_normalization_coefficient, zero, eps))
                first_energy_sum = vector_3d_type{qi.x(), qi.y(), qi.z()};

            scalar_type second_energy_normalization_coefficient{zero};
            vector_3d_type second_energy_sum{zero, zero, zero};
            for (std::size_t const& neighbour : q_neighbours)
            {
                if (neighbour == i)
                    continue;

                input_point_type const& qip = q_point_map(neighbour);

                if (common::are_vectors_equal(qi, qip, eps))
                    continue;

                auto const repulsion = qi - qip;
                scalar_type const r  = common::norm(repulsion);

                scalar_type const wtheta = theta(r);
                scalar_type constexpr deta{1.};
                scalar_type const beta_ii = (wtheta / r) * deta;
                scalar_type const coeff   = beta_ii * wi[neighbour];

                second_energy_sum +=
                    coeff * vector_3d_type{repulsion.x(), repulsion.y(), repulsion.z()};
                second_energy_normalization_coefficient += coeff;
            }
            second_energy_sum /= second_energy_normalization_coefficient;

            if (common::floating_point_equals(second_energy_normalization_coefficient, zero, eps))
                second_energy_sum.setZero();

            vector_3d_type const correction = first_energy_sum + mu * second_energy_sum;
            return input_point_type{correction.x(), correction.y(), correction.z()};
        };

        std::transform(
            std::execution::par,
            is.begin(),
            is.end(),
            next_points.begin(),
            get_updated_point);

        // if (k < K - 1u)
        std::copy(next_points.begin(), next_points.end(), prev_points.begin());
    }

    return std::copy(next_points.begin(), next_points.end(), out_begin);
}

template <
    class RandomAccessIter,
    class PointOutputIter,
    class NormalOutputIter,
    class PointMap,
    class NormalMap>
std::pair<PointOutputIter, NormalOutputIter> resample_away_from_edges(
    RandomAccessIter begin,
    RandomAccessIter end,
    PointOutputIter point_out_begin,
    NormalOutputIter normal_out_begin,
    PointMap const& point_map,
    NormalMap const& normal_map,
    normal_smoothing_params_t const& normal_smoothing_params,
    wlop_params_t const& wlop_params)
{
    using input_element_type = typename std::iterator_traits<RandomAccessIter>::value_type;
    using input_point_type   = std::invoke_result_t<PointMap, input_element_type>;
    using input_normal_type  = std::invoke_result_t<NormalMap, input_element_type>;
    using scalar_type        = typename input_point_type::coordinate_type;
    using output_point_type  = typename xstd::output_iterator_traits<PointOutputIter>::value_type;
    using output_normal_type = typename xstd::output_iterator_traits<NormalOutputIter>::value_type;
    using difference_type    = typename std::iterator_traits<RandomAccessIter>::difference_type;

    static_assert(
        traits::is_point_map_v<PointMap, input_element_type>,
        "point_map must satisfy PointMap concept");

    static_assert(
        traits::is_normal_map_v<NormalMap, input_element_type>,
        "normal_map must satisfy NormalMap concept");

    static_assert(
        traits::is_point_v<output_point_type>,
        "point_out_begin must be iterator to type satisfying Point concept");

    static_assert(
        traits::is_normal_v<output_normal_type>,
        "normal_out_begin must be iterator to type satisfying Normal concept");

    std::size_t const N = static_cast<difference_type>(std::distance(begin, end));
    std::vector<std::size_t> indices(N);
    std::iota(indices.begin(), indices.end(), 0u);

    std::vector<input_point_type> points(N);
    std::transform(begin, end, points.begin(), [&](input_element_type const& e) {
        return point_map(e);
    });
    std::vector<input_normal_type> normals(N);
    std::transform(begin, end, normals.begin(), [&](input_element_type const& e) {
        return normal_map(e);
    });

    std::size_t const K                                             = normal_smoothing_params.K;
    ear::normal_smoothing_params_t internal_normal_smoothing_params = normal_smoothing_params;
    internal_normal_smoothing_params.K                              = 1u;
    ear::wlop_params_t internal_wlop_params                         = wlop_params;
    internal_wlop_params.K                                          = 1u;

    for (std::size_t k = 0u; k < K; ++k)
    {
        auto const internal_point_map = [&](std::size_t const i) {
            return points[i];
        };
        auto const internal_normal_map = [&](std::size_t const i) {
            return normals[i];
        };

        ear::smooth_normals(
            indices.begin(),
            indices.end(),
            normals.begin(),
            internal_point_map,
            internal_normal_map,
            internal_normal_smoothing_params);

        ear::wlop(
            indices.begin(),
            indices.end(),
            points.begin(),
            internal_point_map,
            internal_normal_map,
            internal_wlop_params);
    }

    auto point_it  = std::copy(points.begin(), points.end(), point_out_begin);
    auto normal_it = std::copy(normals.begin(), normals.end(), normal_out_begin);

    return {point_it, normal_it};
}

} // namespace ear
} // namespace algorithm
} // namespace pcp

#endif // PCP_ALGORITHM_RESAMPLE_AWAY_FROM_EDGES_HPP